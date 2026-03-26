#include "ftc_control/controllers/backstepping.hpp"
#include <cmath>
#include <algorithm>

namespace ftc_control {
namespace controllers {

backstepping::backstepping() {
    reset();
    limits_.max_force = 120.0;
    limits_.max_torque = 30.0;
}

void backstepping::setLimits(const Limits& limits) {
    limits_ = limits;
}

void backstepping::reset() {
    tau_.setZero();
    nu_c_old_.setZero();
}

Eigen::Matrix<double, 6, 6> backstepping::getRotationMatrix(double phi, double theta, double psi) {
    Eigen::Matrix<double, 3, 3> R_b_w;
    double cphi = std::cos(phi), sphi = std::sin(phi);
    double cthe = std::cos(theta), sthe = std::sin(theta);
    double cpsi = std::cos(psi), spsi = std::sin(psi);

    R_b_w << cpsi*cthe, cpsi*sthe*sphi - spsi*cphi, cpsi*sthe*cphi + spsi*sphi,
             spsi*cthe, spsi*sthe*sphi + cpsi*cphi, spsi*sthe*cphi - cpsi*sphi,
             -sthe,     cthe*sphi,                 cthe*cphi;

    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J.block<3,3>(0,0) = R_b_w;
    J.block<3,3>(3,3) = Eigen::Matrix<double, 3, 3>::Identity();
    return J;
}

Eigen::Matrix<double, 6, 1> backstepping::compute_backstepping_full(
    const Eigen::Matrix<double, 6, 1>& eta,
    const Eigen::Matrix<double, 6, 1>& eta_d,
    const Eigen::Matrix<double, 6, 1>& eta_d_dot,
    const Eigen::Matrix<double, 6, 1>& nu,
    const Eigen::DiagonalMatrix<double, 6>& K1,
    const Eigen::DiagonalMatrix<double, 6>& K2) 
{
    // Kinematic Control
    Eigen::Matrix<double, 6, 1> e_eta = eta - eta_d;
    while (e_eta(5) > M_PI) e_eta(5) -= 2.0 * M_PI;
    while (e_eta(5) < -M_PI) e_eta(5) += 2.0 * M_PI;

    Eigen::Matrix<double, 6, 6> J = getRotationMatrix(eta(3), eta(4), eta(5));
    
    double dt = 0.02;
    Eigen::Matrix<double, 6, 1> nu_c = J.inverse() * (eta_d_dot - (K1 * e_eta));
    Eigen::Matrix<double, 6, 1> nu_c_dot = (nu_c - nu_c_old_) / dt;
    nu_c_old_ = nu_c;

    Eigen::Matrix<double, 6, 1> e_nu = nu - nu_c;

    // Hydrodynamic Model
    const double m = 11.5;
    const double W = 112.8, B = 114.8;
    const double xb = 0, yb = 0, zb = 0.02;
    const double xg = 0, yg = 0, zg = -0.01;
    const double Ixx = 0.16, Iyy = 0.25, Izz = 0.28;
    const double Ixy = -0.001, Iyz = -0.0001, Ixz = -0.001;
    const double Xu_dot = 5.5, Yv_dot = 12.7, Zw_dot = 14.6;
    const double Kp_dot = 0.12, Mq_dot = 0.12, Nr_dot = 0.12;

    double u = nu(0), v = nu(1), w = nu(2), p = nu(3), q = nu(4), r = nu(5);
    double phi = eta(3), theta = eta(4), psi = eta(5);

    // Mass Matrix (M_RB + M_A)
    Eigen::Matrix<double, 6, 6> M_RB;
    M_RB << m, 0, 0, 0, m*zg, -m*yg,
            0, m, 0, -m*zg, 0, m*xg,
            0, 0, m, m*yg, -m*xg, 0,
            0, -m*zg, m*yg, Ixx, -Ixy, -Ixz,
            m*zg, 0, -m*xg, -Ixy, Iyy, -Iyz,
            -m*yg, m*xg, 0, -Ixz, -Iyz, Izz;
            
    Eigen::Matrix<double, 6, 1> Ma_diag;
    Ma_diag << Xu_dot, Yv_dot, Zw_dot, Kp_dot, Mq_dot, Nr_dot;

    Eigen::Matrix<double, 6, 6> M = M_RB;
    M.diagonal() += Ma_diag;

    // Coriolis Matrix (C_RB + C_A)
    Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
    C(0,4) = m*w; C(0,5) = -m*v; C(1,3) = -m*w; C(1,5) = m*u; C(2,3) = m*v; C(2,4) = -m*u;
    C(3,1) = m*w; C(3,2) = -m*v; C(4,0) = -m*w; C(4,2) = m*u; C(5,0) = m*v; C(5,1) = -m*u;
    C(0,4) += -Zw_dot*w; C(0,5) += Yv_dot*v; C(1,3) += Zw_dot*w; C(1,5) += -Xu_dot*u;

    // Damping (D)
    Eigen::Matrix<double, 6, 1> Dl(4.0, 6.2, 5.2, 0.07, 0.07, 0.07);
    Eigen::Matrix<double, 6, 1> Dnl(18.2, 21.7, 36.8, 1.5, 1.5, 1.5);
    Eigen::Matrix<double, 6, 6> D = Dl.asDiagonal() + (Dnl.array() * nu.array().abs()).matrix().asDiagonal();

    // Restoring Forces
    Eigen::Matrix<double, 6, 1> g;
    g << (W-B)*sin(theta),
         -(W-B)*cos(theta)*sin(phi),
         -(W-B)*cos(theta)*cos(phi),
         -(yg*W-yb*B)*cos(theta)*cos(phi) + (zg*W-zb*B)*cos(theta)*sin(phi),
         (zg*W-zb*B)*sin(theta) + (xg*W-xb*B)*cos(theta)*cos(phi),
         -(xg*W-xb*B)*cos(theta)*sin(phi) - (yg*W-yb*B)*sin(theta);

    // Control Law: tau = M*nu_c_dot + D*nu + g(eta) - J^T*e_eta - K2*e_nu
    Eigen::Matrix<double, 6, 1> tau_c = (M * nu_c_dot) + (D * nu) + (C * nu) + g - M * (J.transpose() * e_eta) - M * (K2 * e_nu);

    tau_ = applyLimits(tau_c);
    return tau_;
}

Eigen::Matrix<double, 6, 1> backstepping::applyLimits(const Eigen::Matrix<double, 6, 1>& tau) const {
    Eigen::Matrix<double, 6, 1> limited = tau;
    for (int i = 0; i < 3; ++i) 
        limited(i) = std::clamp(limited(i), -limits_.max_force, limits_.max_force);
    for (int i = 3; i < 6; ++i) 
        limited(i) = std::clamp(limited(i), -limits_.max_torque, limits_.max_torque);
    return limited;
}

} // namespace controllers
} // namespace ftc_control