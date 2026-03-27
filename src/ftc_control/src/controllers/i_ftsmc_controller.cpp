#include "ftc_control/controllers/i_ftsmc_controller.hpp"

namespace ftc_control {
namespace controllers {

IFTSMCController::IFTSMCController() {
    reset();
}

void IFTSMCController::reset() {
    s_nu_integral_.setZero();
    nu_c_old_.setZero();
}

double IFTSMCController::sig(double x, double beta) const {
    if (std::abs(x) < 1e-6) return 0.0;
    return std::pow(std::abs(x), beta) * ((x > 0) - (x < 0));
}

Eigen::Matrix<double, 6, 6> IFTSMCController::getRotationMatrix(const Eigen::Matrix<double, 6, 1>& eta) const {
    Eigen::Matrix<double, 3, 3> R_b_w;
    double phi = eta(3), theta = eta(4), psi = eta(5);
    double cphi = std::cos(phi), sphi = std::sin(phi);
    double cthe = std::cos(theta), sthe = std::sin(theta);
    double cpsi = std::cos(psi), spsi = std::sin(psi);

    R_b_w << cpsi*cthe, cpsi*sthe*sphi - spsi*cphi, cpsi*sthe*cphi + spsi*sphi,
             spsi*cthe, spsi*sthe*sphi + cpsi*cphi, spsi*sthe*cphi - cpsi*sphi,
             -sthe,     cthe*sphi,                  cthe*cphi;

    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J.block<3,3>(0,0) = R_b_w;
    J.block<3,3>(3,3) = Eigen::Matrix<double, 3, 3>::Identity();
    return J;
}

Eigen::Matrix<double, 6, 1> IFTSMCController::compute_i_ftsmc(
    const Eigen::Matrix<double, 6, 1>& eta,
    const Eigen::Matrix<double, 6, 1>& eta_d,
    const Eigen::Matrix<double, 6, 1>& dot_eta_d,
    const Eigen::Matrix<double, 6, 1>& nu,
    double dt)
{
    // Tuning parameters
    const double beta = 0.99;

    Eigen::Matrix<double, 6, 1> Ki_diag, Gamma_diag;
    Ki_diag    << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Gamma_diag << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    Eigen::Matrix<double, 6, 1> w1_diag, w2_diag;
    w1_diag << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    w2_diag << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    // Kinematic Control
    Eigen::Matrix<double, 6, 1> e_eta = eta_d - eta;
    while (e_eta(5) > M_PI)  e_eta(5) -= 2.0 * M_PI;
    while (e_eta(5) < -M_PI) e_eta(5) += 2.0 * M_PI;

    Eigen::Matrix<double, 6, 6> J = getRotationMatrix(eta);
    Eigen::Matrix<double, 6, 1> sig_e_eta;
    for(int i=0; i<6; ++i) sig_e_eta(i) = Ki_diag(i) * sig(e_eta(i), beta);
    Eigen::Matrix<double, 6, 1> nu_c = J.inverse() * (dot_eta_d + 10 * sig_e_eta);
    Eigen::Matrix<double, 6, 1> dot_nu_c = (nu_c - nu_c_old_) / dt;
    nu_c_old_ = nu_c;

    Eigen::Matrix<double, 6, 1> e_nu = nu_c - nu;

    // PI-Sig Sliding Surface with anti-windup clamping
    Eigen::Matrix<double, 6, 1> sig_e_nu;
    for(int i=0; i<6; ++i) sig_e_nu(i) = Gamma_diag(i) * sig(e_nu(i), beta);

    s_nu_integral_ += (e_nu + sig_e_nu) * dt;
    for(int i=0; i<6; ++i) s_nu_integral_(i) = std::clamp(s_nu_integral_(i), -1.0, 1.0);

    Eigen::Matrix<double, 6, 1> s_nu = e_nu + Ki_diag.asDiagonal() * s_nu_integral_;

    // Hydrodynamic Model (BlueROV2 Heavy)
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

    // Coriolis (C_RB + C_A)
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

    // Control Law: tau_eq (equivalent) + tau_sw (switching)
    Eigen::Matrix<double, 6, 1> dynamic_comp = Ki_diag.asDiagonal() * (e_nu + sig_e_nu);
    Eigen::Matrix<double, 6, 1> tau_eq = (C * nu) + (D * nu) + g + M * (dot_nu_c + dynamic_comp);

    Eigen::Matrix<double, 6, 1> tau_sw_comp;
    for(int i=0; i<6; ++i) {
        tau_sw_comp(i) = w1_diag(i) * sig(s_nu(i), beta) + w2_diag(i) * s_nu(i);
    }
    Eigen::Matrix<double, 6, 1> tau_sw = M * tau_sw_comp;

    return applyLimits(tau_eq + tau_sw);
}

Eigen::Matrix<double, 6, 1> IFTSMCController::applyLimits(const Eigen::Matrix<double, 6, 1>& tau) const {
    Eigen::Matrix<double, 6, 1> limited = tau;
    for (int i = 0; i < 3; ++i)
        limited(i) = std::clamp(limited(i), -limits_.max_force, limits_.max_force);
    for (int i = 3; i < 6; ++i)
        limited(i) = std::clamp(limited(i), -limits_.max_torque, limits_.max_torque);
    return limited;
}

} // namespace controllers
} // namespace ftc_control
