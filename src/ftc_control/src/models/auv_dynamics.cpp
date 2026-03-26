#include "ftc_control/models/auv_dynamics.hpp"
#include <cmath>

namespace ftc_control {
namespace models {

AUVDynamics::AUVDynamics() {
    Params bluerov2_params;

    // Mass properties
    bluerov2_params.mass = 11.5;  // kg

    // Inertia tensor (kg*m^2)
    bluerov2_params.Ib << 0.16, -0.001, -0.001,
                          -0.001, 0.25, -0.0001,
                          -0.001, -0.0001, 0.28;

    // Added mass [Xu̇, Yv̇, Zẇ, Kṗ, Mq̇, Nṙ]
    bluerov2_params.Ma << 5.5, 12.7, 14.6, 0.12, 0.12, 0.12;

    // Linear damping [Xu, Yv, Zw, Kp, Mq, Nr]
    bluerov2_params.Dl << 4.0, 6.2, 5.2, 0.07, 0.07, 0.07;

    // Quadratic damping [X|u|u, Y|v|v, Z|w|w, K|p|p, M|q|q, N|r|r]
    bluerov2_params.Dq << 18.2, 21.7, 36.8, 1.5, 1.5, 1.5;

    // Buoyancy and weight
    bluerov2_params.weight = 112.8;    // N
    bluerov2_params.buoyancy = 114.8;  // N (slightly positive for safety)

    // CoG and CoB positions
    bluerov2_params.r_g = Eigen::Vector3d(0.0, 0.0, 0.02);   // CoG: slightly below center
    bluerov2_params.r_b = Eigen::Vector3d(0.0, 0.0, -0.01);  // CoB: slightly above center

    setParams(bluerov2_params);
}

AUVDynamics::AUVDynamics(const Params& params) {
    setParams(params);
}

void AUVDynamics::setParams(const Params& params) {
    params_ = params;
    updateMassMatrix();
}

// M = M_RB + M_A
void AUVDynamics::updateMassMatrix() {
    double m = params_.mass;
    double xG = params_.r_g(0);
    double yG = params_.r_g(1);
    double zG = params_.r_g(2);

    // Rigid body mass matrix
    Eigen::Matrix<double, DOF, DOF> MRB;
    MRB.setZero();

    MRB(0, 0) = m;
    MRB(1, 1) = m;
    MRB(2, 2) = m;

    // Linear-angular coupling (CoG != origin)
    MRB(0, 4) = m * zG;   MRB(0, 5) = -m * yG;
    MRB(1, 3) = -m * zG;  MRB(1, 5) = m * xG;
    MRB(2, 3) = m * yG;   MRB(2, 4) = -m * xG;

    // Symmetric counterparts
    MRB(4, 0) = m * zG;   MRB(5, 0) = -m * yG;
    MRB(3, 1) = -m * zG;  MRB(5, 1) = m * xG;
    MRB(3, 2) = m * yG;   MRB(4, 2) = -m * xG;

    // Inertia tensor
    MRB.block<3, 3>(3, 3) = params_.Ib;

    // Added mass (diagonal approximation): M_A = diag(Xu̇, Yv̇, Zẇ, Kṗ, Mq̇, Nṙ)
    Eigen::Matrix<double, DOF, DOF> MA;
    MA.setZero();
    for (int i = 0; i < DOF; ++i) {
        MA(i, i) = params_.Ma(i);
    }

    // Total mass and precomputed inverse
    M_ = MRB + MA;
    M_inv_ = M_.inverse();
}

Eigen::Matrix<double, AUVDynamics::DOF, AUVDynamics::DOF>
AUVDynamics::getMassMatrix() const {
    return M_;
}

// C(ν) = C_RB(ν) + C_A(ν)
// Property: C(ν) is skew-symmetric → ν^T · C(ν) · ν = 0
Eigen::Matrix<double, AUVDynamics::DOF, AUVDynamics::DOF>
AUVDynamics::getCoriolisMatrix(const Eigen::Matrix<double, DOF, 1>& nu) const {
    double u = nu(0), v = nu(1), w = nu(2);
    double p = nu(3), q = nu(4), r = nu(5);

    double m = params_.mass;
    double xG = params_.r_g(0);
    double yG = params_.r_g(1);
    double zG = params_.r_g(2);

    double Ixx = params_.Ib(0, 0);
    double Iyy = params_.Ib(1, 1);
    double Izz = params_.Ib(2, 2);
    double Ixy = params_.Ib(0, 1);
    double Ixz = params_.Ib(0, 2);
    double Iyz = params_.Ib(1, 2);

    double Xu = params_.Ma(0);
    double Yv = params_.Ma(1);
    double Zw = params_.Ma(2);
    double Kp = params_.Ma(3);
    double Mq = params_.Ma(4);
    double Nr = params_.Ma(5);

    // Rigid body Coriolis
    Eigen::Matrix<double, DOF, DOF> CRB;
    CRB.setZero();

    // Angular velocity x linear momentum coupling
    CRB(3, 0) = -(m * (q * yG + r * zG));
    CRB(3, 1) = m * (p * yG + w);
    CRB(3, 2) = m * (p * zG - v);

    CRB(4, 0) = m * (q * xG - w);
    CRB(4, 1) = -(m * (p * xG + r * zG));
    CRB(4, 2) = m * (q * zG + u);

    CRB(5, 0) = m * (r * xG + v);
    CRB(5, 1) = m * (r * yG - u);
    CRB(5, 2) = -(m * (p * xG + q * yG));

    // Gyroscopic effects
    CRB(3, 4) = -q * Iyz - p * Ixz + r * Izz;
    CRB(3, 5) = r * Iyz + p * Ixy - q * Iyy;

    CRB(4, 3) = q * Iyz + p * Ixz - r * Izz;
    CRB(4, 5) = -(r * Ixz + q * Ixy - p * Ixx);

    CRB(5, 3) = -(r * Iyz + p * Ixy - q * Iyy);
    CRB(5, 4) = r * Ixz + q * Ixy - p * Ixx;

    // Added mass Coriolis
    Eigen::Matrix<double, DOF, DOF> CA;
    CA.setZero();

    CA(0, 4) = -Zw * w;
    CA(0, 5) = Yv * v;
    CA(1, 3) = Zw * w;
    CA(1, 5) = -Xu * u;
    CA(2, 3) = -Yv * v;
    CA(2, 4) = Xu * u;

    CA(3, 1) = -Zw * w;
    CA(3, 2) = Yv * v;
    CA(4, 0) = Zw * w;
    CA(4, 2) = -Xu * u;
    CA(5, 0) = -Yv * v;
    CA(5, 1) = Xu * u;

    // Added inertia coupling
    CA(3, 4) = -Nr * r;
    CA(3, 5) = Mq * q;
    CA(4, 3) = Nr * r;
    CA(4, 5) = -Kp * p;
    CA(5, 3) = -Mq * q;
    CA(5, 4) = Kp * p;

    return CRB + CA;
}

// Damping: D(ν) = D_L + D_Q * diag(|ν|)
Eigen::Matrix<double, AUVDynamics::DOF, AUVDynamics::DOF>
AUVDynamics::getDampingMatrix(const Eigen::Matrix<double, DOF, 1>& nu) const {
    Eigen::Matrix<double, DOF, DOF> D;
    D.setZero();

    for (int i = 0; i < DOF; ++i) {
        // D_ii = D_linear_i + D_quadratic_i * |velocity_i|
        D(i, i) = params_.Dl(i) + params_.Dq(i) * std::abs(nu(i));
    }

    return D;
}

// Restoring forces: g(η) from weight/buoyancy interaction
// CoB above CoG creates self-righting moment
Eigen::Matrix<double, AUVDynamics::DOF, 1>
AUVDynamics::getRestoringForces(const Eigen::Matrix<double, DOF, 1>& eta) const {
    double phi = eta(3);
    double theta = eta(4);

    double W = params_.weight;
    double B = params_.buoyancy;

    double xG = params_.r_g(0), yG = params_.r_g(1), zG = params_.r_g(2);
    double xB = params_.r_b(0), yB = params_.r_b(1), zB = params_.r_b(2);

    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double cth = std::cos(theta);
    double sth = std::sin(theta);

    Eigen::Matrix<double, DOF, 1> g;
    g.setZero();

    // Restoring forces
    g(0) = (W - B) * sth;
    g(1) = -(W - B) * cth * sphi;
    g(2) = -(W - B) * cth * cphi;

    // Restoring moments (from CoG-CoB offset)
    g(3) = -(yG * W - yB * B) * cth * cphi + (zG * W - zB * B) * cth * sphi;
    g(4) = (zG * W - zB * B) * sth + (xG * W - xB * B) * cth * cphi;
    g(5) = -(xG * W - xB * B) * cth * sphi - (yG * W - yB * B) * sth;

    return g;
}

// Forward dynamics: ν̇ = M⁻¹ · (τ - C(ν)·ν - D(ν)·ν - g(η))
Eigen::Matrix<double, AUVDynamics::DOF, 1>
AUVDynamics::computeAcceleration(
    const Eigen::Matrix<double, DOF, 1>& eta,
    const Eigen::Matrix<double, DOF, 1>& nu,
    const Eigen::Matrix<double, DOF, 1>& tau) const {

    Eigen::Matrix<double, DOF, DOF> C = getCoriolisMatrix(nu);
    Eigen::Matrix<double, DOF, DOF> D = getDampingMatrix(nu);
    Eigen::Matrix<double, DOF, 1> g = getRestoringForces(eta);

    return M_inv_ * (tau - C * nu - D * nu - g);
}

// Kinematic Jacobian: η̇ = J(η) · ν
// J(η) = [R(Θ)  0; 0  T(Θ)] where R is body-to-NED rotation, T is Euler rate transform
// Singular at θ = ±90° (gimbal lock)
Eigen::Matrix<double, AUVDynamics::DOF, AUVDynamics::DOF>
AUVDynamics::getJacobian(const Eigen::Matrix<double, DOF, 1>& eta) const {
    double phi = eta(3);
    double theta = eta(4);
    double psi = eta(5);

    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double cth = std::cos(theta);
    double sth = std::sin(theta);
    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);

    Eigen::Matrix<double, DOF, DOF> J;
    J.setZero();

    // Rotation matrix R (body to NED, ZYX Euler convention)
    J(0, 0) = cpsi * cth;
    J(0, 1) = -spsi * cphi + cpsi * sth * sphi;
    J(0, 2) = spsi * sphi + cpsi * sth * cphi;

    J(1, 0) = spsi * cth;
    J(1, 1) = cpsi * cphi + spsi * sth * sphi;
    J(1, 2) = -cpsi * sphi + spsi * sth * cphi;

    J(2, 0) = -sth;
    J(2, 1) = cth * sphi;
    J(2, 2) = cth * cphi;

    // Euler rate transformation T
    double sec_th = 1.0 / (cth + 1e-10);  // singularity protection

    J(3, 3) = 1.0;
    J(3, 4) = sphi * sth * sec_th;
    J(3, 5) = cphi * sth * sec_th;

    J(4, 4) = cphi;
    J(4, 5) = -sphi;

    J(5, 4) = sphi * sec_th;
    J(5, 5) = cphi * sec_th;

    return J;
}

}  // namespace models
}  // namespace ftc_control
