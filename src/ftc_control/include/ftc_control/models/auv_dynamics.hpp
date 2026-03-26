// 6-DOF AUV dynamic model based on Fossen's marine vehicle formulation.

#ifndef FTC_CONTROL_MODELS_AUV_DYNAMICS_HPP_
#define FTC_CONTROL_MODELS_AUV_DYNAMICS_HPP_

#include <Eigen/Dense>

namespace ftc_control {
namespace models {

/// @brief 6-DOF AUV dynamic model (Fossen formulation)
class AUVDynamics {
public:
    static constexpr int DOF = 6;

    struct Params {
        double mass = 143.0;  // kg
        Eigen::Matrix3d Ib = Eigen::Matrix3d::Identity() * 50.0;  // inertia tensor (kg*m^2)

        Eigen::Matrix<double, DOF, 1> Ma;   // added mass coefficients
        Eigen::Matrix<double, DOF, 1> Dl;   // linear damping coefficients
        Eigen::Matrix<double, DOF, 1> Dq;   // quadratic damping coefficients

        double buoyancy = 1400.0;  // N
        double weight = 1403.0;    // N
        Eigen::Vector3d r_b = Eigen::Vector3d(0, 0, -0.02);  // center of buoyancy (m)
        Eigen::Vector3d r_g = Eigen::Vector3d(0, 0, 0.02);   // center of gravity (m)

        Params() {
            Ma << 30, 40, 80, 5, 10, 10;
            Dl << 50, 60, 80, 10, 10, 15;
            Dq << 100, 120, 200, 20, 20, 30;
        }
    };

    AUVDynamics();
    explicit AUVDynamics(const Params& params);
    ~AUVDynamics() = default;

    void setParams(const Params& params);

    /// @brief Get total mass matrix M = M_RB + M_A
    Eigen::Matrix<double, DOF, DOF> getMassMatrix() const;

    /// @brief Get Coriolis-centripetal matrix C(nu)
    Eigen::Matrix<double, DOF, DOF> getCoriolisMatrix(
        const Eigen::Matrix<double, DOF, 1>& nu) const;

    /// @brief Get damping matrix D(nu) = D_l + D_q*|nu|
    Eigen::Matrix<double, DOF, DOF> getDampingMatrix(
        const Eigen::Matrix<double, DOF, 1>& nu) const;

    /// @brief Get restoring forces g(eta) from gravity and buoyancy
    Eigen::Matrix<double, DOF, 1> getRestoringForces(
        const Eigen::Matrix<double, DOF, 1>& eta) const;

    /// @brief Compute nu_dot = M_inv * (tau - C*nu - D*nu - g)
    Eigen::Matrix<double, DOF, 1> computeAcceleration(
        const Eigen::Matrix<double, DOF, 1>& eta,
        const Eigen::Matrix<double, DOF, 1>& nu,
        const Eigen::Matrix<double, DOF, 1>& tau) const;

    /// @brief Get 6x6 kinematic Jacobian J(eta); singular at pitch = +/-90deg
    Eigen::Matrix<double, DOF, DOF> getJacobian(
        const Eigen::Matrix<double, DOF, 1>& eta) const;

private:
    Params params_;
    Eigen::Matrix<double, DOF, DOF> M_;
    Eigen::Matrix<double, DOF, DOF> M_inv_;

    void updateMassMatrix();
};

}  // namespace models
}  // namespace ftc_control

#endif  // FTC_CONTROL_MODELS_AUV_DYNAMICS_HPP_
