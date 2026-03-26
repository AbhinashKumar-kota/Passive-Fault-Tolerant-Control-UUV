#ifndef FTC_CONTROL_CONTROLLERS_BACKSTEPPING_HPP_
#define FTC_CONTROL_CONTROLLERS_BACKSTEPPING_HPP_

#include <Eigen/Dense>

namespace ftc_control {
namespace controllers {

class backstepping {
public:
    struct Limits {
        double max_force;
        double max_torque;
    };

    backstepping();
    void setLimits(const Limits& limits);
    void reset();

    /// @brief Full 6-DOF backstepping control law
    Eigen::Matrix<double, 6, 1> compute_backstepping_full(
        const Eigen::Matrix<double, 6, 1>& eta,
        const Eigen::Matrix<double, 6, 1>& eta_d,
        const Eigen::Matrix<double, 6, 1>& eta_d_dot,
        const Eigen::Matrix<double, 6, 1>& nu,
        const Eigen::DiagonalMatrix<double, 6>& K1,
        const Eigen::DiagonalMatrix<double, 6>& K2);

    Eigen::Matrix<double, 6, 1> applyLimits(const Eigen::Matrix<double, 6, 1>& tau) const;

private:
    Eigen::Matrix<double, 6, 1> tau_;
    Eigen::Matrix<double, 6, 1> nu_c_old_;
    Limits limits_;

    Eigen::Matrix<double, 6, 6> getRotationMatrix(double phi, double theta, double psi);
};

} // namespace controllers
} // namespace ftc_control

#endif
