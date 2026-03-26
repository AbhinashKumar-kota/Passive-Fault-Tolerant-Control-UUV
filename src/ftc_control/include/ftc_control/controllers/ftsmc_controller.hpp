#ifndef FTC_CONTROL_CONTROLLERS_FTSMC_CONTROLLER_HPP_
#define FTC_CONTROL_CONTROLLERS_FTSMC_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace ftc_control {
namespace controllers {

class FTSMCController {
public:
    struct Limits {
        double max_force = 120.0;
        double max_torque = 30.0;
    };

    FTSMCController();

    /// @brief Compute finite-time sliding mode control law
    Eigen::Matrix<double, 6, 1> compute_ftsmc(
        const Eigen::Matrix<double, 6, 1>& eta,
        const Eigen::Matrix<double, 6, 1>& eta_d,
        const Eigen::Matrix<double, 6, 1>& dot_eta_d,
        const Eigen::Matrix<double, 6, 1>& nu,
        double dt);

    void reset();

private:
    double sig(double x, double beta) const;

    Eigen::Matrix<double, 6, 1> e_nu_old_;
    Eigen::Matrix<double, 6, 1> nu_c_old_;

    Limits limits_;

    Eigen::Matrix<double, 6, 6> getRotationMatrix(const Eigen::Matrix<double, 6, 1>& eta) const;
    Eigen::Matrix<double, 6, 1> applyLimits(const Eigen::Matrix<double, 6, 1>& tau) const;
};

} // namespace controllers
} // namespace ftc_control

#endif
