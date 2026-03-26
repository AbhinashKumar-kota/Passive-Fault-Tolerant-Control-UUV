#ifndef FTC_CONTROL_CONTROLLERS_I_FTSMC_CONTROLLER_HPP_
#define FTC_CONTROL_CONTROLLERS_I_FTSMC_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace ftc_control {
namespace controllers {

struct ControlLimits {
    double max_force = 120.0;
    double max_torque = 30.0;
};

class IFTSMCController {
public:
    IFTSMCController();
    void reset();

    Eigen::Matrix<double, 6, 1> compute_i_ftsmc(
        const Eigen::Matrix<double, 6, 1>& eta,
        const Eigen::Matrix<double, 6, 1>& eta_d,
        const Eigen::Matrix<double, 6, 1>& dot_eta_d,
        const Eigen::Matrix<double, 6, 1>& nu,
        double dt);

private:
    double sig(double x, double beta) const;
    Eigen::Matrix<double, 6, 6> getRotationMatrix(const Eigen::Matrix<double, 6, 1>& eta) const;
    Eigen::Matrix<double, 6, 1> applyLimits(const Eigen::Matrix<double, 6, 1>& tau) const;

    Eigen::Matrix<double, 6, 1> s_nu_integral_;
    Eigen::Matrix<double, 6, 1> nu_c_old_;

    ControlLimits limits_;
};

} // namespace controllers
} // namespace ftc_control

#endif
