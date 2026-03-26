// Thruster allocation: maps desired body forces/torques to individual thruster commands.

#ifndef FTC_CONTROL_ALLOCATION_THRUSTER_ALLOCATION_HPP_
#define FTC_CONTROL_ALLOCATION_THRUSTER_ALLOCATION_HPP_

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace ftc_control {
namespace allocation {

/// @brief Pseudo-inverse thruster allocation with fault tolerance
class ThrusterAllocator {
public:
    static constexpr int DOF = 6;

    struct ThrusterConfig {
        std::string name;
        Eigen::Vector3d position;
        Eigen::Vector3d direction;
        double max_thrust = 100.0;
        double min_thrust = -100.0;
    };

    struct FaultStatus {
        std::vector<bool> healthy;
        std::vector<double> effectiveness;  // 1.0 = full, 0.0 = failed
        FaultStatus(int n = 8) : healthy(n, true), effectiveness(n, 1.0) {}
    };

    ThrusterAllocator(int num_thrusters = 8);
    ~ThrusterAllocator() = default;

    /// @brief Initialize with Girona500 thruster layout (5 thrusters)
    void initGirona500();

    /// @brief Initialize with BlueROV2 Heavy thruster layout (8 thrusters)
    void initBlueROV2();

    /// @brief Set custom thruster configuration
    void setConfiguration(const std::vector<ThrusterConfig>& config);

    /// @brief Update fault status and recompute allocation matrix
    void setFaultStatus(const FaultStatus& status);

    /// @brief Allocate using pseudo-inverse: u = B_pinv * tau
    Eigen::VectorXd allocate(const Eigen::Matrix<double, DOF, 1>& tau);

    /// @brief Allocate using weighted pseudo-inverse
    Eigen::VectorXd allocateWeighted(
        const Eigen::Matrix<double, DOF, 1>& tau,
        const Eigen::MatrixXd& W);

    Eigen::MatrixXd getB() const { return B_; }
    Eigen::MatrixXd getEffectiveB() const;
    bool isAchievable(const Eigen::Matrix<double, DOF, 1>& tau) const;
    std::vector<std::string> getThrusterNames() const;
    int getNumThrusters() const { return num_thrusters_; }

private:
    int num_thrusters_;
    std::vector<ThrusterConfig> config_;
    FaultStatus fault_status_;

    Eigen::MatrixXd B_;
    Eigen::MatrixXd B_pinv_;

    void updateAllocationMatrix();
};

}  // namespace allocation
}  // namespace ftc_control

#endif  // FTC_CONTROL_ALLOCATION_THRUSTER_ALLOCATION_HPP_
