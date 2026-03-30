#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

#include "ftc_control/controllers/ftsmc_controller.hpp"
#include "ftc_control/controllers/i_ftsmc_controller.hpp"
#include "ftc_control/allocation/thruster_allocation.hpp"
#include "ftc_control/utils/frames.hpp"

using namespace ftc_control;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        this->declare_parameter("vehicle_name", "BLUEROV2");
        this->declare_parameter("controller_type", "ftsmc");

        vehicle_name_ = this->get_parameter("vehicle_name").as_string();
        controller_type_ = this->get_parameter("controller_type").as_string();

        ftsmc_controller_ = std::make_unique<controllers::FTSMCController>();
        i_ftsmc_controller_ = std::make_unique<controllers::IFTSMCController>();

        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/FTC/estimated_state", 10,
            std::bind(&ControllerNode::stateCallback, this, std::placeholders::_1));

        target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/FTC/desired_state", 10,
            std::bind(&ControllerNode::targetCallback, this, std::placeholders::_1));

        tau_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/FTC/control_forces", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Controller node initialized with controller: %s", controller_type_.c_str());
    }

private:
    void controlLoop() {
        Eigen::Matrix<double, 6, 1> tau = Eigen::Matrix<double, 6, 1>::Zero();

        static int debug_count = 0;
        if (++debug_count % 50 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "Desired: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                eta_d_(0), eta_d_(1), eta_d_(2), eta_d_(5));
        }

        if (controller_type_ == "ftsmc") {
            tau = ftsmc_controller_->compute_ftsmc(eta_, eta_d_, eta_d_dot_, nu_, 0.02);
        } else if (controller_type_ == "i_ftsmc") {
            tau = i_ftsmc_controller_->compute_i_ftsmc(eta_, eta_d_, eta_d_dot_, nu_, 0.02);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "Unknown controller type: %s. Valid options: ftsmc, i_ftsmc",
                controller_type_.c_str());
        }

        auto tau_msg = std_msgs::msg::Float64MultiArray();
        tau_msg.data.resize(6);
        for (int i = 0; i < 6; ++i) tau_msg.data[i] = tau(i);
        tau_pub_->publish(tau_msg);
    }

    void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        eta_(0) = msg->pose.pose.position.x;
        eta_(1) = msg->pose.pose.position.y;
        eta_(2) = msg->pose.pose.position.z;

        auto euler = utils::quaternionToEuler(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        eta_.segment<3>(3) = euler;

        nu_(0) = msg->twist.twist.linear.x;
        nu_(1) = msg->twist.twist.linear.y;
        nu_(2) = msg->twist.twist.linear.z;
        nu_(3) = msg->twist.twist.angular.x;
        nu_(4) = msg->twist.twist.angular.y;
        nu_(5) = msg->twist.twist.angular.z;
    }

    void targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        eta_d_(0) = msg->pose.pose.position.x;
        eta_d_(1) = msg->pose.pose.position.y;
        eta_d_(2) = msg->pose.pose.position.z;

        auto euler = utils::quaternionToEuler(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        eta_d_.segment<3>(3) = euler;

        eta_d_dot_(0) = msg->twist.twist.linear.x;
        eta_d_dot_(1) = msg->twist.twist.linear.y;
        eta_d_dot_(2) = msg->twist.twist.linear.z;
        eta_d_dot_(3) = msg->twist.twist.angular.x;
        eta_d_dot_(4) = msg->twist.twist.angular.y;
        eta_d_dot_(5) = msg->twist.twist.angular.z;
    }

    std::string vehicle_name_;
    std::string controller_type_;

    std::unique_ptr<controllers::FTSMCController> ftsmc_controller_;
    std::unique_ptr<controllers::IFTSMCController> i_ftsmc_controller_;

    Eigen::Matrix<double, 6, 1> eta_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> nu_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> eta_d_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> eta_d_dot_ = Eigen::Matrix<double, 6, 1>::Zero();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_, target_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
