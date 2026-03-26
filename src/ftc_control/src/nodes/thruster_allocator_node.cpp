#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <Eigen/Dense>
#include <algorithm>

#include "ftc_control/allocation/thruster_allocation.hpp"

using namespace ftc_control;

class ThrusterAllocatorNode : public rclcpp::Node {
public:
    ThrusterAllocatorNode() : Node("thruster_allocator_node") {
        this->declare_parameter("robot_name", "BLUEROV2");
        this->declare_parameter("robot_type", "bluerov2");
        this->declare_parameter("force_to_rpm_gain", 50.0);

        robot_name_ = this->get_parameter("robot_name").as_string();
        robot_type_ = this->get_parameter("robot_type").as_string();
        pwm_gain_ = this->get_parameter("force_to_rpm_gain").as_double();

        if (robot_type_ == "bluerov2") {
            num_thrusters_ = 8;
            allocator_ = std::make_unique<allocation::ThrusterAllocator>(8);
            allocator_->initBlueROV2();
            thruster_topic_ = "/" + robot_name_ + "/setpoint/pwm";
        } else {
            num_thrusters_ = 5;
            allocator_ = std::make_unique<allocation::ThrusterAllocator>(5);
            allocator_->initGirona500();
            thruster_topic_ = "/" + robot_name_ + "/thrusters/setpoint";
        }

        fault_vector_ = Eigen::VectorXd::Ones(num_thrusters_);

        tau_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/FTC/control_forces", 10,
            std::bind(&ThrusterAllocatorNode::tauCallback, this, std::placeholders::_1));

        fault_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/FTC/thruster_faults", 10,
            std::bind(&ThrusterAllocatorNode::faultCallback, this, std::placeholders::_1));

        thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(thruster_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Allocator Node started. Passive Fault Mode active.");
    }

private:
    void faultCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != (size_t)num_thrusters_) {
            RCLCPP_ERROR(this->get_logger(), "Fault vector size mismatch! Expected %d", num_thrusters_);
            return;
        }

        for (int i = 0; i < num_thrusters_; ++i) {
            fault_vector_(i) = std::clamp(msg->data[i], 0.0, 1.0);
        }
        RCLCPP_WARN(this->get_logger(), "Multiplicative fault vector updated.");
    }

    void tauCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 6) return;

        Eigen::Vector<double, 6> tau;
        for (int i = 0; i < 6; ++i) tau(i) = msg->data[i];

        // Nominal allocation: u = B+ * tau
        Eigen::VectorXd u = allocator_->allocate(tau);

        // Inject multiplicative fault (passive FTC — controller is unaware)
        for (int i = 0; i < num_thrusters_; ++i) {
            u(i) *= fault_vector_(i);
        }

        auto thrust_msg = std_msgs::msg::Float64MultiArray();
        thrust_msg.data.resize(num_thrusters_);

        for (int i = 0; i < num_thrusters_; ++i) {
            double rpm = u(i) * pwm_gain_;

            // BlueROV2 Stonefish thruster direction inversions
            if (robot_type_ == "bluerov2") {
                if (i == 0 || i == 1 || i == 5 || i == 6) rpm = rpm;
            }

            thrust_msg.data[i] = std::clamp(rpm, -2000.0, 2000.0);
        }

        thruster_pub_->publish(thrust_msg);
    }

    std::unique_ptr<allocation::ThrusterAllocator> allocator_;
    Eigen::VectorXd fault_vector_;
    int num_thrusters_;
    std::string robot_name_, robot_type_, thruster_topic_;
    double pwm_gain_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_sub_, fault_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterAllocatorNode>());
    rclcpp::shutdown();
    return 0;
}
