// State estimation node — ground truth passthrough or sensor fusion for AUV control

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ftc_control/utils/frames.hpp"

#include <Eigen/Dense>

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode() : Node("state_estimator_node") {
        this->declare_parameter("robot_name", "BLUEROV2");
        this->declare_parameter("robot_type", "bluerov2");
        this->declare_parameter("rate", 50.0);
        this->declare_parameter("use_ground_truth", true);

        robot_name_ = this->get_parameter("robot_name").as_string();
        robot_type_ = this->get_parameter("robot_type").as_string();
        rate_ = this->get_parameter("rate").as_double();
        use_ground_truth_ = this->get_parameter("use_ground_truth").as_bool();

        std::string odom_topic, imu_topic, pressure_topic;
        if (robot_type_ == "bluerov2") {
            odom_topic = "/" + robot_name_ + "/odometry";
            imu_topic = "/" + robot_name_ + "/imu_filter";
            pressure_topic = "/" + robot_name_ + "/pressure";
        } else {
            odom_topic = "/" + robot_name_ + "/dynamics";
            imu_topic = "/" + robot_name_ + "/imu";
            pressure_topic = "/" + robot_name_ + "/pressure";
        }

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&StateEstimatorNode::odomCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&StateEstimatorNode::imuCallback, this, std::placeholders::_1));

        pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            pressure_topic, 10,
            std::bind(&StateEstimatorNode::pressureCallback, this, std::placeholders::_1));

        state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/FTC/estimated_state", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        eta_.setZero();
        nu_.setZero();

        RCLCPP_INFO(this->get_logger(), "State estimator initialized for %s (%s)",
            robot_name_.c_str(), robot_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to odometry: %s", odom_topic.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (use_ground_truth_) {
            eta_(0) = msg->pose.pose.position.x;
            eta_(1) = msg->pose.pose.position.y;
            eta_(2) = msg->pose.pose.position.z;

            auto euler = ftc_control::utils::quaternionToEuler(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            eta_(3) = euler(0);
            eta_(4) = euler(1);
            eta_(5) = euler(2);

            nu_(0) = msg->twist.twist.linear.x;
            nu_(1) = msg->twist.twist.linear.y;
            nu_(2) = msg->twist.twist.linear.z;
            nu_(3) = msg->twist.twist.angular.x;
            nu_(4) = msg->twist.twist.angular.y;
            nu_(5) = msg->twist.twist.angular.z;
        }

        publishState();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!use_ground_truth_) {
            auto euler = ftc_control::utils::quaternionToEuler(
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                msg->orientation.w
            );
            eta_(3) = euler(0);
            eta_(4) = euler(1);
            eta_(5) = euler(2);

            nu_(3) = msg->angular_velocity.x;
            nu_(4) = msg->angular_velocity.y;
            nu_(5) = msg->angular_velocity.z;
        }
    }

    // depth = (P - P_atm) / (rho * g)
    void pressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
        if (!use_ground_truth_) {
            double P_atm = 101325.0;
            double rho = 1025.0;
            double g = 9.81;

            eta_(2) = (msg->fluid_pressure - P_atm) / (rho * g);
        }
    }

    void publishState() {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world_ned";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = eta_(0);
        msg.pose.pose.position.y = eta_(1);
        msg.pose.pose.position.z = eta_(2);

        auto q = ftc_control::utils::eulerToQuaternion(eta_(3), eta_(4), eta_(5));
        msg.pose.pose.orientation.x = q(0);
        msg.pose.pose.orientation.y = q(1);
        msg.pose.pose.orientation.z = q(2);
        msg.pose.pose.orientation.w = q(3);

        msg.twist.twist.linear.x = nu_(0);
        msg.twist.twist.linear.y = nu_(1);
        msg.twist.twist.linear.z = nu_(2);
        msg.twist.twist.angular.x = nu_(3);
        msg.twist.twist.angular.y = nu_(4);
        msg.twist.twist.angular.z = nu_(5);

        state_pub_->publish(msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string robot_name_;
    std::string robot_type_;
    double rate_;
    bool use_ground_truth_;

    Eigen::Matrix<double, 6, 1> eta_;
    Eigen::Matrix<double, 6, 1> nu_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
