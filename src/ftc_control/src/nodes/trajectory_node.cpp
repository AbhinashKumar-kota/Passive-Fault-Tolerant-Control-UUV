// Trajectory generation node for AUV controller reference tracking

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <cmath>

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode() : Node("trajectory_node") {
        this->declare_parameter("rate", 20.0);
        this->declare_parameter("trajectory_type", "hover");

        rate_ = this->get_parameter("rate").as_double();
        trajectory_type_ = this->get_parameter("trajectory_type").as_string();

        desired_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/FTC/desired_state", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/FTC/trajectory_path", 10);

        path_msg_.header.frame_id = "world_ned";

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_),
            std::bind(&TrajectoryNode::timerCallback, this));

        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Trajectory node initialized");
        RCLCPP_INFO(this->get_logger(), "Trajectory type: %s", trajectory_type_.c_str());
    }

private:
    void timerCallback() {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world_ned";
        msg.child_frame_id = "base_link";

        double t = (this->now() - start_time_).seconds();

        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        double yaw = 0.0;
        double yaw_rate = 0.0;

        // hover
        if (trajectory_type_ == "hover") {
            pos << 0.0, 0.0, 2.0;
            vel << 0.0, 0.0, 0.0;
            yaw = 0.0;
            yaw_rate = 0.0;
        }
        // eight
        // x(t) = r*cos(wt), y(t) = 2r*sin(wt)*cos(wt), z(t) = 1.5 + r*cos(wt)
        else if (trajectory_type_ == "eight") {
            double radius = 1.0;
            double omega = 0.02;

            pos << radius * std::cos(omega * t),
                   2 * radius * std::sin(omega * t) * std::cos(omega * t),
                   1.5 + radius * std::cos(omega * t);

            // dx/dt = -r*w*sin(wt), dy/dt = 2r*w*(cos^2(wt) - sin^2(wt)), dz/dt = -r*w*sin(wt)
            vel << -radius * omega * std::sin(omega * t),
                    2*radius * omega * std::cos(omega * t) * std::cos(omega * t) - 2*radius * omega * std::sin(omega * t) * std::sin(omega * t),
                    -radius * omega * std::sin(omega * t);

            // yaw = 0*(wt + pi/2) — effectively disabled
            yaw = 0.0*(omega * t + M_PI / 2.0);

            while (yaw > M_PI) yaw -= 2.0 * M_PI;
            while (yaw < -M_PI) yaw += 2.0 * M_PI;

            yaw_rate = omega;
        }
        // lawnmower
        else if (trajectory_type_ == "lawnmower") {
            double speed = 0.3;
            double width = 6.0;
            double leg_length = 4.0;
            double depth = 2.0;

            double dist = speed * t;

            int leg = static_cast<int>(dist / leg_length);
            double leg_progress = std::fmod(dist, leg_length);

            if (leg % 2 == 0) {
                pos << leg_progress, (leg / 2) * width, depth;
                vel << speed, 0.0, 0.0;
                yaw = 0.0;
            } else {
                pos << leg_length - leg_progress, (leg / 2) * width + width/2, depth;
                vel << -speed, 0.0, 0.0;
                yaw = M_PI;
            }
            yaw_rate = 0.0;
        }
        // waypoint
        else if (trajectory_type_ == "waypoint") {
            pos << 10.0, 10.0, 2.0;
            vel << 0.0, 0.0, 0.0;
            yaw = 0.0;
            yaw_rate = 0.0;
        }

        msg.pose.pose.position.x = pos(0);
        msg.pose.pose.position.y = pos(1);
        msg.pose.pose.position.z = pos(2);

        // Pure yaw quaternion: q = [cos(psi/2), 0, 0, sin(psi/2)]
        msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = std::sin(yaw / 2.0);

        msg.twist.twist.linear.x = vel(0);
        msg.twist.twist.linear.y = vel(1);
        msg.twist.twist.linear.z = vel(2);
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = yaw_rate;

        desired_pose_pub_->publish(msg);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg.header;
        pose_stamped.pose = msg.pose.pose;

        path_msg_.header.stamp = this->now();
        path_msg_.poses.push_back(pose_stamped);

        if (path_msg_.poses.size() > 500) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }

        path_pub_->publish(path_msg_);

        static int count = 0;
        if (++count % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] Desired: x=%.2f y=%.2f z=%.2f yaw=%.2f",
                trajectory_type_.c_str(), pos(0), pos(1), pos(2), yaw);
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr desired_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    double rate_;
    std::string trajectory_type_;
    rclcpp::Time start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
