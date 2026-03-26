#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <iostream>
#include <iomanip>

class DataLoggerNode : public rclcpp::Node {
public:
    DataLoggerNode() : Node("data_logger_node") {
        this->declare_parameter("log_file_name", "auv_mission_log.csv");
        std::string filename = this->get_parameter("log_file_name").as_string();

        csv_file_.open(filename);
        start_time_ = this->now();
        writeHeader();

        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/FTC/estimated_state", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_eta_[0] = msg->pose.pose.position.x;
                current_eta_[1] = msg->pose.pose.position.y;
                current_eta_[2] = msg->pose.pose.position.z;

                tf2::Quaternion q(
                    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
                tf2::Matrix3x3 m(q);
                m.getRPY(current_eta_[3], current_eta_[4], current_eta_[5]);

                current_nu_[0] = msg->twist.twist.linear.x;
                current_nu_[1] = msg->twist.twist.linear.y;
                current_nu_[2] = msg->twist.twist.linear.z;
                current_nu_[3] = msg->twist.twist.angular.x;
                current_nu_[4] = msg->twist.twist.angular.y;
                current_nu_[5] = msg->twist.twist.angular.z;
            });

        target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/FTC/desired_state", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                desired_eta_[0] = msg->pose.pose.position.x;
                desired_eta_[1] = msg->pose.pose.position.y;
                desired_eta_[2] = msg->pose.pose.position.z;

                tf2::Quaternion q(
                    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
                tf2::Matrix3x3 m(q);
                m.getRPY(desired_eta_[3], desired_eta_[4], desired_eta_[5]);
            });

        tau_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/FTC/control_forces", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                tau_ = msg->data;
            });

        pwm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/BLUEROV2/setpoint/pwm", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                pwm_ = msg->data;
            });

        fault_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/FTC/thruster_faults", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                faults_ = msg->data;
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&DataLoggerNode::logData, this));

        RCLCPP_INFO(this->get_logger(), "Logging mission data to: %s", filename.c_str());
    }

    ~DataLoggerNode() { if (csv_file_.is_open()) csv_file_.close(); }

private:
    void writeHeader() {
        csv_file_ << "elapsed_time,x,y,z,roll,pitch,yaw,u,v,w,p,q,r,x_d,y_d,z_d,roll_d,pitch_d,yaw_d,";
        csv_file_ << "err_x,err_y,err_z,err_roll,err_pitch,err_yaw,";
        for(int i=0; i<6; ++i) csv_file_ << "tau_" << i << ",";
        for(int i=0; i<8; ++i) csv_file_ << "pwm_" << i << ",";
        for(int i=0; i<8; ++i) csv_file_ << "fault_" << i << (i==7 ? "" : ",");
        csv_file_ << "\n";
    }

    void logData() {
        rclcpp::Time now = this->now();
        double elapsed = (now - start_time_).seconds();

        csv_file_ << std::fixed << std::setprecision(4) << elapsed << ",";

        for(int i=0; i<6; ++i) csv_file_ << current_eta_[i] << ",";
        for(int i=0; i<6; ++i) csv_file_ << current_nu_[i] << ",";
        for(int i=0; i<6; ++i) csv_file_ << desired_eta_[i] << ",";

        // Normalize RPY errors to [-pi, pi]
        for(int i=0; i<6; ++i) {
            double err = current_eta_[i] - desired_eta_[i];
            if (i >= 3) {
                while (err > M_PI) err -= 2.0 * M_PI;
                while (err < -M_PI) err += 2.0 * M_PI;
            }
            csv_file_ << err << ",";
        }

        for(int i=0; i<6; ++i) csv_file_ << (tau_.size() > (size_t)i ? tau_[i] : 0.0) << ",";
        for(int i=0; i<8; ++i) csv_file_ << (pwm_.size() > (size_t)i ? pwm_[i] : 0.0) << ",";

        // Default to 1.0 (healthy) if no fault message received yet
        for(int i=0; i<8; ++i) {
            double f = (faults_.size() > (size_t)i ? faults_[i] : 1.0);
            csv_file_ << f << (i==7 ? "" : ",");
        }

        csv_file_ << "\n";
    }

    std::ofstream csv_file_;
    rclcpp::Time start_time_;

    std::vector<double> current_eta_{0,0,0,0,0,0};
    std::vector<double> current_nu_{0,0,0,0,0,0};
    std::vector<double> desired_eta_{0,0,0,0,0,0};
    std::vector<double> tau_, pwm_, faults_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_, target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_sub_, pwm_sub_, fault_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
