#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "kinematics.hpp"
#include "pid_controller.hpp"

class AutoController : public rclcpp::Node {
public:
    AutoController();

private:
    void image_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void control_loop();

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> target_;           // x,y,z cm
    sensor_msgs::msg::JointState joints_;  // 当前关节
    bool have_target_{false}, have_joints_{false};

    PidController pid_x_, pid_y_, pid_z_;
};

#endif  // CONTROLLER_HPP_
