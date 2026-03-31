#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>

namespace interface {

class ROS2CommandInterface : public UserCommandInterface {
public:
    ROS2CommandInterface(RobotName robot_name, rclcpp::Node::SharedPtr node) 
        : UserCommandInterface(robot_name), node_(node) {
        
        // Initialize user command to default safe states
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
        usr_cmd_->target_mode = uint8_t(types::RobotMotionState::RLControlMode); // Default to RL mode for Nav2
        usr_cmd_->safe_control_mode = 0;

        // Create subscriber for Nav2 velocity commands
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ROS2CommandInterface::CmdVelCallback, this, std::placeholders::_1)
        );

        std::cout << "[ROS2CommandInterface] Listening on /cmd_vel for autonomous navigation.\n";
    }

    ~ROS2CommandInterface() override {
        Stop();
    }

    void Start() override {
        running_ = true;
        std::cout << "[ROS2CommandInterface] Started.\n";
    }

    void Stop() override {
        running_ = false;
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
    }

    UserCommand* GetUserCommand() override {
        return usr_cmd_;
    }

private:
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!running_) return;

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        
        // Map Nav2 Twist to UserCommand velocity scales
        usr_cmd_->forward_vel_scale = static_cast<float>(msg->linear.x);
        usr_cmd_->side_vel_scale = static_cast<float>(msg->linear.y);
        usr_cmd_->turnning_vel_scale = static_cast<float>(msg->angular.z);
        
        // Update timestamp
        usr_cmd_->time_stamp = node_->get_clock()->now().seconds();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::atomic<bool> running_{false};
    std::mutex cmd_mutex_;
};

} // namespace interface
