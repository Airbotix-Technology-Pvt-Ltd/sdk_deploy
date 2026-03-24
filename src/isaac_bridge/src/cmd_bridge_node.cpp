/**
 * @file cmd_bridge_node.cpp
 * @brief Translates drdds/JOINTS_CMD → Isaac Sim topics (Pure Passthrough).
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "drdds/msg/joints_data_cmd.hpp"
#include <mutex>
#include <chrono>
#include <vector>

namespace isaac_bridge {

class CmdBridgeNode : public rclcpp::Node {
public:
    CmdBridgeNode() : Node("cmd_bridge_node") {
        this->declare_parameter("publish_rate_hz", 200);
        this->declare_parameter("joint_names", std::vector<std::string>{
            "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
            "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
            "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
            "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
        });

        int rate_hz = this->get_parameter("publish_rate_hz").as_int();
        joint_names_ = this->get_parameter("joint_names").as_string_array();

        // ── Subscriber (Lite3_sdk_deploy) ──────────────────────────────────
        cmd_sub_ = create_subscription<drdds::msg::JointsDataCmd>(
            "/JOINTS_CMD", 10,
            [this](drdds::msg::JointsDataCmd::ConstSharedPtr msg) { 
                OnJointCmd(msg); 
            });

        // ── Publisher (Isaac Sim) ──────────────────────────────────────────
        js_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_commands", rclcpp::SystemDefaultsQoS());

        // ── Timer (Steady 200 Hz Passthrough) ────────────────────────────────
        auto period = std::chrono::milliseconds(1000 / rate_hz);
        timer_ = create_wall_timer(period, [this]() { PublishCmd(); });
    }

private:
    rclcpp::Subscription<drdds::msg::JointsDataCmd>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::mutex cmd_mutex_;
    std::vector<std::string> joint_names_;
    drdds::msg::JointsDataCmd last_cmd_;
    bool has_cmd_ = false;

    void OnJointCmd(const drdds::msg::JointsDataCmd::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cmd_ = *msg;
        has_cmd_ = true;
    }

    void PublishCmd() {
        if (!has_cmd_) return;

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        sensor_msgs::msg::JointState js;
        js.header.stamp = get_clock()->now();
        js.name         = joint_names_;
        js.position.resize(12);
        js.velocity.resize(12);
        js.effort.resize(12);

        for (int i = 0; i < 12; ++i) {
            js.position[i] = last_cmd_.data.joints_data[i].position;
            js.velocity[i] = last_cmd_.data.joints_data[i].velocity;
            js.effort[i]   = last_cmd_.data.joints_data[i].torque;
        }
        js_pub_->publish(js);
    }
};

} // namespace isaac_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<isaac_bridge::CmdBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
