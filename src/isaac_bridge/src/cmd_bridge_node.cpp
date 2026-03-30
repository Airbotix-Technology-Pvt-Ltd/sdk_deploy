/**
 * @file cmd_bridge_node.cpp
 * @brief Zero-Latency Sim-Time Synced Position/Velocity Bridge.
 * Translates drdds/JOINTS_CMD → Isaac Sim sensor_msgs/JointState (Pos/Vel Targets).
 **/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "drdds/msg/joints_data_cmd.hpp"
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

namespace isaac_bridge {

class CmdBridgeNode : public rclcpp::Node {
public:
    CmdBridgeNode() : Node("cmd_bridge_node") {
        this->declare_parameter("joint_names", std::vector<std::string>{
            "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
            "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
            "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
            "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
        });

        joint_names_ = this->get_parameter("joint_names").as_string_array();
        q_.assign(12, 0.0f);
        q_init_.assign(12, 0.0f);

        // ── Subscribers ───────────────────────────────────────────────────
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { OnJointState(msg); });

        cmd_sub_ = create_subscription<drdds::msg::JointsDataCmd>(
            "/JOINTS_CMD", 10,
            [this](drdds::msg::JointsDataCmd::ConstSharedPtr msg) { OnJointCmd(msg); });

        // ── Publisher ─────────────────────────────────────────────────────
        js_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_commands", rclcpp::SensorDataQoS());

        // ── Startup Heartbeat (Freeze until RL starts) ────────────────────
        startup_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
            if (!rl_active_ && q_captured_) PublishFreezePose();
        });
    }

private:
    void OnJointState(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t idx = 0; idx < msg->name.size(); ++idx) {
            for (int j = 0; j < 12; ++j) {
                if (msg->name[idx] == joint_names_[j]) {
                    q_[j]  = static_cast<float>(msg->position[idx]);
                    break;
                }
            }
        }

        if (!q_captured_) {
            if (settlement_frames_ < 100) {
                settlement_frames_++;
            } else {
                q_init_ = q_;
                q_captured_ = true;
                RCLCPP_INFO(get_logger(), "Settlement done. Sim-Time synced freeze active.");
            }
        }
    }

    void PublishFreezePose() {
        sensor_msgs::msg::JointState js;
        js.header.stamp = get_clock()->now(); 
        js.name         = joint_names_;
        js.position.assign(12, 0.0);

        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < 12; ++i) {
            js.position[i] = q_init_[i];
        }
        js_pub_->publish(js);
    }

    void OnJointCmd(const drdds::msg::JointsDataCmd::ConstSharedPtr& msg) {
        if (!rl_active_) {
            rl_active_ = true;
            startup_timer_->cancel();
            RCLCPP_INFO(get_logger(), "RL Sync Master Active. Forwarding Pos/Vel targets.");
        }

        sensor_msgs::msg::JointState js;
        js.header.stamp = get_clock()->now(); 
        js.name         = joint_names_;
        js.position.resize(12);
        js.velocity.resize(12);

        for (int i = 0; i < 12; ++i) {
            const auto& c = msg->data.joints_data[i];
            js.position[i] = c.position;
            js.velocity[i] = c.velocity;
            // NOTE: js.effort is purposefully NOT published to avoid fighting simulator PD.
        }
        js_pub_->publish(js);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<drdds::msg::JointsDataCmd>::SharedPtr    cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       js_pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    
    std::vector<std::string> joint_names_;
    std::vector<float> q_;
    std::vector<float> q_init_;
    bool q_captured_ = false;
    bool rl_active_ = false;
    int settlement_frames_ = 0;
    std::mutex state_mutex_;
};

} // namespace isaac_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<isaac_bridge::CmdBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
