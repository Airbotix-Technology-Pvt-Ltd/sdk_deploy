/**
 * @file cmd_bridge_node.cpp
 * @brief Translates drdds/JOINTS_CMD → Isaac Sim topics.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "drdds/msg/joints_data_cmd.hpp"

namespace isaac_bridge {

class CmdBridgeNode : public rclcpp::Node {
public:
    CmdBridgeNode() : Node("cmd_bridge_node") {
        this->declare_parameter("joint_names", std::vector<std::string>{
            "FL_HipX", "FL_HipY", "FL_Knee",
            "FR_HipX", "FR_HipY", "FR_Knee",
            "HL_HipX", "HL_HipY", "HL_Knee",
            "HR_HipX", "HR_HipY", "HR_Knee"
        });

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

        // ── Default Sitting Timer ─────────────────────────────────────────
        // We publish a sitting pose until the SDK starts sending its own commands.
        // This prevents the robot from falling over in simulation on startup.
        sitting_timer_ = create_wall_timer(
            std::chrono::milliseconds(10), [this]() {
                if (!has_received_cmd_) {
                    PublishDefaultSitting();
                }
            });

        RCLCPP_INFO(get_logger(), "Cmd bridge started: /JOINTS_CMD → /joint_commands");
    }

private:
    rclcpp::Subscription<drdds::msg::JointsDataCmd>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::TimerBase::SharedPtr sitting_timer_;
    
    std::vector<std::string> joint_names_;
    bool has_received_cmd_ = false;

    // Standard Lite3 Sitting Pose (SDK Units)
    // HipY: URDF -1.355 -> SDK (-1.355 - 0.8) = -2.155
    // Knee: URDF  2.550 -> SDK ( 2.550 + 1.076) = 3.626
    const std::vector<double> sitting_q_ = {
        0.0, -2.155, 3.626, // FL
        0.0, -2.155, 3.626, // FR
        0.0, -2.155, 3.626, // HL
        0.0, -2.155, 3.626  // HR
    };

    void PublishDefaultSitting() {
        sensor_msgs::msg::JointState js;
        js.header.stamp = get_clock()->now();
        js.name         = joint_names_;
        js.position.resize(12);
        for (int i = 0; i < 12; ++i) {
            double pos = sitting_q_[i];
            // Inverse mapping to Sim: HipY +0.8, Knee -1.076
            if (i % 3 == 1) pos += 0.8;
            if (i % 3 == 2) pos -= 1.076;
            js.position[i] = pos;
        }
        js.velocity     = std::vector<double>(12, 0.0);
        js.effort       = std::vector<double>(12, 0.0);
        js_pub_->publish(js);
    }

    void OnJointCmd(const drdds::msg::JointsDataCmd::ConstSharedPtr& msg) {
        // Simple heuristic: if the SDK is sending 0 for Kp on all joints, it is in 'Idle' or 'Damping'
        // and we should NOT let it override our stable sitting pose in simulation.
        bool has_gains = false;
        for (int i = 0; i < 12; ++i) {
            if (msg->data.joints_data[i].kp > 0.1 || msg->data.joints_data[i].kd > 0.1) {
                has_gains = true;
                break;
            }
        }

        if (!has_gains) {
            // SDK is sending zeros (Idle). Do not update has_received_cmd.
            // This allows sitting_timer_ to keep the robot seated.
            return;
        }

        has_received_cmd_ = true;
        
        sensor_msgs::msg::JointState js;
        js.header.stamp = get_clock()->now();
        js.name         = joint_names_;
        js.position.resize(12);
        js.velocity.resize(12);
        js.effort.resize(12);

        for (int i = 0; i < 12; ++i) {
            double pos = msg->data.joints_data[i].position;
            // Inverse mapping to Sim: HipY +0.8, Knee -1.076
            if (i % 3 == 1) pos += 0.8;
            if (i % 3 == 2) pos -= 1.076;
            js.position[i] = pos;
            
            js.velocity[i] = msg->data.joints_data[i].velocity;
            js.effort[i]   = msg->data.joints_data[i].torque;
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
