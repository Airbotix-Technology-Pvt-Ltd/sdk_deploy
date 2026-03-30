/**
 * @file state_bridge_node.cpp
 * @brief Zero-Latency Sim-Time Synced Sensor Forwarder.
 * Translates Isaac Sim's sensor_msgs (Simulation Time) → drdds hardware messages.
 **/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/imu_data.hpp"
#include <cmath>
#include <mutex>
#include <vector>
#include <string>

namespace isaac_bridge {

class StateBridgeNode : public rclcpp::Node {
public:
    StateBridgeNode() : Node("state_bridge_node") {
        this->declare_parameter("joint_names", std::vector<std::string>{
            "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
            "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
            "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
            "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
        });

        joint_names_ = this->get_parameter("joint_names").as_string_array();

        joints_pub_ = create_publisher<drdds::msg::JointsData>("/JOINTS_DATA", 10);
        imu_pub_    = create_publisher<drdds::msg::ImuData>("/IMU_DATA", 10);

        // ── Event-Driven Subscribers (Zero Latency) ─────────────────────────
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { OnJointState(msg); });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(msg); });
    }

private:
    void OnJointState(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        drdds::msg::JointsData jd;
        jd.header.stamp = msg->header.stamp; // SYNC: Use Isaac Sim's timestamp

        for (int j = 0; j < 12; ++j) {
            bool found = false;
            for (size_t idx = 0; idx < msg->name.size(); ++idx) {
                if (msg->name[idx] == joint_names_[j]) {
                    jd.data.joints_data[j].position = static_cast<float>(msg->position[idx]);
                    jd.data.joints_data[j].velocity = static_cast<float>(msg->velocity[idx]);
                    jd.data.joints_data[j].torque   = static_cast<float>(msg->effort[idx]);
                    found = true;
                    break;
                }
            }
            jd.data.joints_data[j].status_word = found ? 1 : 0;
        }
        for (int i = 12; i < 16; ++i) jd.data.joints_data[i].status_word = 1;
        joints_pub_->publish(jd);
    }

    void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        const auto& q = msg->orientation;
        // Check for Zero Quat (Identity check if invalid)
        if (q.w == 0 && q.x == 0 && q.y == 0 && q.z == 0) return;

        // Quat → RPY
        float r = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        float p = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
        float y = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        drdds::msg::ImuData imu;
        imu.header.stamp = msg->header.stamp; // SYNC: Use Isaac Sim's timestamp
        imu.data.roll    = r;
        imu.data.pitch   = p;
        imu.data.yaw     = y;
        imu.data.omega_x = msg->angular_velocity.x;
        imu.data.omega_y = msg->angular_velocity.y;
        imu.data.omega_z = msg->angular_velocity.z;
        imu.data.acc_x   = msg->linear_acceleration.x;
        imu.data.acc_y   = msg->linear_acceleration.y;
        imu.data.acc_z   = msg->linear_acceleration.z;
        imu_pub_->publish(imu);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Publisher<drdds::msg::JointsData>::SharedPtr            joints_pub_;
    rclcpp::Publisher<drdds::msg::ImuData>::SharedPtr               imu_pub_;
    std::vector<std::string> joint_names_;
};

} // namespace isaac_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<isaac_bridge::StateBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
