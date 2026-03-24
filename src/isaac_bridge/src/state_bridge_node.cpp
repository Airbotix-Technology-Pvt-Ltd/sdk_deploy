/**
 * @file state_bridge_node.cpp
 * @brief Translates Isaac Sim's sensor_msgs → drdds hardware messages.
 *
 * This node makes Isaac Sim look like the real Lite3 hardware interface
 * by publishing /JOINTS_DATA and /IMU_DATA.
 *
 * CRITICAL: drdds/ImuData expects Roll/Pitch/Yaw
 **/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

// drdds messages (same as real hardware interface)
#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/imu_data.hpp"

#include <array>
#include <cstring>
#include <mutex>
#include <chrono>
#include <cmath>
#include <random>

namespace isaac_bridge {

class StateBridgeNode : public rclcpp::Node {
public:
    StateBridgeNode() : Node("state_bridge_node") {
        // ── Parameters ────────────────────────────────────────────────────
        this->declare_parameter("publish_rate_hz", 200);
        this->declare_parameter("joint_names", std::vector<std::string>{
            "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
            "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
            "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
            "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
        });

        int rate_hz = this->get_parameter("publish_rate_hz").as_int();
        joint_names_ = this->get_parameter("joint_names").as_string_array();

        // ── Publishers (drdds) ─────────────────────────────────────────────
        joints_pub_ = create_publisher<drdds::msg::JointsData>("/JOINTS_DATA", 10);
        imu_pub_    = create_publisher<drdds::msg::ImuData>("/IMU_DATA", 10);

        // ── Subscribers (Isaac Sim) ────────────────────────────────────────
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SystemDefaultsQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { OnJointState(msg); });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SystemDefaultsQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(msg); });

        // ── Timer ─────────────────────────────────────────────────────────
        auto period = std::chrono::milliseconds(1000 / rate_hz);
        timer_ = create_wall_timer(period, [this]() { PublishData(); });

        // Jitter for SDK initialization (Required for stationary simulation)
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::uniform_real_distribution<float>(-1e-5, 1e-5);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Publisher<drdds::msg::JointsData>::SharedPtr            joints_pub_;
    rclcpp::Publisher<drdds::msg::ImuData>::SharedPtr               imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::mutex data_mutex_;

    std::mt19937 gen_;
    std::uniform_real_distribution<float> dist_;

    // Robot state buffers
    std::array<float, 12> q_   = {};
    std::array<float, 12> dq_  = {};
    std::array<float, 12> tau_ = {};

    float rpy_[3]   = {}; // Rad
    float acc_[3]   = {};
    float omega_[3] = {};

    void OnJointState(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t idx = 0; idx < msg->name.size(); ++idx) {
            for (int j = 0; j < 12; ++j) {
                if (msg->name[idx] == joint_names_[j]) {
                    if (idx < msg->position.size()) q_[j]   = static_cast<float>(msg->position[idx]);
                    if (idx < msg->velocity.size()) dq_[j]  = static_cast<float>(msg->velocity[idx]);
                    if (idx < msg->effort.size())   tau_[j] = static_cast<float>(msg->effort[idx]);
                    break;
                }
            }
        }
    }

    void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Quat → RPY (Rad)
        const auto& q = msg->orientation;
        rpy_[0] = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        rpy_[1] = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
        rpy_[2] = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        acc_[0]   = msg->linear_acceleration.x;
        acc_[1]   = msg->linear_acceleration.y;
        acc_[2]   = msg->linear_acceleration.z;

        omega_[0] = msg->angular_velocity.x;
        omega_[1] = msg->angular_velocity.y;
        omega_[2] = msg->angular_velocity.z;
    }

    void PublishData() {
        auto now = get_clock()->now();
        std::lock_guard<std::mutex> lock(data_mutex_);

        // /JOINTS_DATA
        drdds::msg::JointsData jd;
        jd.header.stamp = now;
        for (int i = 0; i < 12; ++i) {
            jd.data.joints_data[i].status_word = 1;
            jd.data.joints_data[i].position    = q_[i] + dist_(gen_); // Suble jitter for SDK init
            jd.data.joints_data[i].velocity    = dq_[i];
            jd.data.joints_data[i].torque      = tau_[i];
        }
        joints_pub_->publish(jd);

        // /IMU_DATA 
        drdds::msg::ImuData imu;
        imu.header.stamp = now;
        imu.data.roll    = rpy_[0];
        imu.data.pitch   = rpy_[1];
        imu.data.yaw     = rpy_[2];
        imu.data.omega_x = omega_[0];
        imu.data.omega_y = omega_[1];
        imu.data.omega_z = omega_[2];
        imu.data.acc_x   = acc_[0];
        imu.data.acc_y   = acc_[1];
        imu.data.acc_z   = acc_[2];
        imu_pub_->publish(imu);
    }
};

} // namespace isaac_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<isaac_bridge::StateBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
