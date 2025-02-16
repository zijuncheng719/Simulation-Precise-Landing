#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 包含必要的头文件
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
class ManualControlPublisher : public rclcpp::Node
{
public:
    ManualControlPublisher() : Node("manual_control_publisher")
    {
        // 声明参数
        this->declare_parameter<float>("roll", 0.0);
        this->declare_parameter<float>("pitch", 0.0);
        this->declare_parameter<float>("yaw", 0.0);
        this->declare_parameter<float>("throttle", 0.0);
        this->declare_parameter<float>("aux1", 0.0);
        this->declare_parameter<float>("aux2", 0.0);

        // 设置QoS策略
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); // 设置为BestEffort以确保兼容性

        publisher_ = this->create_publisher<px4_msgs::msg::ManualControlSetpoint>("/sim_fmu/out/manual_control_setpoint", qos);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ManualControlPublisher::publish_manual_control_setpoint, this)); // 设置为20ms，即50Hz
    }

private:
    float limit_value(float value, float min_value, float max_value) {
        if (value > max_value) {
            return max_value;
        } else if (value < min_value) {
            return min_value;
        } else {
            return value;
        }
    }

    void publish_manual_control_setpoint()
    {
        // 获取参数
        this->get_parameter("roll", roll_);
        this->get_parameter("pitch", pitch_);
        this->get_parameter("yaw", yaw_);
        this->get_parameter("throttle", throttle_);
        this->get_parameter("aux1", aux1_);
        this->get_parameter("aux2", aux2_);

        // 限制参数范围在-1到1之间
        roll_ = limit_value(roll_, -1.0, 1.0);
        pitch_ = limit_value(pitch_, -1.0, 1.0);
        yaw_ = limit_value(yaw_, -1.0, 1.0);
        throttle_ = limit_value(throttle_, -1.0, 1.0);
        aux1_ = limit_value(aux1_, -1.0, 1.0);
        aux2_ = limit_value(aux2_, -1.0, 1.0);

        auto message = px4_msgs::msg::ManualControlSetpoint();
        message.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        message.timestamp_sample = message.timestamp;
        message.valid = true;
        message.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_RC; // 设置数据源

        // 设置手动控制的值
        message.roll = roll_;
        message.pitch = pitch_;
        message.yaw = yaw_;
        message.throttle = throttle_;
        message.flaps = 0.0;
        message.aux1 = aux1_;
        message.aux2 = aux2_;
        message.aux3 = 0.0;
        message.aux4 = 0.0;
        message.aux5 = 0.0;
        message.aux6 = 0.0;
        message.sticks_moving = false;
        message.buttons = 0;

        RCLCPP_INFO(this->get_logger(), "Publishing manual control setpoint");
        publisher_->publish(message);
    }

    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    float roll_;
    float pitch_;
    float yaw_;
    float throttle_;
    float aux1_;
    float aux2_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControlPublisher>());
    rclcpp::shutdown();
    return 0;
}