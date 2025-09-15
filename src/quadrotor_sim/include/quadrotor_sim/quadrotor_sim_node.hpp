#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ros2_msgs/msg/joy_control.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "quadrotor_sim/battery_model.hpp"

using namespace std::chrono_literals;

struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector4d motor_speeds;

    State operator+(const State& other) const;
    
    State operator*(double scalar) const;
};

class QuadrotorSimNode : public rclcpp::Node
{
public:
    QuadrotorSimNode();
private:
    void initialize();
    Eigen::Vector4d clamp_thrust(const Eigen::Vector4d thrusts) const;
    Eigen::Quaterniond quaternion_derivative(const Eigen::Quaterniond& q, const Eigen::Vector3d& omega);
    State compute_derivatives(const State& s);
    State rk4_step(const State& s, double dt);
    void timer_callback();
    void joy_callback(const ros2_msgs::msg::JoyControl::SharedPtr msg);
    void collision_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr drone_state_pub_;
    rclcpp::Subscription<ros2_msgs::msg::JoyControl>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr collision_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double sim_dt_ = 0.001;
    double timer_dt_ = 0.01;

    int steps_per_callback = static_cast<int>(timer_dt_ / sim_dt_);
    double mass_;
    double hover_thrust_;
    
    State state_;
    
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inertia_inv_;
    
    Eigen::Vector3d gravity_;

    double thrust_;
    Eigen::Vector3d torque_;
    
    double thrust_min_;
    double thrust_max_;
    double k_mot_;
    double arm_length_;
    BatteryModel battery_;
    std::array<float, 4> motor_commands_;
};
