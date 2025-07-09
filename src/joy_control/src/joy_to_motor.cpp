#include "rclcpp/rclcpp.hpp"
#include "ros2_msgs/msg/joy_control.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
//#include "ros2_msgs/msg/motor_commands.hpp"

class JoyToMotorNode : public rclcpp::Node {
public:
    JoyToMotorNode() : Node("joy_to_motor_node") {
        joy_sub_ = this->create_subscription<ros2_msgs::msg::JoyControl>(
            "joy_control", 10,
            std::bind(&JoyToMotorNode::joy_callback, this, std::placeholders::_1));

        motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_commands", 10);
    }

private:
    rclcpp::Subscription<ros2_msgs::msg::JoyControl>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
    //rclcpp::Publisher<ros2_msgs::msg::MotorCommands>::SharedPtr motor_pub_;

    void joy_callback(const ros2_msgs::msg::JoyControl::SharedPtr msg) {

        float t = 50 * msg->throttle;
        float r = msg->roll;
        float p = msg->pitch;
        float y = msg->yaw;
        //RCLCPP_INFO(this->get_logger(), "joy callback: %f", r);

        float k_f = 3.0e-6;

        float k = 1;
        // Basic mixer for quadrotor in X or + config
        float m0 = k * (t/4 - r/2 - p/2 - y/4);  // Front Left
        float m1 = k * (t/4 + r/2 - p/2 + y/4);  // Front Right
        float m2 = k * (t/4 + r/2 + p/2 - y/4);  // Rear Right
        float m3 = k * (t/4 - r/2 + p/2 + y/4);  // Rear Left

        //ros2_msgs::msg::MotorCommands motors;
        //motors.motor_speeds = {m0, m1, m2, m3};
        auto motors = std_msgs::msg::Float32MultiArray();
        motors.data = {m0, m1, m2, m3};
      //  RCLCPP_INFO(this->get_logger(), "motor speeds from joy callback: %f, %f, %f, %f", m0, m1, m2, m3);
        motor_pub_->publish(motors);
    }
};
