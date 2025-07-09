#include "joy_to_motor.cpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToMotorNode>());
    rclcpp::shutdown();
    return 0;
}
