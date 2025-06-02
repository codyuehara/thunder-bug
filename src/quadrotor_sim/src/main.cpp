#include "quadrotor_sim_node.cpp"
#include "gate_publisher.cpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto sim_node = std::make_shared<QuadrotorSimNode>();
    auto gate_node = std::make_shared<GatePublisher>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(sim_node);
    exec.add_node(gate_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

