#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "gate_msgs/msg/gate.hpp"
#include "gate_msgs/msg/gate_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class GatePublisher : public rclcpp::Node
{
public:
    GatePublisher() : Node("gate_publisher")
    {
        publisher_ = this->create_publisher<gate_msgs::msg::GateArray>("gates", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&GatePublisher::publish_gates, this));
    }

private:
    void publish_gates()
    {
        auto msg = gate_msgs::msg::GateArray();

        for (int i = 0; i < 3; ++i)
        {
            gate_msgs::msg::Gate gate;

            gate.position.x = 5.0 * i;
            gate.position.y = 0.0;
            gate.position.z = 2.0;

            gate.width = 2.0;
            gate.height = 2.0;
            gate.thickness = 0.1;

            msg.gates.push_back(gate);
        }

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published %ld gates", msg.gates.size());
    }

    rclcpp::Publisher<gate_msgs::msg::GateArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GatePublisher>());
    rclcpp::shutdown();
    return 0;
}

