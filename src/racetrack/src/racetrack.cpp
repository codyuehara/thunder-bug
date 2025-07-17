#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ros2_msgs/srv/race_track.hpp"
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <memory>
#include <vector>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

class RaceTrackNode : public rclcpp::Node
{
public:
    RaceTrackNode() : Node("racetrack_node")
    {
        this->declare_parameter<std::string>("scenario", "default_track");
        std::string scenario_name;
        this->get_parameter("scenario", scenario_name);

        std::string yaml_path = ament_index_cpp::get_package_share_directory("racetrack") + "/config/" + scenario_name + ".yaml";
        RCLCPP_INFO(this->get_logger(), "loading scenario from: %s", yaml_path.c_str());

        YAML::Node root = YAML::LoadFile(yaml_path);
        YAML::Node config = root["racetrack_node"]["ros__parameters"];

        start_pose_ = parsePose(config["start_pose"]);
        for (const auto & gate : config["gate_poses"])
        {
            gate_poses_.push_back(parsePose(gate));
        }

        service_ = this->create_service<ros2_msgs::srv::RaceTrack>(
            "get_race_track", std::bind(&RaceTrackNode::handle_service, this, _1, _2));

        num_laps_ = config["num_laps"].as<int>();

        RCLCPP_INFO(this->get_logger(), "Race track loaded: %zu gates, %d laps", gate_poses_.size(), num_laps_);
    }

private:
    geometry_msgs::msg::Pose parsePose(const YAML::Node &node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node["position"]["x"].as<double>();
        pose.position.y = node["position"]["y"].as<double>();
        pose.position.z = node["position"]["z"].as<double>();
        pose.orientation.x = node["orientation"]["x"].as<double>();
        pose.orientation.y = node["orientation"]["y"].as<double>();
        pose.orientation.z = node["orientation"]["z"].as<double>();
        pose.orientation.w = node["orientation"]["w"].as<double>();
        return pose;
    }

    void handle_service(const std::shared_ptr<ros2_msgs::srv::RaceTrack::Request> request, std::shared_ptr<ros2_msgs::srv::RaceTrack::Response> response)
    {
        bool requested = request->request;
        if (!requested)
        {
            response->num_laps = 0;
            return;
        }
        response->start_pose = start_pose_;
        response->gate_poses = gate_poses_;
        response->num_laps = num_laps_;
        RCLCPP_INFO(this->get_logger(), "sent race track response ");
    }

    rclcpp::Service<ros2_msgs::srv::RaceTrack>::SharedPtr service_;
    geometry_msgs::msg::Pose start_pose_;
    std::vector<geometry_msgs::msg::Pose> gate_poses_;
    int num_laps_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RaceTrackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

