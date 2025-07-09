#include "quadrotor_sim/quadrotor_sim_node.hpp"

State State::operator+(const State& other) const {
    State result;
    result.position = position + other.position;
    result.velocity = velocity + other.velocity;
    result.orientation = (orientation.coeffs() + other.orientation.coeffs()).normalized();
    result.angular_velocity = angular_velocity + other.angular_velocity;
    return result;
}

State State::operator*(double scalar) const {
    State result;
    result.position = position * scalar;
    result.velocity = velocity * scalar;
    result.orientation = Eigen::Quaterniond(orientation.coeffs() * scalar);
    result.angular_velocity = angular_velocity * scalar;
    return result;
}

QuadrotorSimNode::QuadrotorSimNode()
: Node("quadrotor_sim_node"),
  mass_(1.0),
  thrust_min_(0.0),
  thrust_max_(100)
{
    joy_sub_ = this->create_subscription<ros2_msgs::msg::JoyControl>("joy_control", 10, std::bind(&QuadrotorSimNode::joy_callback, this, std::placeholders::_1));
    collision_sub_ = this->create_subscription<std_msgs::msg::String>("collision_event", 10, std::bind(&QuadrotorSimNode::collision_callback, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);

    initialize();
}

void QuadrotorSimNode::initialize()
{
    //init state
    state_.position.setZero();
    state_.velocity.setZero();
    state_.orientation = Eigen::Quaterniond::Identity();
    state_.angular_velocity.setZero();

    inertia_ = Eigen::Matrix3d::Zero();
    inertia_(0,0) = 0.01;
    inertia_(1,1) = 0.01;
    inertia_(2,2) = 0.02;

    inertia_inv_ = inertia_.inverse();

    gravity_ = Eigen::Vector3d(0, 0, -9.81);

    hover_thrust_ = mass_ * 9.81;
    //thrust_ = 0;
    thrust_ = hover_thrust_;
    torque_.setZero();

    timer_.reset();
    timer_ = this->create_wall_timer(10ms, 
                std::bind(&QuadrotorSimNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Quad sim started");
}

Eigen::Vector4d QuadrotorSimNode::clamp_thrust(const Eigen::Vector4d thrusts) const
{
    return thrusts.cwiseMax(thrust_min_).cwiseMin(thrust_max_);
}

Eigen::Quaterniond QuadrotorSimNode::quaternion_derivative(const Eigen::Quaterniond& q, const Eigen::Vector3d& omega)
{
    Eigen::Quaterniond omega_q(0, omega.x(), omega.y(), omega.z());
    Eigen::Quaterniond q_dot = Eigen::Quaterniond((q * omega_q).coeffs() * 0.5);
    return q_dot;
}

State QuadrotorSimNode::compute_derivatives(const State& s)
{
    State ds;

    //RCLCPP_INFO(this->get_logger(), "thrust: %f, torque x: %f, torque y: %f", thrust_, torque_[0], torque_[1]);
    Eigen::Vector3d thrust_world = s.orientation * Eigen::Vector3d(0,0,thrust_);
    Eigen::Vector3d accel = gravity_ + 1 / mass_ * thrust_world;
    Eigen::Vector3d ang_accel = inertia_inv_ * (torque_ - s.angular_velocity.cross(inertia_ * s.angular_velocity));
    Eigen::Quaterniond q_dot = quaternion_derivative(s.orientation, s.angular_velocity);

    ds.position = s.velocity;
    ds.velocity = accel;
    ds.orientation = q_dot;
    ds.angular_velocity = ang_accel;

    return ds;
}

State QuadrotorSimNode::rk4_step(const State& s, double dt)
{
    State k1 = compute_derivatives(s);
    State k2 = compute_derivatives(s + k1 * (dt/2.0));
    State k3 = compute_derivatives(s + k2 * (dt/2.0));
    State k4 = compute_derivatives(s + k3 * dt);

    State result;
    result.position = s.position + dt / 6.0 * (k1.position + 2*k2.position + 2*k3.position + k4.position);
    result.velocity = s.velocity + dt / 6.0 * (k1.velocity + 2*k2.velocity + 2*k3.velocity + k4.velocity);
    result.angular_velocity = s.angular_velocity + dt / 6.0 * (k1.angular_velocity + 2*k2.angular_velocity + 2*k3.angular_velocity + k4.angular_velocity);

    Eigen::Quaterniond q = s.orientation;
    Eigen::Quaterniond dq = k1.orientation;
    dq.coeffs() += 2.0 * k2.orientation.coeffs();
    dq.coeffs() += 2.0 * k3.orientation.coeffs();
    dq.coeffs() += k4.orientation.coeffs();
    dq.coeffs() *= dt / 6.0;
    result.orientation = Eigen::Quaterniond(q.coeffs() + dq.coeffs()).normalized();

    return result;

}

void QuadrotorSimNode::timer_callback()
{
    //rclcpp::Rate rate(1);
    for (int i = 0; i < steps_per_callback; ++i)
    {
        state_ = rk4_step(state_, sim_dt_);
    }       

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "world";
    
    pose_msg.pose.position.x = state_.position.x();
    pose_msg.pose.position.y = state_.position.y();
    pose_msg.pose.position.z = state_.position.z();

    pose_msg.pose.orientation.x = state_.orientation.x();
    pose_msg.pose.orientation.y = state_.orientation.y();
    pose_msg.pose.orientation.z = state_.orientation.z();
    pose_msg.pose.orientation.w = state_.orientation.w();

    pose_pub_->publish(pose_msg); 
    //rate.sleep();
}

void QuadrotorSimNode::joy_callback(const ros2_msgs::msg::JoyControl::SharedPtr msg)
{
    thrust_ = msg->throttle;
    torque_[2] = msg->roll;
    torque_[0] = msg->pitch;
    torque_[1] = msg->yaw;
}

void QuadrotorSimNode::collision_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "COLLISION");
    initialize();
}

/*
int main(int argc, char* argv[])
{
rclcpp::init(argc, argv);
auto node = std::make_shared<QuadrotorSimNode>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}
   
*/ 
