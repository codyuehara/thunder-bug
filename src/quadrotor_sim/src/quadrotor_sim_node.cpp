#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <btBulletDynamicsCommon.h>

class QuadrotorSimNode : public rclcpp::Node
{
public:
    struct QuadrotorState {
        btVector3 pos; // world position
        btVector3 vel; // world vel
        btQuaternion quat; // orientation (body to world)
        btVector3 omega; // ang vel in body frame
    };
    struct QuadrotorDerivative {
        btVector3 dpos;
        btVector3 dvel;
        btQuaternion dquat;
        btVector3 domega;
    };       
 
    QuadrotorSimNode() : Node("quad_sim")
    {
        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/motor_commands", 10, 
            std::bind(&QuadrotorSimNode::motor_callback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/quad_pose", 10);

        init_bullet();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), 
                    std::bind(&QuadrotorSimNode::update, this));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    btDynamicsWorld* world_;
    btRigidBody* quad_body_;

    QuadrotorState state_;
    float motor_forces_[4] = {0, 0, 0, 0};
    btScalar mass; 
    btVector3 gravity;
    double Jx = 0.0023;
    double Jy = 0.0023;
    double Jz = 0.004;
    btMatrix3x3 J;
    btMatrix3x3 J_inv;
    float arm_length = 0.1f; // distance from the center to motor in meters
    float torque_constant = 0.001f; 

    void init_bullet()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing bullet");
btBroadphaseInterface* broadphase = new btDbvtBroadphase();
        btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

        world_ = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
        gravity.setValue(0.0, 0.0, -9.81);
        world_->setGravity(gravity);

        btCollisionShape* shape = new btBoxShape(btVector3(0.2, 0.05, 0.2)); // Quad body

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(0, 1, 0));

        mass = 1.0;
        J.setValue(
            Jx, 0, 0, 
            0, Jy, 0,
            0, 0, Jz
        );
        J_inv.setValue(
            1/Jx, 0.0, 0.0,
            0.0, 1/Jy, 0.0,
            0.0, 0.0, 1/Jz
        );

        btVector3 inertia(0, 0, 0);
        shape->calculateLocalInertia(mass, inertia);

        btDefaultMotionState* motionState = new btDefaultMotionState(transform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, inertia);
        quad_body_ = new btRigidBody(rbInfo);
        world_->addRigidBody(quad_body_);

        //init state
        state_.pos.setValue(0, 0, 0);
        state_.vel.setValue(0, 0, 0);
        state_.quat.setValue(1, 0, 0, 0);
        state_.omega.setValue(0, 0, 0);

    }

    void motor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "motor callback");
        if (msg->data.size() == 4)
        {
            for (int i = 0; i < 4; ++i)
            {
                motor_forces_[i] = msg->data[i];
        RCLCPP_INFO(this->get_logger(), "motor forces: %f, %f, %f, %f", motor_forces_[0], motor_forces_[1], motor_forces_[2], motor_forces_[3]); 
            }
        }
    }
    void update()
    {
//        RCLCPP_INFO(this->get_logger(), "step sim");

        double dt = 0.01;

        double temp1 = arm_length / std::sqrt(2) * (motor_forces_[0] + motor_forces_[1] - motor_forces_[2] - motor_forces_[3]);
        if (std::isnan(temp1)) temp1 = 0;
        double temp2 = arm_length / std::sqrt(2) * (-motor_forces_[0] + motor_forces_[1] + motor_forces_[2] - motor_forces_[3]);
        if (std::isnan(temp2)) temp2 = 0;
        double temp3 = torque_constant * (motor_forces_[0] - motor_forces_[1] + motor_forces_[2] - motor_forces_[3]);
        btVector3 torque(temp1, temp2, temp3);

        RCLCPP_INFO(this->get_logger(), "torque: %f, %f, %f", torque.x(), torque.y(), torque.z());

        btVector3 force(0, 0, (motor_forces_[0] + motor_forces_[1] + motor_forces_[2] + motor_forces_[3]));

        RCLCPP_INFO(this->get_logger(), "force: %f, %f, %f", force.x(), force.y(), force.z());
        //print_state();
        state_ = rk4_step(state_, torque, force, dt);
        print_state();

        publish_pose();
    }    

    void publish_pose()
    {    
        geometry_msgs::msg::Pose pose;
        pose.position.x = state_.pos.x();    
        pose.position.y = state_.pos.y();    
        pose.position.z = state_.pos.z();    

        pose.orientation.x = state_.quat.x();
        pose.orientation.y = state_.quat.y();
        pose.orientation.z = state_.quat.z();
        pose.orientation.w = state_.quat.w();

        pub_->publish(pose);

    }
    
    QuadrotorDerivative compute_dynamics(const QuadrotorState& s, const btVector3& torque_body, const btVector3& force)
    {

        QuadrotorDerivative d;
    
        // Rotation matrix from quaternion (world <- body)
        btMatrix3x3 R(s.quat);

        btVector3 thrust_world = R * force; 

        d.dpos = s.vel;
        RCLCPP_INFO(this->get_logger(), "d pos: %f, %f, %f", d.dpos.x(), d.dpos.y(), d.dpos.z());
        d.dvel = (1 / mass) * thrust_world + gravity;
        RCLCPP_INFO(this->get_logger(), "d vel: %f, %f, %f", d.dvel.x(), d.dvel.y(), d.dvel.z());

        btQuaternion omega_q(0, s.omega.x(), s.omega.y(), s.omega.z());
        d.dquat = omega_q * s.quat * 0.5;
        RCLCPP_INFO(this->get_logger(), "d quat: %f, %f, %f, %f", d.dquat.w(), d.dquat.x(), d.dquat.y(), d.dquat.z());

        btVector3 J_omega = J * s.omega;
        btVector3 cross = s.omega.cross(J_omega);
        btVector3 net_torque = torque_body - cross;
        d.domega = J_inv * net_torque;
        RCLCPP_INFO(this->get_logger(), "d omega: %f, %f, %f", d.domega.x(), d.domega.y(), d.domega.z());

        return d;
    }
    QuadrotorState rk4_step(const QuadrotorState& s, const btVector3& torque_body, const btVector3& force, double dt)
    {
        auto f = [&](const QuadrotorState& x) {
            return compute_dynamics(x, torque_body, force);
        }; 
        
        QuadrotorDerivative k1 = f(s);
        
        QuadrotorState s2 = s;
        s2.pos += 0.5 * dt * k1.dpos;
        s2.vel += 0.5 * dt * k1.dvel;
        s2.quat += k1.dquat * 0.5 * dt;
        s2.omega += 0.5 * dt * k1.domega;
        s2.quat.normalize();

        QuadrotorDerivative k2= f(s2);

        QuadrotorState s3 = s;
        s3.pos += 0.5 * dt * k2.dpos;
        s3.vel += 0.5 * dt * k2.dvel;
        s3.quat += k2.dquat * 0.5 * dt;
        s3.omega += 0.5 * dt * k2.domega;
        s3.quat.normalize();

        QuadrotorDerivative k3 = f(s3);

        QuadrotorState s4 = s;
        s4.pos += 0.5 * dt * k3.dpos;
        s4.vel += 0.5 * dt * k3.dvel;
        s4.quat += k3.dquat * 0.5 * dt;
        s4.omega += 0.5 * dt * k3.domega;
        s4.quat.normalize();

        QuadrotorDerivative k4 = f(s4);

        QuadrotorState result = s;
		result.pos += (dt / 6.0) * (k1.dpos + 2.0 * k2.dpos + 2.0 * k3.dpos + k4.dpos);
		result.vel += (dt / 6.0) * (k1.dvel + 2.0 * k2.dvel + 2.0 * k3.dvel + k4.dvel);
		result.quat += (k1.dquat + k2.dquat * 2.0 + k3.dquat * 2.0 + k4.dquat) * (dt / 6.0);
		result.omega += (dt / 6.0) * (k1.domega + 2.0 * k2.domega + 2.0 * k3.domega + k4.domega);
		result.quat.normalize(); 

        return result;		
    }

    void print_state()
    {
        RCLCPP_INFO(this->get_logger(), "pos: %f, %f, %f", state_.pos.x(), state_.pos.y(), state_.pos.z());
        RCLCPP_INFO(this->get_logger(), "vel: %f, %f, %f", state_.vel.x(), state_.vel.y(), state_.vel.z());
        RCLCPP_INFO(this->get_logger(), "quat: %f, %f, %f, %f", state_.quat.w(), state_.quat.x(), state_.quat.y(), state_.quat.z());
        RCLCPP_INFO(this->get_logger(), "omega: %f, %f, %f", state_.omega.x(), state_.omega.y(), state_.omega.z());

    }

};
