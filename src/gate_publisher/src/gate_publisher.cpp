#include <thread>
#include <iostream>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "gate_msgs/msg/gate.hpp"
#include "gate_msgs/msg/gate_array.hpp"
#include <btBulletDynamicsCommon.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class GatePublisher : public rclcpp::Node
{
public:
    GatePublisher() : Node("gate_publisher")
    {
		init_bullet();
        publisher_ = this->create_publisher<gate_msgs::msg::GateArray>("gates", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&GatePublisher::publish_gates, this));
    }

	struct Gate {
		btRigidBody* body;
		btVector3 position;
		float width, height, thickness;
	};

	std::vector<Gate> gates;

	void createGate(btDiscreteDynamicsWorld* world, btVector3 pos, float w, float h, float t)
	{
		// Create collision shape
		btCollisionShape* gateShape = new btBoxShape(btVector3(w/2, h/2, t/2));
		
		// Set transform
		btTransform gateTransform;
		gateTransform.setIdentity();
		gateTransform.setOrigin(pos);
		
		// Static object (mass=0)
		btScalar mass(0);
		btVector3 localInertia(0,0,0);

		btDefaultMotionState* motionState = new btDefaultMotionState(gateTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, gateShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		world->addRigidBody(body);

		gates.push_back({body, pos, w, h, t});
	}


private:
    void init_bullet()
    {
        // Bullet setup
        btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
        btBroadphaseInterface* broadphase = new btDbvtBroadphase();
        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
        btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);

        dynamicsWorld->setGravity(btVector3(0, 0, -9.81));

        // Ground plane
        btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1)
    , 0);
        btDefaultMotionState* groundMotion = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
        btRigidBody::btRigidBodyConstructionInfo groundCI(0, groundMotion, groundShape, btVector3(0, 0, 0));
        btRigidBody* groundBody = new btRigidBody(groundCI);
        dynamicsWorld->addRigidBody(groundBody);

        //add gates
        createGate(dynamicsWorld, btVector3(5, 0, 0), 2.0f, 2.0f, 0.1);
        createGate(dynamicsWorld, btVector3(10, 10, 5), 2.0f, 2.0f, 0.1);
        createGate(dynamicsWorld, btVector3(15, 20, -3), 2.0f, 2.0f, 0.1);
 
    }

    void publish_gates()
    {
        auto msg = gate_msgs::msg::GateArray();
 /*
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
*/
        for (Gate g : gates)
        {
            gate_msgs::msg::Gate gate;
            gate.position.x = g.position.getX();
            gate.position.y = g.position.getY();
            gate.position.z = g.position.getZ();

            gate.width = g.width;
            gate.height = g.height;
            gate.thickness = g.thickness;

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

