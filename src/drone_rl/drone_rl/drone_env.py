import numpy as np
import threading
import time
import gym
from gym import spaces
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray, Empty

class ROS2Runner:
    def __init__(self, node_name='drone_racing_env_node'):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            pass
    def destroy(self):
        try:
            self.executor.shutdown()
        except Exception:
            pass
        self.node.destroy_node()

class DroneRacingEnv(gym.Env):
    metadata = {'render.modes':[]}
    
    def __init__(self):
        super().__init__()
        self.observation_space = spaces.Box(low=-np.inf,high=np.inf,shape=(31,),dtype=np.float32)
        self.action_space = spaces.Box(low=-1,high=1,shape=(4,),dtype=np.float32)    
        print("Initing")

        # first 15 are pos, vel, attitude in rot mat
        # [px, py, pz, vx, vy, vz, r11, r12, r13, r21, r22, r23, r31, r32, r33, 
        # next 12 are 4 gate corner positions of next gate
        # c1x, c1y, c1z, c2x, c2y, c2z, c3x, c3y, c3z, c4x, c4y, c4z,
        # last 4 are prev action aka mass normalized collective thrust & body rates
        # a1, a2, a3, a4]
        self._drone_state = np.zeros(15)
        self._gate_obs = np.zeros(12)
        #self._obs = np.zeros(31, dtype=np.float32)
        self._prev_action = np.zeros(4, dtype=np.float32)
        self.runner = None
        self.node = None

    def start_ros(self, node_name='drone_racing_env_node'):
        """
        Start ROS2 node in background thread. Call after env instantiation.
        """
        if self.runner is None:
            self.runner = ROS2Runner(node_name=node_name)
            self.node = self.runner.node
            self.action_pub = self.node.create_publisher(
                Float32MultiArray,
                '/drone_action',
                10
            )

            self.node.create_subscription(
                Float32MultiArray,
                '/drone_state',
                self._drone_state_callback,
                10
            )
            self.node.create_subscription(
                Float32MultiArray,
                '/gate_obs',
                self._gate_obs_callback,
                10
            )

    def _drone_state_callback(self, msg: Float32MultiArray):
        """
        Callback to update internal observation vector from ROS2 topic.
        """
        data = np.array(msg.data, dtype=np.float32)
        if data.shape[0] == 15:
            self._drone_state[:] = data
            print(data)
        else:
            # optional: ignore or log warning
            pass

    def _gate_obs_callback(self, msg: Float32MultiArray):
        """
        Callback to update internal observation vector from ROS2 topic.
        """
        data = np.array(msg.data, dtype=np.float32)
        if data.shape[0] == 15:
            self._gate_obs[:] = data
            print(data)
        else:
            # optional: ignore or log warning
            pass

    def reset(self):
        self._drone_state[:] = 0.0
        self._gate_obs[:] = 0.0
        self._prev_action[:] = 0.0
        obs = np.concatenate([self._drone_state, self._gate_obs, self._prev_action])
        return obs

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._prev_action = action.copy()

        r_prog = 1.0
        r_cmd = 1
        r_perc = 1
        r_crash = 0        

        obs = np.concatenate([self._drone_state, self._gate_obs, self._prev_action])
        reward = r_prog + r_perc + r_cmd - r_crash
        done = r_crash != 0.0
        info = {}
        return obs, reward, done, info
