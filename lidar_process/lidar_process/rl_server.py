import os
import signal
import sys
import copy
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header, Int32
from geometry_msgs.msg import Point, Twist
from cabot_msgs.msg import PoseLog # type: ignore
from lidar_process_msgs.msg import PositionArray, PositionHistoryArray, RobotMessage #type: ignore
# from lidar_process_msgs.srv import RlAction

from .sgan import inference
from . import crowd_attn_rl 

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data

class RLServer(Node):

    def __init__(self):
        super().__init__('rl_server')

        entity_callback_group = MutuallyExclusiveCallbackGroup()
        server_callback_group = ReentrantCallbackGroup()
        #transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sensor_data_qos = qos_profile_sensor_data

        self.entity_sub = self.create_subscription(
            PositionHistoryArray , 
            '/entity_histories',
            self.entity_cb,
            qos_profile=sensor_data_qos,
            callback_group = entity_callback_group
        )

        self.robot_sub = self.create_subscription(
            RobotMessage , 
            '/rl_robot_info',
            self.robot_cb,
            qos_profile=sensor_data_qos,
            callback_group = server_callback_group
        )

        self.robot_pub = self.create_publisher(
            Twist,
            '/rl_robot_cmd',
            10
        )

        timer_period = 0.05  # seconds
        self.pub_timer = self.create_timer(timer_period, self.robot_timer_cb)

        self.positions_history = []
        self.robot_observation = {"robot_pos": np.array([0.0, 0.0]),
                                    "robot_vel": np.array([0.0,0.0]),
                                    "robot_goal": np.array([0.0, 0.0]),
                                    "robot_th": 0.0,
                                    "pedestrians_pos": [],
                                    "pedestrians_pos_history": []}

        sgan_model_path = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                        "sgan-models", 
                                        "eth_8_model.pt")
        rl_model_path = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                        "trained-models")
        self.agent = crowd_attn_rl.CrowdAttnRL(sgan_model_path, rl_model_path, "41665.pt")

        # self.rl_srv = self.create_service(RlAction, 'rl_action', self.rl_action_cb, callback_group = server_callback_group)

        return
    
    def robot_cb(self, msg):
        robot_pos = msg.robot_pos
        robot_vel = msg.robot_vel
        robot_th = msg.robot_th
        robot_goal = msg.robot_goal

        positions_history = copy.copy(self.positions_history)

        observation = {}
        robot_pos_np = np.array([robot_pos.x, robot_pos.y])
        robot_vel_np = np.array([robot_vel.x, robot_vel.y])
        robot_goal_np = np.array([robot_goal.x, robot_goal.y])
        observation["robot_pos"] = robot_pos_np
        observation["robot_vel"] = robot_vel_np
        observation["robot_th"] = robot_th
        observation["robot_goal"] = robot_goal_np
        observation["pedestrians_pos_history"] = np.array(positions_history)
        if (len(positions_history) == 0):
            observation["pedestrians_pos"] = []
        else:
            pedestrians_pos = []
            for i in range(len(positions_history)):
                pedestrians_pos.append(positions_history[i][-1])
            observation["pedestrians_pos"] = np.array(pedestrians_pos)

        self.robot_observation = observation
        return
    
    def robot_timer_cb(self):
        msg = Twist()
        action = self.agent.act(self.robot_observation)
        msg.linear.x = float(action[0])
        msg.angular.z = float(action[1])
        self.robot_pub.publish(msg)
        return
    
    def entity_cb(self, msg):
        self.positions_history = []

        entity_history = msg.positions_history
        time_steps = len(entity_history)
        for i in range(time_steps):
            positions = []
            entity_msg = entity_history[i]
            id_array = entity_msg.ids
            pos_array = entity_msg.positions
            num_entities = len(id_array)
            entity_ids_np = np.zeros(num_entities)
            entity_pos_np = np.zeros((num_entities, 2))
            for j in range(num_entities):
                entity_ids_np[j] = id_array[j].data
                entity_pos_np[j, 0] = pos_array[j].x
                entity_pos_np[j, 1] = pos_array[j].y
            unique_entity_ids = np.sort(np.unique(entity_ids_np))
            for id in unique_entity_ids:
                condition = (entity_ids_np == id)
                pos = np.mean(entity_pos_np[condition, :], axis=0)
                positions.append(pos)
            self.positions_history.append(positions)
        self.positions_history = np.array(self.positions_history)
        self.positions_history = np.transpose(self.positions_history, (1, 0, 2))
        return

def main():
    rclpy.init()
    rl_server = RLServer()
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(rl_server, executor)
    except KeyboardInterrupt:
        #process_thread.join()
        rl_server.get_logger().info("Shutting down")
    except Exception as err:
        rl_server.get_logger().error(err)

    rl_server.destroy_node()
    rclpy.shutdown()
    return


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)
    return

signal.signal(signal.SIGINT, receiveSignal)

if __name__ == '__main__':
    main()