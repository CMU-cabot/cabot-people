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
from people_msgs.msg import People
from cabot_msgs.msg import PoseLog # type: ignore
from visualization_msgs.msg import Marker
from lidar_process_msgs.msg import PositionArray, PositionHistoryArray, RobotMessage #type: ignore
# from lidar_process_msgs.srv import RlAction

from .sgan import inference
from . import crowd_attn_rl 
from . import group_mpc_rl
from . import utils
from . import grouping
from . import visualization

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data

USE_PED_TRACKER = True
USE_GROUP_RL = True
USE_CPP_MPC = True

class RLServer(Node):

    def __init__(self):
        super().__init__('rl_server')

        entity_callback_group = MutuallyExclusiveCallbackGroup()
        server_callback_group = ReentrantCallbackGroup()
        #transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sensor_data_qos = qos_profile_sensor_data

        if not USE_PED_TRACKER:
            self.get_logger().info("Not using pedestrian tracker, using lidar-based entities instead")
            self.entity_sub = self.create_subscription(
                PositionHistoryArray , 
                '/entity_histories',
                self.entity_cb,
                qos_profile=sensor_data_qos,
                callback_group = entity_callback_group
            )
        else:
            self.get_logger().info("Using pedestrian tracker")
            max_queue_size = 100
            self.dt = 0.1
            self.history_length = 8
            self.people_history_queue = utils.SimpleQueue(max_queue_size)
            self.entity_sub = self.create_subscription(
                People, 
                '/people',
                self.people_cb,
                qos_profile=sensor_data_qos,
                callback_group = entity_callback_group
            )

        self.robot_sub = self.create_subscription(
            RobotMessage , 
            '/rl_robot_info',
            self.robot_cb,
            qos_profile=sensor_data_qos,
            callback_group = entity_callback_group
        )

        if not USE_CPP_MPC:
            self.robot_pub = self.create_publisher(
                Twist,
                '/rl_robot_cmd',
                10,
                callback_group = entity_callback_group
            )
        else:
            self.rl_subgoal_pub = self.create_publisher(
                Point,
                '/rl_subgoal',
                10,
                callback_group = entity_callback_group
            )
            self.people_pub = self.create_publisher(
                PositionHistoryArray,
                '/rl_people',
                10,
                callback_group = entity_callback_group
            )

        self.vis_pub = self.create_publisher(
            Marker,
            '/rl_subgoal_vis',
            10,
            callback_group = entity_callback_group
        )

        self._high_level_pos_threshold = self.declare_parameter('high_level_pos_threshold', 1.5).value
        self._high_level_vel_threshold = self.declare_parameter('high_level_vel_threshold', 0.5).value
        self._high_level_ori_threshold = self.declare_parameter('high_level_ori_threshold', 15.0).value
        self._high_level_ori_threshold = self._high_level_ori_threshold / 180 * np.pi
        self._static_threshold = self.declare_parameter('static_threshold', 0.3).value

        self.vis_debug = True
        timer_period = 0.05  # seconds
        if USE_GROUP_RL:
            if not USE_CPP_MPC:
                self.pub_timer = self.create_timer(timer_period, self.robot_hybrid_cb)
            else:
                self.pub_timer = self.create_timer(timer_period, self.robot_rl_cb)
        else:
            self.pub_timer = self.create_timer(timer_period, self.robot_hybrid_cb)

        self.positions_history = []
        self.velocities_history = []
        self.robot_observation = {"robot_pos": np.array([0.0, 0.0]),
                                    "robot_vel": np.array([0.0,0.0]),
                                    "robot_goal": np.array([0.0, 0.0]),
                                    "num_pedestrians": 0,
                                    "robot_th": 0.0,
                                    "pedestrians_pos": [],
                                    "pedestrians_pos_history": [],
                                    "pedestrians_vel": [],
                                    "pedestrians_vel_history": [],
                                    "group_labels": []}

        sgan_model_path = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                        "sgan-models", 
                                        "eth_8_model.pt")
        
        if not USE_GROUP_RL:
            rl_model_path = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                            "crowdattn-trained-models")
            self.agent = crowd_attn_rl.CrowdAttnRL(sgan_model_path, rl_model_path, "41665.pt")
        else:
            # n_samples_0100000.zip
            # spd_1_omega_0785.zip
            rl_model_fpath = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                            "group-rl-models",
                                            "spd_1_omega_0785.zip")
            rl_config_path = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                            "group-rl-configs",
                                            "rl_config.yaml")
            mpc_config_path = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                            "group-rl-configs",
                                            "crowd_mpc.config")
            self.agent = group_mpc_rl.GroupRLMPC(rl_model_fpath, rl_config_path, mpc_config_path)

        self.agent.reset()

        # self.rl_srv = self.create_service(RlAction, 'rl_action', self.rl_action_cb, callback_group = server_callback_group)

        return
    
    def robot_cb(self, msg):
        # collect observation information from the robot controller
        robot_pos = msg.robot_pos
        robot_vel = msg.robot_vel
        robot_th = msg.robot_th
        robot_goal = msg.robot_goal

        positions_history = copy.copy(self.positions_history)
        velocities_history = copy.copy(self.velocities_history)

        observation = {}
        robot_pos_np = np.array([robot_pos.x, robot_pos.y])
        robot_vel_np = np.array([robot_vel.linear.x, robot_vel.angular.z])
        robot_goal_np = np.array([robot_goal.x, robot_goal.y])
        observation["num_pedestrians"] = len(positions_history)
        observation["robot_pos"] = robot_pos_np
        observation["robot_vel"] = robot_vel_np
        observation["robot_th"] = robot_th
        observation["robot_goal"] = robot_goal_np
        observation["pedestrians_pos_history"] = np.array(positions_history)
        observation["pedestrians_vel_history"] = np.array(velocities_history)
        if (len(positions_history) == 0):
            observation["pedestrians_pos"] = []
            observation["pedestrians_vel"] = []
            observation["group_labels"] = []
        else:
            pedestrians_pos = []
            pedestrians_vel = []
            for i in range(len(positions_history)):
                pedestrians_pos.append(positions_history[i][-1])
                pedestrians_vel.append(velocities_history[i][-1])
            observation["pedestrians_pos"] = np.array(pedestrians_pos)
            observation["pedestrians_vel"] = np.array(pedestrians_vel)
            observation["group_labels"] = grouping.pedestrian_grouping(
                pedestrians_pos, 
                pedestrians_vel, 
                self._high_level_pos_threshold, 
                self._high_level_vel_threshold, 
                self._high_level_ori_threshold,
                self._static_threshold)

        if not(abs(robot_goal.x - self.robot_observation["robot_goal"][0]) < 1e-6 and
               abs(robot_goal.y - self.robot_observation["robot_goal"][1]) < 1e-6):
            self.agent.reset()

        self.robot_observation = observation
        #print("Robot obs;", self.robot_observation)
        self.get_logger().debug("Robot pos: {}, Goal: {}, Num ped: {}".format(robot_pos_np, robot_goal_np, observation["num_pedestrians"]))
        return
    
    def robot_hybrid_cb(self):
        # public command for full stack RL
        msg = Twist()
        action, sub_goal = self.agent.act(self.robot_observation)
        self.get_logger().info("Action: {}".format(action))
        msg.linear.x = float(action[0])
        msg.angular.z = float(action[1])
        self.robot_pub.publish(msg)
        if self.vis_debug:
            marker, _ = visualization.create_entity_marker(
                float(sub_goal[0]), float(sub_goal[1]), 9999, Header(frame_id="map"), "rl_subgoal_vis", marker_size=0.5, text_marker_needed=False)
            self.vis_pub.publish(marker)
        return

    def robot_rl_cb(self):
        # public info for controller from RL
        
        # people_array is a NxTx2 array
        people_array, sub_goal = self.agent.act_rl(self.robot_observation)
        self.get_logger().info("RL Sub-goal: {}".format(sub_goal))
        sub_goal_msg = Point()
        sub_goal_msg.x = float(sub_goal[0])
        sub_goal_msg.y = float(sub_goal[1])
        self.rl_subgoal_pub.publish(sub_goal_msg)
        self.get_logger().info("Point published")

        try:
            people_array_msg = PositionHistoryArray()
            if people_array is None:
                people_array_msg.horizon = 0
                people_array_msg.positions_history = []
            else:
                num_people = len(people_array)
                if num_people == 0:
                    num_timesteps = 0
                else:
                    num_timesteps = people_array.shape[1]
                people_array_msg.horizon = num_timesteps
                for i in range(num_timesteps):
                    pos_array_msg = PositionArray()
                    for j in range(num_people):
                        pos_msg = Point()
                        pos_msg.x = float(people_array[j, i, 0])
                        pos_msg.y = float(people_array[j, i, 1])
                        pos_array_msg.positions.append(pos_msg)
                        pos_array_msg.ids.append(Int32(data=j))
                        pos_array_msg.quantity = num_people
                    people_array_msg.positions_history.append(pos_array_msg)
            self.people_pub.publish(people_array_msg)
        except Exception as e:
            self.get_logger().error("Error publishing people_array: {}".format(e))
            return
        self.get_logger().info("People array poublished")
        if self.vis_debug:
            marker, _ = visualization.create_entity_marker(
                float(sub_goal[0]), float(sub_goal[1]), 9999, Header(frame_id="map"), "rl_subgoal_vis", marker_size=0.5, text_marker_needed=False)
            self.vis_pub.publish(marker)
        return

    
    def entity_cb(self, msg):
        # If using pure lidar tracking-based entities
        self.positions_history = []
        self.velocities_history = []

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
        if num_entities == 0:
            self.positions_history = np.array([])
            self.velocities_history = np.array([])
        else:
            self.positions_history = np.array(self.positions_history)
            self.positions_history = np.transpose(self.positions_history, (1, 0, 2))
            self.velocities_history = np.diff(self.positions_history, axis=1) / self.dt
            self.velocities_history = np.concatenate((self.velocities_history, 
                                                    self.velocities_history[:, -1:, :]), axis=1)
        return
    
    def people_cb(self, msg):
        # If using cabot-people tracker
        self.positions_history = []
        self.velocities_history = []

        people_history_entity = {}
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        people_history_entity["time"] = current_time

        # Parse the people message
        people = msg.people
        current_ids = []
        tmp_positions_history = {}
        tmp_velocities_history = {}
        tmp_lastest_velocities = {}
        for person in people:
            person_id = person.name
            if (person_id == "time") or (person_id == ""):
                self.get_logger().warn("Person ID cannot be 'time' or empty string")
                continue
            pos = [person.position.x, person.position.y]
            vel = [person.velocity.x, person.velocity.y]
            people_history_entity[person_id] = {"pos": pos, "vel": vel}

            current_ids.append(person_id)
            tmp_positions_history[person_id] = [pos]
            tmp_velocities_history[person_id] = [vel]
            tmp_lastest_velocities[person_id] = vel
            if self.people_history_queue.is_full():
                _ = self.people_history_queue.dequeue()
            self.people_history_queue.enqueue(people_history_entity)

        if not self.people_history_queue.is_full():
            self.get_logger().warn("Not enough history data, please wait for a few seconds")
            return
        
        # get history idxes wrt to current time, dt and history length
        history_idx_array = []
        history_idx = 0
        for i in range(self.history_length - 1):
            target_time = current_time - self.dt * (self.history_length - 1 - i)
            history_time = self.people_history_queue._items[history_idx]["time"]
            if (history_time > target_time) or (abs(history_time - target_time) < 1e-5):
                # self.get_logger().warn("Not enough history length!")
                history_idx_array.append(history_idx - 1)
                continue
            while history_time < target_time:
                history_idx += 1
                if history_idx >= len(self.people_history_queue._items):
                    self.get_logger().error("History resolution too slow. Could be due to large dt or small queue size. Please check.")
                    return
                history_time = self.people_history_queue._items[history_idx]["time"]
            history_idx_array.append(history_idx - 1)
        history_idx_array = history_idx_array[::-1]  # become new to old

        # fill in the history positions for each current person
        for person_id in current_ids:
            for i, idx in enumerate(history_idx_array):
                history_entity = self.people_history_queue._items[idx]
                history_entity_next = self.people_history_queue._items[idx + 1]
                if (person_id in history_entity) and (person_id in history_entity_next):
                    # linear interpolation
                    t1 = history_entity["time"]
                    t2 = history_entity_next["time"]
                    pos1 = history_entity[person_id]["pos"]
                    vel1 = history_entity[person_id]["vel"]
                    pos2 = history_entity_next[person_id]["pos"]
                    vel2 = history_entity_next[person_id]["vel"]
                    time = current_time - self.dt * (i + 1)
                    if t2 - t1 < 1e-5:
                        pos = pos1
                        vel = vel1
                    else:
                        pos = [pos1[0] + (pos2[0] - pos1[0]) * (time - t1) / (t2 - t1),
                               pos1[1] + (pos2[1] - pos1[1]) * (time - t1) / (t2 - t1)]
                        vel = [vel1[0] + (vel2[0] - vel1[0]) * (time - t1) / (t2 - t1),
                               vel1[1] + (vel2[1] - vel1[1]) * (time - t1) / (t2 - t1)]
                    tmp_positions_history[person_id].append(pos)
                    tmp_velocities_history[person_id].append(vel)
                    tmp_lastest_velocities[person_id] = vel
                else:
                    last_pos = tmp_positions_history[person_id][-1]
                    last_vel = tmp_lastest_velocities[person_id]
                    new_pos = [last_pos[0] - last_vel[0] * self.dt, last_pos[1] - last_vel[1] * self.dt]
                    tmp_positions_history[person_id].append(new_pos)
                    tmp_velocities_history[person_id].append(last_vel)

        for person_id in current_ids:
            pos_history = tmp_positions_history[person_id]
            vel_history = tmp_velocities_history[person_id]
            self.positions_history.append(pos_history[::-1])  # old to new in the end
            self.velocities_history.append(vel_history[::-1])
            
        return

def main():
    rclpy.init()
    rl_server = RLServer()
    try:
        executor = SingleThreadedExecutor()
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