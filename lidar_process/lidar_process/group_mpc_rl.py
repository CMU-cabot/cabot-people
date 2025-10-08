import os
import numpy as np
import torch
import yaml
from time import time

# Import through group_rl
from .group_rl.config import get_args, check_args
from .group_rl.sim.mpc.group_linear_mpc import GroupLinearMPC
from .group_rl.sim.mpc import mpc_utils
from .group_rl.obs_data_parser import ObsDataParser

#### RL model
from .group_rl.rl.rl_agent import SAC
from .group_rl.rl.utils import load_config

class GroupRLMPC(object):
    """
    Hybrid Reinforcement Learning - MPC inference class
    Modified from main_eval.py to provide an act() method similar to crowd_attn_rl.py
    """
    
    def __init__(self, rl_model_weight_path,
                 rl_config_path="group_rl/rl_config.yaml", 
                 mpc_config_path="group_rl/crowd_mpc.config"):
        """
        Initialize the hybrid RL-MPC agent
        
        Args:
            rl_model_weight_path: Path to the trained RL model weights
            rl_config_path: Path to RL configuration file
            mpc_config_path: Path to MPC configuration file
        """

        self.robot_speed = 1.0 
        # self.human_num = 20
        
        # Device setup
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Load configurations
        self.rl_config = load_config(rl_config_path)
        self.mpc_config = mpc_utils.parse_config_file(mpc_config_path)
        
        # Get args from config
        self.args = get_args()
        
        # Initialize RL agent
        self.rl_agent = SAC(self.rl_config["state_shape"], self.rl_config["action_shape"],
                           self.rl_config["latent_dim"], self.device)
        
        # Load pretrained RL model
        self.with_exploration = False
        self.rl_agent.load_pretrained_agent(rl_model_weight_path)
        
        # Initialize observation data parser
        self.obs_data_parser = ObsDataParser(self.mpc_config, self.args)
        
        # Calculate max follow position delta
        self.max_follow_pos_delta = (self.mpc_config.getint('mpc_env', 'mpc_horizon') *
                                   self.mpc_config.getfloat('mpc_env', 'max_speed'))
        
        # Track processing times
        self.rl_time = []
        self.mpc_time = []
        
        # MPC steps counter
        self.mpc_steps_counter = 0
        self.mpc_steps_per_follow_state = 2
        self.max_ang_speed = 0.785
        
        # Store current follow state
        self.current_follow_state = None
        
        # Initialize MPC (will be reinitialized for each episode)
        self.mpc = None

        print("Load successful: RL model from {}, MPC config from {}".format(
            rl_model_weight_path, mpc_config_path))

        return
    
    def preprocess_rl_obs(self, obs, current_state, robot_vx, robot_vy, goal_pos):
        """
        Preprocess observation for RL model
        """
        obs = obs.copy()
        current_state = current_state.copy()
        current_pos = current_state[:2].reshape(1, -1)
        obs[:, :2] = obs[:, :2] - current_pos
        obs[obs > 1e4] = 0

        obs[:, 2] = obs[:, 2] - robot_vx
        obs[:, 3] = obs[:, 3] - robot_vy

        goal_pos = np.array(goal_pos).reshape(1, -1)
        goal_pos = goal_pos - current_pos
        goal_vx_vy = np.array([-robot_vx, -robot_vy]).reshape(1, -1)
        obs = obs.reshape(1, -1)
        obs = np.concatenate([goal_pos, goal_vx_vy, obs], axis=1)
        return obs
    
    def get_rl_follow_state(self, obs):
        """
        Use RL model to generate follow state
        """
        rl_time_start = time()

        goal_pos = obs['robot_goal']
        
        # Get robot state
        current_state, target, robot_speed, robot_motion_angle = self.obs_data_parser.get_robot_state(obs)
        robot_vx = robot_speed * np.cos(robot_motion_angle)
        robot_vy = robot_speed * np.sin(robot_motion_angle)
        
        # Get human state
        nearby_human_state = self.obs_data_parser.get_human_state(obs)
        
        # Preprocess observation for RL
        rl_obs = self.preprocess_rl_obs(nearby_human_state, current_state, robot_vx, robot_vy, goal_pos)
        
        # Get RL action
        if self.with_exploration:
            rl_actions, _, entropies = self.rl_agent.get_action(
                torch.FloatTensor(rl_obs).to(self.device), with_exploration=self.with_exploration)
        else:
            rl_actions = self.rl_agent.get_action(
                torch.FloatTensor(rl_obs).to(self.device), with_exploration=self.with_exploration)
        rl_actions = rl_actions.cpu().detach().numpy()
        
        # Rescale actions
        follow_pos = rl_actions[0, :2].copy()
        follow_pos = follow_pos * self.max_follow_pos_delta
        # Convert relative pos to global pos
        follow_pos = follow_pos + current_state[:2]
        
        follow_state = np.array([follow_pos[0], follow_pos[1], 0.0, 0.0])
        follow_state = follow_state.reshape(1, -1)
        
        rl_time_end = time()
        rl_time = rl_time_end - rl_time_start
        print("RL follow state:", follow_state, "Time:", rl_time)
        self.rl_time.append(rl_time)

        ############ use fixed way to generate a follow state ##
        # follow_state = obs_data_parser.get_follow_state(obs, robot_motion_angle, target) ## follow_state is (4,): pos_x, pos_y, speed, motion_angle
        ########################################################
        
        return follow_state, target
    
    def act(self, obs):
        """
        Given an observation, return an action using hybrid group RL-MPC approach
        
        Args:
            obs: Observation dictionary containing robot and environment state
            
        Returns:
            action: MPC action output
        """
        robot_goal = obs['robot_goal']
        robot_pos = obs['robot_pos']
        robot_th = obs['robot_th']
        robot_speed = self.robot_speed
        if obs['num_pedestrians'] == 0:
            desired_th = np.arctan2(robot_goal[1] - robot_pos[1], robot_goal[0] - robot_pos[0])
            action = np.zeros(2)
            action[0] = robot_speed
            action[1] = desired_th - robot_th
            return action, robot_goal
        
        # Use existing follow state and get target
        current_state, target, robot_speed, robot_motion_angle = self.obs_data_parser.get_robot_state(obs)
        
        # Check if we need to update follow state (every 10 MPC steps)
        if self.mpc_steps_counter % self.mpc_steps_per_follow_state == 0:
            # Use RL to generate new follow state
            self.current_follow_state, target = self.get_rl_follow_state(obs)
        
        # Use MPC to generate action
        mpc_time_start = time()
        action = self.mpc.get_action(obs, target, self.current_follow_state)
        v_pref = self.robot_speed
        if action[0]> v_pref:
            action[0] = v_pref
        if action[0] < 0.0:
            action[0] = 0.0
        if action[1] > self.max_ang_speed:
            action[1] = self.max_ang_speed
        if action[1] < -self.max_ang_speed:
            action[1] = -self.max_ang_speed
        mpc_time_end = time()
        
        mpc_time = mpc_time_end - mpc_time_start
        print("MPC action:", action, "Time:", mpc_time)
        self.mpc_time.append(mpc_time)
        
        # Update counter
        self.mpc_steps_counter += 1
        
        return action, self.current_follow_state[0][:2]
    
    def act_rl(self, obs):
        """
        Given an observation, return people array and sub-goal using RL only
        
        Args:
            obs: Observation dictionary containing robot and environment state
            
        Returns:
            people_array: PeopleArray message for cabot-people
            sub_goal: Sub-goal position from RL
        """
        robot_goal = obs['robot_goal']
        if obs['num_pedestrians'] == 0:
            return [], robot_goal

        # Use RL to generate follow state
        self.current_follow_state, target = self.get_rl_follow_state(obs)
        sub_goal = self.current_follow_state[0][:2]

        people_array = self.mpc.get_people_array(obs)
        
        return people_array, sub_goal
    
    def reset(self):
        """
        Reset the agent state for a new episode
        """
        self.mpc_steps_counter = 0
        # self.current_follow_state = None
        self.mpc = GroupLinearMPC(self.mpc_config, self.args)
    
    def get_processing_time(self):
        """
        Get average processing times for RL and MPC components
        
        Returns:
            rl_time: Average RL processing time
            mpc_time: Average MPC processing time
        """
        if len(self.rl_time) == 0 or len(self.mpc_time) == 0:
            return None, None
        else:
            return np.mean(self.rl_time), np.mean(self.mpc_time)
    
    def get_total_processing_time(self):
        """
        Get total processing time (RL + MPC)
        
        Returns:
            total_time: Average total processing time
        """
        rl_time, mpc_time = self.get_processing_time()
        if rl_time is not None and mpc_time is not None:
            return rl_time + mpc_time
        return None