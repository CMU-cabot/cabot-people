import os
import numpy as np
import math
import torch
from time import time

from .group_mpc_rl import GroupRLMPC, cabot_speed_limit, load_mpc_config

# group_rl SAC checkpoints are full pickles; torch 2.6 (JetPack 6.2) would refuse them
from .torch_compat import patch_torch_load
patch_torch_load()

from .group_rl.config import get_args
from .group_rl.obs_data_parser import ObsDataParser
from .group_rl.sim.mpc import mpc_utils
from .social_momentum_mpc import SocialMomentumMPC


class SocialMomentumRLMPC(GroupRLMPC):
    """
    Social Momentum agent that extends GroupRLMPC.
    """

    def __init__(self, rl_model_weight_path,
                 rl_config_path=None,
                 mpc_config_path=None,
                 use_rl=True):
        """
        Initialize the Social Momentum agent.
        
        Args:
            rl_model_weight_path: Path to the trained RL model weights
            rl_config_path: Path to RL configuration file
            mpc_config_path: Path to MPC configuration file
            use_rl: If True, use RL for subgoal generation. If False, MPC-only mode.
        """
        if rl_config_path is None:
            rl_config_path = "group_rl/rl_config.yaml"
        if mpc_config_path is None:
            mpc_config_path = "group_rl/crowd_mpc.config"

        self.use_rl = use_rl
        if self.use_rl:
            super().__init__(
                rl_model_weight_path,
                rl_config_path,
                mpc_config_path,
            )
        else:
            self.robot_speed = cabot_speed_limit(1.0)
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.mpc_config = load_mpc_config(mpc_config_path)
            self.args = get_args()
            self.obs_data_parser = ObsDataParser(self.mpc_config, self.args)
            self.max_follow_pos_delta = (self.mpc_config.getint('mpc_env', 'mpc_horizon') *
                                         self.mpc_config.getfloat('mpc_env', 'max_speed'))
            self.rl_time = []
            self.mpc_time = []
            self.mpc_steps_counter = 0
            self.mpc_steps_per_follow_state = 2
            self.max_ang_speed = 0.785
            self.current_follow_state = None
            self.mpc = None
            print("Load successful: MPC config from {}".format(mpc_config_path))

        print("SocialMomentumRLMPC initialized. RL mode: {}".format(
            "RL+MPC" if self.use_rl else "MPC-only"))

    def reset(self):
        if self.use_rl:
            # GroupRLMPC.reset() is what builds self.mpc (a GroupLinearMPC).
            # Overriding reset() without calling it left self.mpc as the None set
            # in __init__, so act_rl() raised AttributeError on the first cycle
            # with a pedestrian in view.
            super().reset()
            return

        self.mpc_steps_counter = 0
        self.mpc = SocialMomentumMPC(self.mpc_config, self.args)

    def act(self, obs):
        if self.use_rl:
            return super().act(obs)

        robot_goal = obs['robot_goal']
        robot_pos = obs['robot_pos']
        robot_th = obs['robot_th']

        if obs['num_pedestrians'] == 0:
            # No pedestrians: simple proportional control toward goal
            desired_th = np.arctan2(robot_goal[1] - robot_pos[1],
                                    robot_goal[0] - robot_pos[0])
            action = np.zeros(2)
            action[0] = self.robot_speed
            action[1] = desired_th - robot_th
            return action, robot_goal

        # Use goal directly as follow state (no RL subgoal)
        follow_state = np.array([[robot_goal[0], robot_goal[1], 0.0, 0.0]])

       
        mpc_time_start = time()
        current_state, target, robot_speed, robot_motion_angle = \
            self.obs_data_parser.get_robot_state(obs)
        
        action = self.mpc.get_action(obs, target, follow_state)

        # Clamp velocities
        v_pref = self.robot_speed
        action[0] = np.clip(action[0], 0.0, v_pref)
        action[1] = np.clip(action[1], -self.max_ang_speed, self.max_ang_speed)

        mpc_time_end = time()
        self.mpc_time.append(mpc_time_end - mpc_time_start)

        self.mpc_steps_counter += 1

        return action, robot_goal

    def act_rl(self, obs):
        if self.use_rl:
            return super().act_rl(obs)
        robot_goal = obs['robot_goal']
        if obs['num_pedestrians'] == 0:
            return [], robot_goal

        people_array = self.mpc.get_people_array(obs)
        return people_array, robot_goal
