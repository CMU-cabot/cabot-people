"""Sampling MPC that scores rollouts by social momentum.

social_momentum_rl_mpc.py has always named a SocialMomentumMPC, but the class
never existed in HiCrowd-EXPO, so its MPC-only branch left self.mpc as None and
died on the first pedestrian. This is that class.

It is a GroupLinearMPC with one extra term in the rollout cost, mirroring what
CaBotSocialMomentumController computes on the nav2 side
(cabot_navigation2/plugins/cabot_social_momentum_controller.cpp,
calculateSocialMomentumCost): for every pedestrian the robot is interacting
with, the angular momentum of the robot-pedestrian pair about their midpoint is
rewarded for keeping its sign along the horizon and penalised for flipping. A
consistent sign means the robot commits to passing on one side instead of
dithering in front of someone.

This class lives here rather than in group_rl because group_rl is a checkout of
HiCrowd-EXPO, pinned by cabot's dependency-override.repos and gitignored, so
anything added there would not travel to another robot.
"""

import numpy as np

from .group_rl.sim.mpc.group_linear_mpc import GroupLinearMPC


def _cross2d(ax, ay, bx, by):
    return ax * by - ay * bx


class SocialMomentumMPC(GroupLinearMPC):
    def __init__(self, mpc_config, args):
        super().__init__(mpc_config, args)

        # Defaults are the nav2 ones from nav2_params_social_momentum.yaml, so
        # the Python and C++ sides behave the same when the config says nothing.
        def _f(key, default):
            try:
                return mpc_config.getfloat('mpc_env', key)
            except Exception:
                return default

        self.sm_interaction_distance = _f('sm_interaction_distance', 6.0)
        self.sm_interaction_angle_deg = _f('sm_interaction_angle_deg', 100.0)
        self.sm_consistent_reward = _f('sm_consistent_reward', 5.0)
        self.sm_switching_penalty = _f('sm_switching_penalty', 5.0)
        self.w_social = _f('w_social', 1.0)

    def evaluate_rollouts(self):
        # GroupLinearMPC fills rollout_costs with w_safe * collision + w_follow *
        # distance to the follow state; social momentum is added on top so the
        # safety and following behaviour is unchanged.
        super().evaluate_rollouts()

        if not self.has_ped or self.future_positions is None:
            return
        if self.rollouts is None or self.num_rollouts is None:
            return

        for i in range(self.num_rollouts):
            self.rollout_costs[i] += self.w_social * self._social_momentum_cost(self.rollouts[i])

    def _interacting(self, ped_start):
        """Pedestrians close enough and roughly in front, as isInteractingAgent does.

        Returns a list of (index, weight); the weight is 1 / distance, so someone
        at arm's length dominates someone at the edge of the horizon.
        """
        out = []
        for n in range(len(ped_start)):
            dx = ped_start[n][0] - self.robot_pos[0]
            dy = ped_start[n][1] - self.robot_pos[1]
            dist = float(np.hypot(dx, dy))
            if dist >= self.sm_interaction_distance:
                continue
            rel = np.degrees(np.arctan2(dy, dx) - self.robot_th)
            rel = (rel + 180.0) % 360.0 - 180.0
            if abs(rel) > self.sm_interaction_angle_deg:
                continue
            out.append((n, 1.0 / max(dist, 0.01)))
        return out

    def _social_momentum_cost(self, rollout):
        # rollout: (T, 2) robot positions. future_positions: (N, T, 2).
        steps = min(len(rollout), self.future_positions.shape[1])
        if steps < 2:
            return 0.0

        interacting = self._interacting(self.future_positions[:, 0])
        if not interacting:
            return 0.0

        # The C++ side reads one velocity per pedestrian off the prediction and
        # holds it, and takes the robot's velocity as the one it is travelling at
        # now -- the same for every candidate. Here each rollout is differentiated
        # instead, so a candidate is judged by the momentum it would actually
        # create rather than by the one the robot already has. Same quantity,
        # finer resolution: it is what makes the term discriminate between
        # rollouts that pass on opposite sides at the same speed.
        robot_vel = np.diff(rollout[:steps], axis=0) / self.dt

        cost = 0.0
        for n, weight in interacting:
            ped = self.future_positions[n]
            ped_vel = (ped[1] - ped[0]) / self.dt

            l_values = np.empty(steps - 1)
            for t in range(steps - 1):
                cx = 0.5 * (rollout[t][0] + ped[t][0])
                cy = 0.5 * (rollout[t][1] + ped[t][1])
                l_values[t] = (
                    _cross2d(rollout[t][0] - cx, rollout[t][1] - cy, robot_vel[t][0], robot_vel[t][1])
                    + _cross2d(ped[t][0] - cx, ped[t][1] - cy, ped_vel[0], ped_vel[1])
                )

            if len(l_values) < 2:
                continue

            ped_cost = 0.0
            for k in range(len(l_values) - 1):
                if l_values[k] * l_values[k + 1] > 0.0:
                    ped_cost -= self.sm_consistent_reward * abs(l_values[k])
                else:
                    ped_cost += self.sm_switching_penalty
            cost += weight * ped_cost

        return cost
