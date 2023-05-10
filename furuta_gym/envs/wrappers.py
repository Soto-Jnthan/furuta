import logging
import time
import gym
from gym.spaces import Box
import numpy as np

class ControlFrequency(gym.Wrapper):
    """
    Enforce a sleeping time (dt) between each step.
    """
    def __init__(self, env):
        super(ControlFrequency, self).__init__(env)
        self.dt = self.env.timing.dt
        self.last = None

    def step(self, action):
        if self.last is not None:
            loop_time = time.perf_counter() - self.last
            if loop_time > self.dt:
                logging.warning(f"loop_time = {np.round(1e3 * loop_time, 2)} ms > dt")
            while time.perf_counter() - self.last < self.dt:
                pass
            self.env.timing.dt_actual = time.perf_counter() - self.last
        self.last = time.perf_counter()
        
        return self.env.step(action)

    def reset(self):
        self.last = None
        self.env.timing.dt_actual = 0
        
        return self.env.reset()

class HistoryWrapper(gym.Wrapper):
    """
    Track history of observations for given amount of steps
    Initial steps are zero-filled
    Modified version of code obtained from: https://github.com/Armandpl/furuta/tree/master
    """
    def __init__(self, env, steps, use_continuity_cost, alpha):
        super(HistoryWrapper, self).__init__(env)
        self.steps = steps
        self.alpha = alpha
        self.use_continuity_cost = use_continuity_cost

        # concat obs with action
        self.step_low = np.concatenate([self.observation_space.low,
                                        self.action_space.low])
        self.step_high = np.concatenate([self.observation_space.high,
                                         self.action_space.high])

        # stack for each step
        obs_low = np.tile(self.step_low, (self.steps, 1))
        obs_high = np.tile(self.step_high, (self.steps, 1))

        self.observation_space = Box(low=obs_low, high=obs_high)

        self.history = self._make_history()

    def _make_history(self):
        return [np.zeros_like(self.step_low) for _ in range(self.steps)]

    def _continuity_cost(self, obs):
        action = obs[-1][-1]
        last_action = obs[-2][-1]
        continuity_cost = self.alpha * np.power((action-last_action), 2).sum()

        return continuity_cost

    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.history.pop(0)

        obs = np.concatenate([obs, action])
        self.history.append(obs)
        obs = np.array(self.history)

        if self.use_continuity_cost:
            reward -= self._continuity_cost(obs)

        return obs, reward, done, info

    def reset(self):
        self.history = self._make_history()
        self.history.pop(0)
        obs = np.concatenate([self.env.reset(),
                              np.zeros_like(self.env.action_space.low)])
        self.history.append(obs)
        
        return np.array(self.history)
