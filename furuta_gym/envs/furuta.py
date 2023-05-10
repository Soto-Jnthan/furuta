import logging
import wandb
import numpy as np
import gym
from gym.spaces import Box
from .common import Timing, C2D_Filter
from .serial_uc import Serial_uC

PHI = 0
THETA = 1
PHI_DOT = 2
THETA_DOT = 3

PH_MAX = np.pi # rad
TH_MAX = 2 * np.pi  # rad
PH_DOT_MAX = 300 # rad/s 
TH_DOT_MAX = 400 # rad/s

TH_ACT_LIM = 0.8 * np.pi # rad (must be less than np.pi)
MTR_MAX_STEP_RATE = 4000 # microstep/s

MTR_STEP_ANGLE = 1.8 # degrees
MTR_MODE_DVSR = 8 # Power of 2 for the microstep mode
RADS_PER_STEPS = np.radians(MTR_STEP_ANGLE / MTR_MODE_DVSR)

class Furuta(gym.Env):
    def __init__(self, port, baudrate, fs):
        # Initialize serial communication between the Gym environment and the uC
        self.uC = Serial_uC(port, baudrate)
    
        # Limits
        state_max = np.array([PH_MAX, TH_MAX, PH_DOT_MAX, TH_DOT_MAX], dtype=np.float32)
        obs_max = np.array([1.0, 1.0, 1.0, 1.0, PH_DOT_MAX, TH_DOT_MAX], dtype=np.float32)
        act_max = np.array([1.0], dtype=np.float32)
        
        # Spaces
        self.state_space = Box(low=-state_max, high=state_max, dtype=np.float32)
        self.observation_space = Box(low=-obs_max, high=obs_max, dtype=np.float32)
        self.action_space = Box(low=-act_max, high=act_max, dtype=np.float32)
        
        self.lim_act = ActionLimiter(self.state_space, self.action_space, TH_ACT_LIM)
        self.map_act = lambda a: a * MTR_MAX_STEP_RATE 
        
        self.timing = Timing(fs)
        self.vel_filt = C2D_Filter(2, dt=self.timing.dt)
        
        self.prev_state = None
        self.read_state()
        
    def step(self, a):
        assert a is not None, "Action should be not None"
        assert isinstance(a, np.ndarray), "The action should be a ndarray"
        assert np.all(not np.isnan(a)), "Action NaN is not a valid action"
        assert a.ndim == 1, "The action = {a} must be 1d but the input is {a.ndim}d"
        
        a_cmd = a if self.prev_state is None else self.lim_act(self.prev_state, a)
            
        self.uC.write_speed(self.map_act(a_cmd[0]))
        
        state = self.read_state()
        
        reward = self.rwd(state, a)
        
        done = not self.state_space.contains(state) or self.uC.ep_done
        
        if self.uC.ep_done:
            logging.info("Episode terminated by uC")
        
        obs = self.get_obs(state)
            
        info = {"pendulum_angle": float(state[PHI]),
                "motor_angle": float(state[THETA]),
                "pendulum_angle_velocity": float(state[PHI_DOT]),
                "motor_angle_velocity": float(state[THETA_DOT]),
                "reward": float(reward),
                "done": bool(done),
                "action": float(a),
                "corrected_action": float(a_cmd),
                "loop_time": float(self.timing.dt_actual)}  # policy output
        
        if logging.root.level == logging.DEBUG:
            wandb.log({**info})
    
       # Return the observation, reward, done flag and info dict.
        return obs, reward, done, info   
        
    def read_state(self):
        # Return the state of the environment
        phi, theta = self.uC.read_state()
        phi *= np.pi / 2048
        theta *= RADS_PER_STEPS 
        pos = np.array([phi, theta]) # rad
        if self.prev_state is not None:
            dphi = phi - self.prev_state[PHI]
            if dphi < -np.pi: # adjust angular difference if needed
                dphi += 2 * np.pi
            elif dphi > np.pi:
                dphi -= 2 * np.pi
            vel = self.vel_filt(np.array([self.rel_phi_0 + dphi, theta])) # rad/s
            vel *= self.timing.dt / self.timing.dt_actual if self.timing.dt_actual != 0 else 1
            self.rel_phi_0 += dphi
        else:
            vel = np.array([0, 0]) # rad/s
            self.rel_phi_0 = 0 # initial relative phi displacement (rad)
            self.vel_filt.set_initial_state(np.array([self.rel_phi_0, theta]))
        state = np.concatenate([pos, vel]).astype(np.float32)
        self.prev_state = state
        return state
    
    def rwd(self, state, a):
        # Return the reward value for given timestep
        cost = state[PHI] ** 2 
        cost += 2e-3 * state[PHI_DOT] ** 2 
        cost += 1e-1 * state[THETA] ** 2
        cost += 2e-2 * state[THETA_DOT] ** 2
        cost += 3e-2 * a[0] ** 2
        return np.float32(-cost)
        
    def get_obs(self, state):
        # Return the current observation
        return np.array([np.sin(state[PHI]),
                         np.cos(state[PHI]),
                         np.sin(state[THETA]),
                         np.cos(state[THETA]),
                         state[PHI_DOT],
                         state[THETA_DOT]], dtype=np.float32)
    
    def reset(self):
        # Return state at reset position
        logging.info("Reset env...")
        while True:
            reset_done = self.uC.reset()
            if reset_done:
                break
        logging.info("Reset done")
        return self.get_obs(self.read_state())
    
    def render(self):
        # Not Implemented
        pass
        
    def close(self):
        self.uC.close()
        
class ActionLimiter:
    """
    ActionLimiter class for avoiding joint violations
    Modified version of the code obtained from: https://git.ias.informatik.tu-darmstadt.de/quanser/clients/-/tree/master/quanser_robots/qube
    """
    def __init__(self, state_space, action_space, th_lim_min):
        self._th_lim_min = th_lim_min
        self._th_lim_max = (state_space.high[THETA] + self._th_lim_min) / 2.0
        self._th_lim_stiffness = action_space.high[0] / (self._th_lim_max - self._th_lim_min)
        self._clip = lambda a: np.clip(a, action_space.low, action_space.high)
        self._relu = lambda x: x * (x > 0.0)

    def _joint_lim_violation_force(self, x):
        th = x[THETA]
        thd = x[THETA_DOT]
        up = self._relu(th-self._th_lim_max) - self._relu(th-self._th_lim_min)
        dn = -self._relu(-th-self._th_lim_max) + self._relu(-th-self._th_lim_min)
        if (th > self._th_lim_min and thd > 0.0 or th < -self._th_lim_min and thd < 0.0):
            force = self._th_lim_stiffness * (up + dn)
        else:
            force = 0.0
        return force

    def __call__(self, x, a):
        force = self._joint_lim_violation_force(x)
        return self._clip(force if force else a)