import gym 
import numpy as np
import time
from filters import MMFilter
from Serial_uC import Serial_uC

class FurutaPendulum(gym.Env):
    def __init__(self, max_ep_steps=1000, step_time=0.015):
        
        self.KV42_MOVILITY_ANGLE = 360*1.5 # degrees
        self.KV42_STEP_ANGLE = 1.8 # degrees
        self.KV42_MODE_DVSR = 4 # Power of 2 for the microstep mode
        self.KV42_MAX_STEP_RATE = 2000 # step*s^-1
        
        self.MAX_EPISODE_STEPS = max_ep_steps                  
        self.STEP_TIME = step_time # s
        self.STEPS_TO_RAD_FCTR = (np.pi * self.KV42_STEP_ANGLE) / (180 * self.KV42_MODE_DVSR)
        
        self.PHI_COST_COEF = 1e0
        self.PHI_DOT_COST_COEF = 1e-2
        self.OUT_OF_BOUNDARY_COST = 1e3
        
        # Initialize serial communication between the RL model and the uC
        self.uC = Serial_uC('/dev/ttyACM0', 1000000)
        self.uC_ep_done = False
        
        # State Variables
        self.phi = 0
        self.theta = 0
        self.phi_dot = 0
        self.theta_dot = 0
    
        # Limits
        self.th_max = np.radians(self.KV42_MOVILITY_ANGLE)/2 #rad
        self.ph_dot_max = 50 #rad/s [Measured while testing the encoder]
        self.th_dot_max = self.STEPS_TO_RAD_FCTR * self.KV42_MAX_STEP_RATE #rad/s
        
        obs_max = np.array([1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32)
        act_max = np.array([1.0], dtype=np.float32)
    
        # Spaces
        self.observation_space = gym.spaces.Box(low=-obs_max, high=obs_max, dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-act_max, high=act_max, dtype=np.float32)
        
        self.episode_steps = 0
        
        self.phi_0 = None
        self.t_0 = None
        self.vel_filt = MMFilter(3)
        
    def step(self, action):
        """ 
        action_resized = np.sign(action) * self.KV42_MAX_STEP_RATE * np.log(self.KV42_MAX_STEP_RATE * np.abs(action) + 1) / np.log(self.KV42_MAX_STEP_RATE+1)
        """
        action_resized = action* self.KV42_MAX_STEP_RATE 
        self.uC.write_speed(action_resized)
        
        self.update_state()
        
        self.episode_steps += 1 
         
        """  
        reward = (-1 + np.cos(self.phi, dtype=np.float32)) / 2
        """  
        cost = self.PHI_COST_COEF * self.phi ** 2
        cost += self.PHI_DOT_COST_COEF * self.phi_dot ** 2
        if np.abs(self.theta) >= 0.95 * self.th_max:
            cost += self.OUT_OF_BOUNDARY_COST
        reward = float(-cost)
        
        # Check if the episode is done due to truncation
        if self.episode_steps >= self.MAX_EPISODE_STEPS:
            truncated = True
        else:
            truncated = False  
        
        if self.uC_ep_done or truncated:
            done = True
        else:
            done = False
            
        info = {"pendulum_angle": float(self.phi),
                "motor_angle": float(self.theta),
                "pendulum_angle_velocity": float(self.phi_dot),
                "motor_angle_velocity": float(self.theta_dot),
                "reward": float(reward),
                "done": bool(done),
                "TimeLimit.truncated": bool(truncated),
                "action": float(action)}  # policy output
    
       # Return the observation, reward, flags and any additional info
        return self.get_obs(), reward, done, info   
        
    def update_state(self):
        uC_data = self.uC.read_state()
        t = time.perf_counter()
        self.phi = uC_data[0] * np.pi / 2048 #rad
        #########################################################################################
        if self.t_0 is not None:
            dphi = self.phi - self.phi_0
            if dphi < -np.pi:
                dphi += 2 * np.pi
            elif dphi > np.pi:
                dphi -= 2 * np.pi
            self.phi_dot = self.vel_filt(dphi / (t - self.t_0))
        else:
            self.vel_filt.reset()
            self.phi_dot = 0
        self.phi_0 = self.phi
        self.t_0 = t
        #########################################################################################
        self.theta = self.STEPS_TO_RAD_FCTR * uC_data[1] #rad
        self.theta_dot = self.STEPS_TO_RAD_FCTR * uC_data[2] #rad/s
        self.uC_ep_done = bool(uC_data[3]) #ul.
        
    def reset(self):
        # Return state at reset position
        self.episode_steps = 0
        self.uC.reset()
        self.t_0 = None
        self.update_state()
        return self.get_obs()
        
    def get_obs(self):
        # Return the current observation
        return np.array([np.cos(self.phi),
                         np.sin(self.phi),
                         np.clip(self.theta/self.th_max, -1, 1), 
                         np.clip(self.phi_dot/self.ph_dot_max, -1, 1),
                         np.clip(self.theta_dot/self.th_dot_max, -1, 1)], dtype=np.float32)
    
    def render(self):
        # Not Implemented
        pass
        
    def close(self):
        self.uC.close()