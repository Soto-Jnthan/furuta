import logging
import os
import gym
import wandb
import furuta_gym 

from gym.wrappers import TimeLimit
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from furuta_gym.envs.wrappers import HistoryWrapper, ControlFrequency

class Robot():
    def __init__(self, args):
        self.model = SAC.load(args["model_dir"])
        self.env = make_env(args)

    def run_episode(self):
        print("Episode starts")
        total_reward = 0
        obs = self.env.reset()
        while True:
            action, _states = self.model.predict(obs, deterministic=True)
            obs, reward, done, info = self.env.step(action)
            total_reward += reward
            if done:
                obs = self.env.reset()
                break
        return total_reward

def main(args=None):
    config = {
        "env_name": 'Furuta-v0',
        "port": 'COM3', 
        "baudrate": 1000000, 
        "fs": 100,
        "max_episode_timesteps": 1000,
        "model_dir":'sac.zip'
    }
    
    run = wandb.init(
        project="furuta",
        config=config,
        sync_tensorboard=True,
        save_code=True,
        job_type="inference"
    )

    robot = Robot(run.config)
    
    wandb.run.summary["state_max"] = robot.env.state_space.high

    try:
        while True:
            rwd = robot.run_episode()
            print(f"Episode Reward: {rwd}")
            input("Press enter to run episode\n")
    except:
        robot.env.close()
        run.finish()

def make_env(config):
    env = gym.make(
        config["env_name"], 
        port=config["port"], 
        baudrate=config["baudrate"], 
        fs=config["fs"]
    )
    env = TimeLimit(env, config["max_episode_timesteps"])
    env = HistoryWrapper(env, 2, True, alpha=1)
    env = Monitor(env)
    env = ControlFrequency(env)
    return env

if __name__ == "__main__":
    logging_level = logging.DEBUG
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)
    main()