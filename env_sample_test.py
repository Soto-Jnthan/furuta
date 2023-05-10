import logging
import gym
from gym.wrappers import TimeLimit
from stable_baselines3.common.monitor import Monitor
import furuta_gym  # noqa F420
from furuta_gym.envs.wrappers import HistoryWrapper, ControlFrequency

def main(args=None):
    config = {
        "env_name": 'Furuta-v0',
        "port": 'COM3', 
        "baudrate": 1000000, 
        "fs": 60,
        "max_episode_timesteps": 1000,
        "total_timesteps": 1000000
    }
    env = make_env(config)

    episodes = 10
    for episode in range(1, episodes+1):
        state = env.reset()
        done = False
        score = 0 

        while not done:
            action = env.action_space.sample()
            n_state, reward, done, info = env.step(action)
            score+=reward
        print('Episode:{} Score:{}'.format(episode, score))
    env.reset()
    env.close()
    
def make_env(config):
    env = gym.make(
        config["env_name"], 
        port=config["port"], 
        baudrate=config["baudrate"], 
        fs=config["fs"]
    )
    env = TimeLimit(env, config["max_episode_timesteps"])
    env = HistoryWrapper(env, 2, True, alpha=1e-1)
    env = Monitor(env)
    env = ControlFrequency(env)
    return env

if __name__ == "__main__":
    main()