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
from wandb.integration.sb3 import WandbCallback

def main(args=None):
    config = {
        "env_name": 'Furuta-v0',
        "port": 'COM3', 
        "baudrate": 1000000, 
        "fs": 100,
        "max_episode_timesteps": 1000,
        "total_timesteps": 280*1000
    }
    
    run = wandb.init(
        project="furuta",
        config=config,
        sync_tensorboard=True,
        save_code=True
    )
    
    log_path = os.path.join('runs', f'{run.id}')
    buffer_path = os.path.join(log_path, 'buffers')
    model_path = os.path.join(log_path, 'models')
    
    env = make_env(config)
    
    wandb.run.summary["state_max"] = env.state_space.high

    model = SAC(
                "MlpPolicy", DummyVecEnv([lambda: env]),
                verbose=2, seed=1, learning_starts=500,
                use_sde=True, use_sde_at_warmup=True,
                sde_sample_freq=64,
                train_freq=(1, 'episode'),
                gradient_steps=-1,
                tensorboard_log=log_path,
    )

    try:
        logging.info("Starting to train")
        model.learn(
            total_timesteps=config["total_timesteps"],
            callback=WandbCallback(verbose=2),
        )
    except:
        logging.info("Interupting training")

    model.save(model_path)
    model.save_replay_buffer(buffer_path)

    env.close()
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
    logging.basicConfig( format='%(levelname)s: %(message)s', level=logging_level)
    main()