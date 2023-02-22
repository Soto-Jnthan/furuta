import os
import time
import tempfile
os.environ['MPLCONFIGDIR'] = tempfile.mkdtemp()

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecFrameStack
from stable_baselines3.common.evaluation import evaluate_policy

from FurutaPendulum import FurutaPendulum
from wrappers import StepTimeCtrl, HistoryWrapper

MAX_STEP_TIME = 1/60 #s
env = HistoryWrapper(StepTimeCtrl(FurutaPendulum(1000, MAX_STEP_TIME)),alpha=1e0)

log_path = os.path.join('Training', 'Logs')
model = SAC("MlpPolicy", env, verbose=1, tensorboard_log=log_path, use_sde=True, sde_sample_freq=64)
for i in range(0,20):
    model.learn(total_timesteps=20000)
    model_path = os.path.join('Training', 'Models','SAC_20k'+ '_' + str(i))
    model.save(model_path)
    env.uC.reset()
    time.sleep(60 * 3)
    
env.close()
    