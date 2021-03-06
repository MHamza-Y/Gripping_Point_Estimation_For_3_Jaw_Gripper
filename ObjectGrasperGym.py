import gym
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.policies import ActorCriticCnnPolicy
from stable_baselines3.sac import CnnPolicy

from jaw_gripper.envs import JawGripperEnv
from stable_baselines3 import PPO, TD3, SAC
import numpy as np

import pybullet as pb

from save_on_best_result_callback import SaveOnBestTrainingRewardCallback

save_folder = 'training_ws/'
checkpoint_callback = CheckpointCallback(save_freq=10000, save_path=save_folder,
                                         name_prefix="rl_model")
env = JawGripperEnv(renders=True)
env = Monitor(env, save_folder)

best_model_save_callback = SaveOnBestTrainingRewardCallback(check_freq=2110, log_dir=save_folder)

# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = PPO(policy=ActorCriticCnnPolicy, env=env, verbose=2, tensorboard_log=save_folder, use_sde=True,
            sde_sample_freq=60, n_steps=9500,
            batch_size=64, seed=564664, clip_range_vf=0.2, learning_rate=0.0008)

# model = PPO.load('training_ws/rl_model_350000_steps.zip', env=env, use_sde=True,
#                  sde_sample_freq=30, n_steps=2100,
#                  batch_size=75, seed=56556, gamma=0.995, gae_lambda=0.97)
model.learn(
   total_timesteps=10000000, callback=[best_model_save_callback, checkpoint_callback])

obs = env.reset()
while pb.isConnected(physicsClientId=env.client):
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    if done:
        env.reset()
env.close()
