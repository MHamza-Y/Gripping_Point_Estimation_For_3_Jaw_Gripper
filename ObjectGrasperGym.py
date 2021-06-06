import gym
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common import results_plotter
import jaw_gripper
from jaw_gripper.envs import JawGripperEnv
from stable_baselines3 import DDPG, PPO
from stable_baselines3.ppo import policies
import numpy as np
import os

import pybullet as pb

from save_on_best_result_callback import SaveOnBestTrainingRewardCallback

log_dir = "traning_ws/"
# os.makedirs(log_dir, exist_ok=True)
env = JawGripperEnv(renders=True)
env = Monitor(env, log_dir)

n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
# model = DDPG(policy=policies.CnnPolicy, env=env, action_noise=action_noise, verbose=10, buffer_size=20000,batch_size=40)
model = PPO(policy=policies.ActorCriticCnnPolicy, env=env, verbose=10, batch_size=150, n_steps=300)

callback = SaveOnBestTrainingRewardCallback(check_freq=1000, log_dir=log_dir)
model.learn(
    total_timesteps=100000, callback=callback)
model.save('final_model/jaw_gripper_model')
results_plotter.plot_results([log_dir], 1000, results_plotter.X_TIMESTEPS, "3 Jaw Gripper")
obs = env.reset()
while pb.isConnected(physicsClientId=env.client):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
env.close()
