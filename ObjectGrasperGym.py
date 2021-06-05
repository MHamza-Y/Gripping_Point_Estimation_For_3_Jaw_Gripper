import gym
from stable_baselines3.common.noise import NormalActionNoise

import jaw_gripper
from jaw_gripper.envs import JawGripperEnv
from stable_baselines3 import DDPG
from stable_baselines3.ddpg import policies
import numpy as np


import pybullet as pb

env = JawGripperEnv(renders=True)
# for _ in range(10000):
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
model = DDPG(policy=policies.CnnPolicy, env=env, action_noise=action_noise, verbose=1, buffer_size=10000).learn(
    total_timesteps=10000)
obs = env.reset()
# while pb.isConnected(physicsClientId=env.client):
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
env.close()
