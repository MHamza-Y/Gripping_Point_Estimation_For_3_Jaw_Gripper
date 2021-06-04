import gym
import jaw_gripper
import pybullet as pb
from jaw_gripper.envs import JawGripperEnv
from stable_baselines import DDPG
from stable_baselines.ddpg import policies

env = JawGripperEnv(renders=True)
# for _ in range(10000):
model = DDPG(policy=policies.CnnPolicy,env=env,verbose=1).learn(total_timesteps=100000)

# while pb.isConnected(physicsClientId=env.client):
#
#     # env.step(env.action_space.sample())  # take a random action
#     env.step({})
env.close()
