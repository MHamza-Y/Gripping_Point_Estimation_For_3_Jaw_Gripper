import gym
import jaw_gripper
import pybullet as pb
from jaw_gripper.envs import JawGripperEnv
import matplotlib.pyplot as plt

env = JawGripperEnv(renders=True)
# for _ in range(10000):
while pb.isConnected(physicsClientId=env.client):

    # env.step(env.action_space.sample())  # take a random action
    env.step({})
env.close()
