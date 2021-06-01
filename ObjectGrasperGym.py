import gym
import jaw_gripper
import pybullet as pb
from jaw_gripper.envs import JawGripperEnv

env = JawGripperEnv(renders=True)
env.reset()
# for _ in range(10000):
while pb.isConnected(physicsClientId=env.client):
    # env.render()
    #env.step(env.action_space.sample())  # take a random action
    env.step({})
env.close()
