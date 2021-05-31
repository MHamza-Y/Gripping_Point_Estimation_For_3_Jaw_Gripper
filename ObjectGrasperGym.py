import gym
import jaw_gripper
from jaw_gripper.envs import JawGripperEnv

env = JawGripperEnv(renders=True)
env.reset()
for _ in range(10000):
    #env.render()
    env.step(env.action_space.sample()) # take a random action
env.close()