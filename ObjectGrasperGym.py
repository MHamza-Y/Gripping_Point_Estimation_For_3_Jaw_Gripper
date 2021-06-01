import gym
import jaw_gripper
import pybullet as pb
from jaw_gripper.envs import JawGripperEnv
import matplotlib.pyplot as plt

env = JawGripperEnv(renders=False)
env.reset()
# for _ in range(10000):
while pb.isConnected(physicsClientId=env.client):
    frame = env.render('rgb_array')
    plt.plot(frame[:,:,0])
    # env.step(env.action_space.sample())  # take a random action
    env.step({})
env.close()
