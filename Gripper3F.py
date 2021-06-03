import random as rand
import time

import numpy
import pybullet as pb
import pybullet_data
# from stable_baselines import DDPG
# from stable_baselines.common.policies import CnnPolicy

from jaw_gripper.envs import JawGripperEnv
from jaw_gripper.robots.UR3FRobot import UR3FRobot

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


def main():
    pb_client = pb.connect(pb.GUI)
    pybullet_data_path = pybullet_data.getDataPath()
    pb.setAdditionalSearchPath(pybullet_data_path)
    plane = pb.loadURDF("plane.urdf")
    tray = pb.loadURDF('jaw_gripper/resources/tote/toteA_large.urdf', useFixedBase=True, basePosition=[-0.8, 0, 0])
    random_x = -round(rand.uniform(0.1, 0.6), 10)
    random_obj = pb.loadURDF('jaw_gripper/resources/models/ycb/021_bleach_cleanser.urdf',
                             basePosition=[random_x - 0.3, 0, 0.2])
    robot = UR3FRobot(pb_client)
    _, robot_id = robot.get_ids()
    pb.setRealTimeSimulation(1)
    pb.setGravity(0, 0, -10)
    joint_positions = pb.calculateInverseKinematics2(bodyUniqueId=robot_id,
                                                     endEffectorLinkIndices=robot.end_effectors_indexes,
                                                     targetPositions=[[0.6, 0.5, 1.1], [0.6, 0.5, 1.1], [0.4, 0.5, 1.1],
                                                                      [0.4, 0.5, 0.4], [0.5, 0.5, 1.1],
                                                                      [0.5, 0.5, 1.1]])
    # pb.setJointMotorControlArray(bodyIndex=robot_id, jointIndices=range(15), controlMode=pb.POSITION_CONTROL,
    #                              targetPositions=joint_positions, targetVelocities=[0] * 15, forces=[100.] * 15,
    #                              positionGains=[0.3] * 15, velocityGains=[1] * 15)
    robot.end_effectors_distances_from_object(random_obj)
    print(numpy.array(range(15)))
    while pb.isConnected(pb_client):
        time.sleep(1. / 240.)


main()
