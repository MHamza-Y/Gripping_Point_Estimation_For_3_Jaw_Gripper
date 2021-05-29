import pybullet
import pybullet as pb
import time
import pybullet_data
import random as rand


def main():
    pb_client = pb.connect(pb.GUI)
    pybullet_data_path = pybullet_data.getDataPath()
    pb.setAdditionalSearchPath(pybullet_data_path)
    plane = pb.loadURDF("plane.urdf")
    gripper_robot = pb.loadURDF('jaw_gripper/resources/fh_desc/model.urdf', useFixedBase=False, basePosition=[0, 0, 0.25])
    tray = pb.loadURDF('jaw_gripper/resources/tote/toteA_large.urdf', useFixedBase=True, basePosition = [-1, 0, 0])
    random_x = -round(rand.uniform(0.1, 1.0), 10)
    cube = pb.loadURDF('jaw_gripper/resources/objects/cube.urdf', basePosition=[random_x - 0.7, 0, 1])

    pb.setCollisionFilterGroupMask(tray,-1,0,0)
    pb.setCollisionFilterPair(gripper_robot,tray,-1, -1, 1)

    _link_name_to_index = {pb.getBodyInfo(gripper_robot)[0].decode('UTF-8'): -1, }

    for _id in range(pb.getNumJoints(gripper_robot)):
        _name = pb.getJointInfo(gripper_robot, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id

    #pb.calculateInverseKinematics2(gripper_robot,_link_name_to_index['H1_F1_link_2'],[0,0,1])
    print(_link_name_to_index)
    pb.setRealTimeSimulation(1)
    pb.setGravity(0, 0, -10)
    while (pb.isConnected()):
        time.sleep(1. / 240.)
        pb.setGravity(0, 0, -10)

main()