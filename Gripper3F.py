import random as rand
import time

import pybullet as pb
import pybullet_data
#from stable_baselines import DDPG
#from stable_baselines.common.policies import CnnPolicy

from jaw_gripper.envs import JawGripperEnv

RENDER_HEIGHT = 720
RENDER_WIDTH = 960
def main():

    #env = JawGripperEnv()
    #model = DDPG(CnnPolicy,env,verbose=1)
    pb_client = pb.connect(pb.GUI)
    pybullet_data_path = pybullet_data.getDataPath()
    pb.setAdditionalSearchPath(pybullet_data_path)
    plane = pb.loadURDF("plane.urdf")
    gripper_robot = pb.loadURDF('jaw_gripper/resources/fh_desc/model.urdf', useFixedBase=False, basePosition=[0, 0, 0.5])
    tray = pb.loadURDF('jaw_gripper/resources/tote/toteA_large.urdf', useFixedBase=True, basePosition = [-0.8, 0, 0])
    random_x = -round(rand.uniform(0.1, 0.6), 10)

    random_obj = pb.loadSDF('jaw_gripper/resources/models/ycb/001_chips_can/chips_can.sdf')
    random_obj_texture = pb.loadTexture('jaw_gripper/resources/models/ycb/001_chips_can/tsdf/textured.png')
    pb.changeVisualShape(random_obj[0],-1,textureUniqueId=random_obj_texture)
    pb.resetBasePositionAndOrientation(random_obj[0],[random_x - 0.3, 0, 0.2],pb.getQuaternionFromEuler([0,0,1]))

    #pb.setCollisionFilterGroupMask(tray,-1,0,0)
    #pb.setCollisionFilterPair(gripper_robot,tray,-1, -1, 1)

    _link_name_to_index = {pb.getBodyInfo(gripper_robot)[0].decode('UTF-8'): -1, }

    for _id in range(pb.getNumJoints(gripper_robot)):
        _name = pb.getJointInfo(gripper_robot, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id

    #pb.calculateInverseKinematics2(gripper_robot,_link_name_to_index['H1_F1_link_2'],[0,0,1])
    print(_link_name_to_index)
    pb.setRealTimeSimulation(1)
    pb.setGravity(0, 0, -10)
    cam_dist = 1.5
    cam_yaw = 0
    cam_pitch = -30
    while (pb.isConnected()):
        base_pos, orn = pb.getBasePositionAndOrientation(gripper_robot)
        view_matrix = pb.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[-1,-0.2,0.6],
                                                                distance=cam_dist,
                                                                yaw=cam_yaw,
                                                                pitch=cam_pitch,
                                                                roll=0,
                                                                upAxisIndex=2)
        proj_matrix = pb.computeProjectionMatrixFOV(fov=60,
                                                         aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                                                         nearVal=0.2,
                                                         farVal=4)
        (_, _, px, _, _) = pb.getCameraImage(width=RENDER_WIDTH,
                                                  height=RENDER_HEIGHT,
                                                  viewMatrix=view_matrix,
                                                  projectionMatrix=proj_matrix,
                                                  renderer=pb.ER_BULLET_HARDWARE_OPENGL)
        time.sleep(1. / 240.)
        pb.setGravity(0, 0, -10)

main()