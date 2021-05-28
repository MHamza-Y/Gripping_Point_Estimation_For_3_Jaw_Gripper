import pybullet
import pybullet as pb
import time
import pybullet_data


def main():
    pb_client = pb.connect(pb.GUI)
    pybullet_data_path = pybullet_data.getDataPath()
    pb.setAdditionalSearchPath(pybullet_data_path)
    #pb.loadURDF("plane.urdf")
    gripper_robot = pb.loadURDF('../resources/smart_grasping_sandbox/fh_desc/model.urdf', useFixedBase=1)
    pybullet.setRealTimeSimulation(1)
    time.sleep(10000)
    #pybullet.setAdditionalSearchPath()

main()