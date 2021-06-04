from gym.spaces import Box
from gym.utils.seeding import np_random as gym_np_random
import gym
import numpy as np
import pybullet as pb
import pybullet_data
import os
import time
from jaw_gripper.resources.models.TargetObject import TargetObject
from jaw_gripper.resources.tote.Tray import Tray
from jaw_gripper.robots.UR3FRobot import UR3FRobot


class JawGripperEnv(gym.Env):

    def __init__(self, width=960, height=720, timeStep=(1. / 240.), renders=False,
                 target_object_models_folder='jaw_gripper/resources/models/ycb'):
        self._timeStep = timeStep
        self._width = width
        self._height = height
        self._observation = []
        self._renders = renders
        self._target_object_models_folder = target_object_models_folder
        self._seed = self.seed()
        self.pb_connection_type = pb.GUI if self._renders else pb.DIRECT
        self.client = pb.connect(self.pb_connection_type)
        pybullet_data_path = pybullet_data.getDataPath()
        pb.setAdditionalSearchPath(pybullet_data_path)
        self.load_world()
        action_space_lower_limits, action_space_upper_limits = self.robot.get_action_space_limits()

        self.observation_space = Box(
            low=np.zeros(shape=[self._height, self._width, 4]),
            high=np.full(shape=[self._height, self._width, 4], fill_value=255)
        )
        self.action_space = Box(
            low=np.array(action_space_lower_limits),
            high=np.array(action_space_upper_limits)
        )

    def load_world(self):
        self.step_number = 0
        pb.resetSimulation(self.client)
        self.plane = pb.loadURDF("plane.urdf")
        self.robot = UR3FRobot(self.client)
        _, self.tray = Tray(self.client).get_ids()
        _, self.target_object_id = TargetObject(model_path=self.get_random_target_object_path(),
                                                client=self.client).get_ids()
        pb.setGravity(0, 0, -9.8)
        self.done = False
        self.setup_camera()

    def step(self, action):
        pb.stepSimulation(self.client)
        self.update_observation()
        if self._renders:
            time.sleep(self._timeStep)

        self.step_number += 1

        done = self._termination()
        reward = self._reward()
        return np.array(self._observation), reward, done, {}

    def reset(self):
        self.load_world()

    def render(self, mode='human', close=False):
        if mode != "rgb_array":
            return np.array([])
        return self.update_observation()

    def seed(self, seed=None):
        self.np_random, seed = gym_np_random(seed)
        return [seed]

    def setup_camera(self, camera_target_position=None, cam_dist=1.1, cam_yaw=0, cam_pitch=-40, fov=60, near_val=0.1,
                     far_val=3):

        if camera_target_position is None:
            camera_target_position = [-1, -0.2, 0.6]

        self.view_matrix = pb.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=camera_target_position,
                                                                distance=cam_dist,
                                                                yaw=cam_yaw,
                                                                pitch=cam_pitch,
                                                                roll=0,
                                                                upAxisIndex=2)
        self.proj_matrix = pb.computeProjectionMatrixFOV(fov=fov,
                                                         aspect=float(self._width) / self._height,
                                                         nearVal=near_val,
                                                         farVal=far_val)

    def update_observation(self):
        img_arr = pb.getCameraImage(width=self._width,
                                    height=self._height,
                                    viewMatrix=self.view_matrix,
                                    projectionMatrix=self.proj_matrix,
                                    renderer=pb.ER_BULLET_HARDWARE_OPENGL)
        rgb = img_arr[2]
        np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
        self._observation = np_img_arr
        return self._observation

    def get_random_target_object_path(self):
        filenames = sorted(
            [os.fsdecode(file) for file in os.listdir(self._target_object_models_folder) if
             os.fsdecode(file).endswith(".urdf")])
        chosen_model_path = self._target_object_models_folder + '/' + self.np_random.choice(filenames)
        return chosen_model_path

    def _termination(self):
        return {}

    def _reward(self):
        return 1-(self.robot.end_effector_distance_from_object(self.target_object_id)/4)
