import numpy
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

    def __init__(self, width=480, height=360, timeStep=(1. / 240.), renders=False,
                 target_object_models_folder='jaw_gripper/resources/models/ycb', max_steps=300):
        self.done = False
        self._timeStep = timeStep
        self.max_steps = max_steps
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

        self.observation_space = Box(0, 255, [self._height, self._width, 5], np.uint8)
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
        self.reward = 0
        self.setup_camera()

    def step(self, action):
        if self._renders:
            time.sleep(self._timeStep)
        self.robot.apply_action(action)
        pb.stepSimulation(self.client)
        self.step_number += 1
        print(action)
        done = self._termination()
        reward = self._reward()
        print(f'Reward: {reward}')
        print(f'Step: {self.step_number}')
        return self.update_observation(), reward, done, {}

    def reset(self):
        self.load_world()
        return self.update_observation()

    def render(self, mode='human', close=False):
        if close:
            self.close()
        # if mode != "rgb_array":
        #     return np.array([])
        return self.update_observation()

    def seed(self, seed=None):
        self.np_random, seed = gym_np_random(seed)
        return [seed]

    def setup_camera(self, camera_target_position=None, cam_dist=2, cam_yaw=0, cam_pitch=-90, fov=60, near_val=0.3,
                     far_val=10):
        self.near_val = near_val
        self.far_val = far_val
        if camera_target_position is None:
            camera_target_position = [0, 0, 0]

        self.view_matrix = pb.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=camera_target_position,
                                                                distance=cam_dist,
                                                                yaw=cam_yaw,
                                                                pitch=cam_pitch,
                                                                roll=-45,
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

        rgba = img_arr[2]
        np_img_arr = np.reshape(rgba, (self._height, self._width, 4))
        depth_buffer_opengl = np.reshape(img_arr[3], [self._height, self._width])
        depth_opengl = self.far_val * self.near_val / (self.far_val - (self.far_val - self.near_val) * depth_buffer_opengl)
        segmented_mask = np.reshape(img_arr[4], [self._height, self._width])
        rgb_with_depth_and_segmentation = np.dstack((np_img_arr[:, :, :3], depth_opengl, segmented_mask))
        self._observation = rgb_with_depth_and_segmentation
        print(np.shape(self._observation))
        return self._observation

    def get_random_target_object_path(self):
        filenames = sorted(
            [os.fsdecode(file) for file in os.listdir(self._target_object_models_folder) if
             os.fsdecode(file).endswith(".urdf")])
        chosen_model_path = self._target_object_models_folder + '/' + self.np_random.choice(filenames)
        return chosen_model_path

    def _termination(self):
        if self.done or self.step_number >= self.max_steps:
            return True
        return False

    def _reward(self):
        fingers_touching_object_reward = 0
        distance_reward = (1 - (self.robot.end_effector_distance_from_object(self.target_object_id) / 4)) * 10
        if self.robot.fingers_in_contact_with(self.target_object_id):
            end_effector_height = self.robot.get_end_effector_position()[2]
            fingers_touching_object_reward = 30 * end_effector_height
            if end_effector_height > 0.4:
                fingers_touching_object_reward += 1000
                self.done = True

        time_reward = 300/self.step_number
        self.reward = distance_reward + fingers_touching_object_reward + time_reward
        return self.reward

    def close(self):
        pb.disconnect(self.client)

    # def __del__(self):
    #     pb.disconnect(self.client)
