import math

import pybullet as pb
import numpy


# {'world': -1, 'base_link': 0, 'shoulder_link': 1, 'upper_arm_link': 2, 'forearm_link': 3, 'wrist_1_link': 4, 'wrist_2_link': 5, 'wrist_3_link': 6, 'ee_link': 7, 'H1_base_attach': 8, 'H1_wrist_attach': 9, 'H1_base_link': 10, 'H1_insert_F1': 11, 'H1_F1_palm_link': 12, 'H1_F1_base_link': 13, 'H1_F1_link_1': 14, 'H1_F1_link_2': 15, 'H1_F1_tip': 16, 'H1_F1_optoforce': 17, 'H1_insert_F2': 18, 'H1_F2_palm_link': 19, 'H1_F2_base_link': 20, 'H1_F2_link_1': 21, 'H1_F2_link_2': 22, 'H1_F2_tip': 23, 'H1_F2_optoforce': 24, 'H1_insert_F3': 25, 'H1_F3_palm_link': 26, 'H1_F3_base_link': 27, 'H1_F3_link_1': 28, 'H1_F3_link_2': 29, 'H1_F3_tip': 30, 'H1_F3_optoforce': 31, 'tool0': 32, 'base': 33}

class UR3FRobot:

    def __init__(self, client, model_path='jaw_gripper/robots/fh_desc/model.urdf'):
        self.client = client
        self.model_path = model_path
        self.gripper_robot = pb.loadURDF(self.model_path, useFixedBase=True,
                                         basePosition=[0, 0.4, 0.05], physicsClientId=self.client)
        pb.changeDynamics(self.gripper_robot, linkIndex=3, jointLowerLimit=-2.6, jointUpperLimit=2.6,
                          physicsClientId=self.client)
        self.num_joints = pb.getNumJoints(self.gripper_robot, physicsClientId=self.client)
        self.joints_map = self.get_joints_map()
        print(self.joints_map)
        # gripper joints names, indices and their limits
        self.gripper_joints = ['H1_F1J1', 'H1_F1J2', 'H1_F1J3', 'H1_F2J1', 'H1_F2J2', 'H1_F2J3', 'H1_F3J1', 'H1_F3J2',
                               'H1_F3J3']
        self.gripper_joints_force = 3
        self.gripper_joints_indices = list(map((lambda name: self.joints_map[name]), self.gripper_joints))
        self.gripper_joints_upper_limits = [0.785398163397, 1.0471975512, 1.57079632679, 0.785398163397, 1.0471975512,
                                            1.57079632679, 0.785398163397, 1.0471975512, 1.57079632679]
        self.gripper_joints_lower_limits = [-0.785398163397, -1.3962634016, -1.0471975512, -0.785398163397,
                                            -1.3962634016, -1.0471975512, -0.785398163397, -1.3962634016, -1.0471975512]

        # moveable joints names and indices
        self.motor_names, self.motor_indices = self.get_motor_indices()
        self.motor_indices_initial_value = [2.359576893457236, -2.702367198655559, 2.423464060533199,
                                            -1.1461043160631084, 0.6939171645958004, 0.7216500100305062,
                                            -0.7853947587201159, -1.3962850169782266, -0.7434535745496392,
                                            -0.025677142976759444, 0.14071720513226824, 1.211964111026222,
                                            0.004917451705394786, -1.2247367331012684, -1.0471975512000005]
        for i, joint_index in enumerate(self.motor_indices):
            pb.resetJointState(self.gripper_robot, joint_index, self.motor_indices_initial_value[i],
                               physicsClientId=self.client)

        self.__load_link_name_to_index()
        print(self._link_name_to_index)
        # end effector joint name and index
        self.end_effector_link_name = 'H1_base_link'
        self.end_effector_link_index = self.get_link_index_for(self.end_effector_link_name)

        self.finger_1_links = ['H1_F1_link_1', 'H1_F1_link_2']
        self.finger_1_links_indices = list(map((lambda name: self._link_name_to_index[name]), self.finger_1_links))
        self.finger_2_links = ['H1_F2_link_1', 'H1_F2_link_2']
        self.finger_2_links_indices = list(map((lambda name: self._link_name_to_index[name]), self.finger_2_links))
        self.finger_3_links = ['H1_F3_link_1', 'H1_F3_link_2']
        self.finger_3_links_indices = list(map((lambda name: self._link_name_to_index[name]), self.finger_3_links))

        self.wrist_start_link_index = 8

        self.total_num_of_joints = len(self.motor_indices)
        self.target_velocities = [0] * self.total_num_of_joints
        self.max_forces = [300.] * self.total_num_of_joints
        self.position_gains = [0.3] * self.total_num_of_joints
        self.velocity_gains = [1] * self.total_num_of_joints

    def get_ids(self):
        return self.client, self.gripper_robot

    def get_action_space_limits(self):
        lower_limits = [-0.1, -0.1, -0.1, 0, 0, 0] + self.gripper_joints_lower_limits
        upper_limits = [0.1, 0.1, 0.1, 2 * math.pi, 2 * math.pi, 2 * math.pi] + self.gripper_joints_upper_limits

        return lower_limits, upper_limits

    def apply_action(self, action):
        damping = 0.0005
        dx = action[0]*damping
        dy = action[1]*damping
        dz = action[2]*damping
        alpha = action[3]
        beta = action[4]
        gamma = action[5]
        fingers_joint_positions = action[6:]

        x, y, z = self.get_end_effector_position()
        x_target = x + dx
        y_target = y + dy
        z_target = z + dz
        target_pos = [x_target, y_target, z_target]
        target_orn = pb.getQuaternionFromEuler([alpha, beta, gamma])
        joint_positions = pb.calculateInverseKinematics(bodyUniqueId=self.gripper_robot,
                                                        endEffectorLinkIndex=self.end_effector_link_index,
                                                        targetPosition=target_pos, targetOrientation=target_orn,
                                                        physicsClientId=self.client)

        pb.setJointMotorControlArray(bodyIndex=self.gripper_robot, jointIndices=self.motor_indices,
                                     controlMode=pb.POSITION_CONTROL,
                                     targetPositions=joint_positions, targetVelocities=self.target_velocities,
                                     forces=self.max_forces,
                                     positionGains=self.position_gains, velocityGains=self.velocity_gains,
                                     physicsClientId=self.client)

        for i, joint_index in enumerate(self.gripper_joints_indices):
            pb.setJointMotorControl2(bodyIndex=self.gripper_robot, jointIndex=joint_index,
                                     controlMode=pb.POSITION_CONTROL, targetPosition=fingers_joint_positions[i],
                                     force=self.gripper_joints_force, physicsClientId=self.client)

    def get_end_effector_position(self):
        return numpy.array(pb.getLinkState(self.gripper_robot, linkIndex=self.end_effector_link_index,
                                           physicsClientId=self.client)[0])

    def end_effector_distance_from_object(self, target_object_id):
        target_object_pos = numpy.array(
            pb.getBasePositionAndOrientation(bodyUniqueId=target_object_id, physicsClientId=self.client)[0],
            dtype=numpy.float32)
        return numpy.linalg.norm(self.get_end_effector_position() - target_object_pos)

    def __load_link_name_to_index(self):
        self._link_name_to_index = {pb.getBodyInfo(self.gripper_robot)[0].decode('UTF-8'): -1, }

        for _id in range(pb.getNumJoints(self.gripper_robot, physicsClientId=self.client)):
            _name = pb.getJointInfo(self.gripper_robot, _id, physicsClientId=self.client)[12].decode('UTF-8')
            self._link_name_to_index[_name] = _id

    def get_link_index_for(self, name):
        return self._link_name_to_index[name]

    def get_joints_map(self):
        joints_map = {}
        print(self.num_joints)
        for i in range(self.num_joints):
            joint_info = pb.getJointInfo(self.gripper_robot, i, self.client)
            join_name = str(joint_info[1]).replace("b'", "").replace("'", "")
            joints_map[join_name] = i

        return joints_map

    def get_motor_indices(self):
        motor_names = []
        motor_indices = []
        for i in range(self.num_joints):
            joint_info = pb.getJointInfo(self.gripper_robot, i, self.client)
            q_index = joint_info[3]
            if q_index > -1:
                motor_names.append(str(joint_info[1]))
                motor_indices.append(i)
        return motor_names, motor_indices

    def is_hand_self_coliding(self):
        self_collision = False
        for i in range(-1, self.wrist_start_link_index - 1):
            contact_points = pb.getContactPoints(bodyA=self.gripper_robot, bodyB=self.gripper_robot,
                                                 linkIndexA=i,
                                                 physicsClientId=self.client)
            if contact_points:
                self_collision = True

        return self_collision

    def fingers_in_contact_with(self, target_object_id):
        finger_1_touching = False
        for link_index in self.finger_1_links_indices:
            finger_1_contact_points = pb.getContactPoints(bodyA=self.gripper_robot, bodyB=target_object_id,
                                                          linkIndexA=link_index,
                                                          physicsClientId=self.client)
            if finger_1_contact_points:
                finger_1_touching = True

        finger_2_touching = False
        for link_index in self.finger_2_links_indices:
            finger_2_contact_points = pb.getContactPoints(bodyA=self.gripper_robot, bodyB=target_object_id,
                                                          linkIndexA=link_index,
                                                          physicsClientId=self.client)
            if finger_2_contact_points:
                finger_2_touching = True

        finger_3_touching = False
        for link_index in self.finger_3_links_indices:
            finger_3_contact_points = pb.getContactPoints(bodyA=self.gripper_robot, bodyB=target_object_id,
                                                          linkIndexA=link_index,
                                                          physicsClientId=self.client)
            if finger_3_contact_points:
                finger_3_touching = True

        return finger_1_touching and finger_2_touching and finger_3_touching

    def is_in_contact_with_object(self, target_object_id):
        in_contact = False
        contact_points = pb.getContactPoints(bodyA=self.gripper_robot, bodyB=target_object_id,
                                             physicsClientId=self.client)

        if contact_points:
            in_contact = True

        return in_contact
