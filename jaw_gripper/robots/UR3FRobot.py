import pybullet as pb
import numpy


# {'world': -1, 'base_link': 0, 'shoulder_link': 1, 'upper_arm_link': 2, 'forearm_link': 3, 'wrist_1_link': 4, 'wrist_2_link': 5, 'wrist_3_link': 6, 'ee_link': 7, 'H1_base_attach': 8, 'H1_wrist_attach': 9, 'H1_base_link': 10, 'H1_insert_F1': 11, 'H1_F1_palm_link': 12, 'H1_F1_base_link': 13, 'H1_F1_link_1': 14, 'H1_F1_link_2': 15, 'H1_F1_tip': 16, 'H1_F1_optoforce': 17, 'H1_insert_F2': 18, 'H1_F2_palm_link': 19, 'H1_F2_base_link': 20, 'H1_F2_link_1': 21, 'H1_F2_link_2': 22, 'H1_F2_tip': 23, 'H1_F2_optoforce': 24, 'H1_insert_F3': 25, 'H1_F3_palm_link': 26, 'H1_F3_base_link': 27, 'H1_F3_link_1': 28, 'H1_F3_link_2': 29, 'H1_F3_tip': 30, 'H1_F3_optoforce': 31, 'tool0': 32, 'base': 33}

class UR3FRobot:

    def __init__(self, client, model_path='jaw_gripper/robots/fh_desc/model.urdf'):
        self.client = client
        self.model_path = model_path
        self.gripper_robot = pb.loadURDF(self.model_path, useFixedBase=False,
                                         basePosition=[0, 0, 0.25], physicsClientId=self.client)
        self.end_effectors = ['H1_F1_link_1', 'H1_F1_link_2', 'H1_F1_tip', 'H1_F2_link_1', 'H1_F2_link_2', 'H1_F2_tip',
                              'H1_F3_link_1',
                              'H1_F3_link_2', 'H1_F3_tip']
        self.__load_link_name_to_index()
        self.end_effectors_indexes = self.get_link_index_for(self.end_effectors)
        self.total_num_of_joints = 15
        self.target_velocities = [0] * self.total_num_of_joints
        self.max_forces = [100.] * self.total_num_of_joints
        self.position_gains = [0.3] * self.total_num_of_joints
        self.velocity_gains = [1] * self.total_num_of_joints

    def get_ids(self):
        return self.client, self.gripper_robot

    def apply_action(self, action):
        pass

    def __load_link_name_to_index(self):
        self._link_name_to_index = {pb.getBodyInfo(self.gripper_robot)[0].decode('UTF-8'): -1, }

        for _id in range(pb.getNumJoints(self.gripper_robot)):
            _name = pb.getJointInfo(self.gripper_robot, _id)[12].decode('UTF-8')
            self._link_name_to_index[_name] = _id

    def get_link_index_for(self, names):
        return list(map((lambda name: self._link_name_to_index[name]), names))

    def end_effectors_distances_from_object(self, target_object_id):
        distances = []

        joint_states = pb.getLinkStates(bodyUniqueId=self.gripper_robot, linkIndices=self.end_effectors_indexes,
                                        physicsClientId=self.client)

        print(joint_states[0])

        return distances

    def act(self, change_in_positions_raw):
        change_in_positions = numpy.array(change_in_positions_raw).reshape(-1, 3)
        target_positions = numpy.array()
        for index, change_in_position in enumerate(change_in_positions):
            dx = change_in_position[0]
            dy = change_in_position[1]
            dz = change_in_position[2]
            state = pb.getLinkState(objectUniqueId=self.gripper_robot, linkIndex=index,
                                    physicsClientId=self.client)
            x, y, z = state[0]
            x_target = x + dx
            y_target = y + dy
            z_target = z + dz
            target_pos = [x_target, y_target, z_target]
            numpy.append(target_positions, [target_pos], axis=0)

        joint_positions = pb.calculateInverseKinematics2(bodyUniqueId=self.gripper_robot,
                                                         endEffectorLinkIndices=self.end_effectors_indexes,
                                                         targetPositions=target_positions)

        pb.setJointMotorControlArray(bodyIndex=self.gripper_robot, jointIndices=range(self.total_num_of_joints),
                                     controlMode=pb.POSITION_CONTROL,
                                     targetPositions=joint_positions, targetVelocities=self.target_velocities,
                                     forces=self.max_forces,
                                     positionGains=self.position_gains, velocityGains=self.velocity_gains)
