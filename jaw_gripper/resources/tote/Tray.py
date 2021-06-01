import pybullet as pb


class Tray:

    def __init__(self, client, model_path='jaw_gripper/resources/tote/toteA_large.urdf', base_pos=None):
        if base_pos is None:
            base_pos = [-0.8, 0, 0]

        self.client = client
        self.base_pos = base_pos
        self.tray = pb.loadURDF(model_path, basePosition=self.base_pos,physicsClientId=self.client,useFixedBase=True)

    def get_ids(self):
        return self.client, self.tray
