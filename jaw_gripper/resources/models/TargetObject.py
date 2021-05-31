import pybullet as pb


class TargetObject:

    def __init__(self, model_path, client, base_pos=None):
        if base_pos is None:
            base_pos = [-0.5, 0, 0.2]
        self.model_path = model_path
        self.base_pos = base_pos
        self.client = client
        self._target_object = pb.loadURDF(fileName=self.model_path, basePosition=self.base_pos,
                                          physicsClientId=self.client)

    def get_ids(self):
        return self.client, self._target_object
