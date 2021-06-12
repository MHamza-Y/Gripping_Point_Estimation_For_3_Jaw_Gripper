from stable_baselines3.common.policies import ActorCriticCnnPolicy


class CustomActorCriticCnnPolicy(ActorCriticCnnPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomActorCriticCnnPolicy, self).__init__(*args, **kwargs,
                                                         net_arch=[1024, 512, 256, 128,
                                                                   dict(pi=[256, 128, 64], vf=[256, 128, 64])])
