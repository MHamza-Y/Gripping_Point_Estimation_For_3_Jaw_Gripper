from gym.envs.registration import register

register(
    id='JawGripperEnv-v0',
    entry_point='jaw_gripper.envs:SimpleDrivingEnv'
)
