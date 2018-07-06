#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='position', update_hz=20)

def pos():
	return env._end_effector_pose()
print(pos())
