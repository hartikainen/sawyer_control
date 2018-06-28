#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2


env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='joint_space_impd', update_hz=20)
env.set_safety_box(pos_high = np.array([0.72, 0.145, 0.45]), pos_low = np.array([0.421, 0.145, 0.26]))
def pos(): 
	return env._end_effector_pose()[:3]
total = np.zeros(3)
for i in range(10): 
	delta = np.random.uniform(-0.05, 0.05, 3)
	cur = pos()
	env._joint_act(delta)
	change = cur - pos()

	total += np.abs(change)
	print(change)
print('avg', total/10)