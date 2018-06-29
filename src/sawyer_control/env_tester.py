#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
import time

env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='joint_space_impd', update_hz=20)
time_total = 0
env.set_safety_box(pos_high = np.array([0.72, 0.9, 0.8]), pos_low = np.array([0.421, -0.8, 0.26]))
def pos(): 
	return env._end_effector_pose()[:3]
total = np.zeros(3)
for i in range(10):

	delta = np.random.uniform(-0.1, 0.1, 3)
	curtime = time.time()
	cur = pos()
	env._joint_act(delta)
	time_total += time.time() - curtime
	change = cur - pos()
	total += np.abs(change)
	print(change)

print('avg', total/10)
print('avg_freq', 10/time_total)
