#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
import time
#reset_pos = [0.42050388, 0.04335139, 0.23998401]
[ 0.45389536, -0.29943681,  0.1622716 ]
[0.81412411, 0.20957713, 0.20857288]
[0.6036064 , 0.02196496, 0.38539356]
env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='position', update_hz=20)
time_total = 0
env.set_safety_box(pos_high = np.array([1, 1,1]), pos_low = np.array([0, -1, 0.26]))
def pos(): 
	return env._end_effector_pose()[:3]
# total = np.zeros(3)
# for i in range(10):
#
# 	delta = np.random.uniform(-0.05, 0.05, 3)
# 	curtime = time.time()
# 	cur = pos()
# 	env._joint_act(delta)
# 	time_total += time.time() - curtime
# 	change = cur - pos()
# 	total += np.abs(change)
#
# print('avg', total/10)
# print('avg_freq', 10/time_total)
