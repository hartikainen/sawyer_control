#!/usr/bin/python3
from sawyer_reaching import SawyerXYZReachingEnv
import numpy as np
import cv2
import time
env = SawyerXYZReachingEnv(reward='norm', safety_box=True, action_mode='joint_space_impd', update_hz=20)
time_total = 0
env.set_safety_box(pos_high = np.array([1, 0.4,0.5]), pos_low = np.array([0.2, -0.4, 0.1]))
reset_pos = np.array([0.3785973058969759, 0.010523406106589955, 0.17737810134900142])
def reset():
	env.joint_space_impd  = False
	env.thresh = False
	env._joint_act(pos() - reset_pos)
	env.joint_space_impd  = True
def pos():
	return env._end_effector_pose()[:3]
total_dist = np.zeros(3)
error = np.zeros(3)
n = 16

steps = [np.array([0.05, 0, 0]), np.array([-0.05, 0, 0]), np.array([0.0, 0.05, 0]), np.array([0.0, -0.05, 0])]
for i in range(n):
	# delta = np.random.uniform(-0.05, 0.05, 3)
	delta = steps[i // 4] # np.array([0.05, 0, 0])
	curtime = time.time()
	cur = pos()
	env._joint_act(delta)
	time_total += time.time() - curtime
	change = pos() - cur
	print('error', delta - change)
	error += np.abs(change - delta)
	total_dist += np.abs(change)

print('avg_error', error/n)
print('avg_speed', total_dist/time_total)
