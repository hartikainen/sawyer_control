#!/usr/bin/env python
import rospy
import intera_interface as ii
from sawyer_control.srv import angle_action
from sawyer_control.srv import *
import numpy as np



#!/usr/bin/env python

# Copyright (c) 2013-2016, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Intera SDK Joint Torque Example: joint springs
"""


import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class JointController(object):

    def __init__(self,
                 limb,
                 rate = 1000.0,
                 ):

        # control parameters
        self._rate = rate # Hz

        # create our limb instance
        self._limb = limb

        # initialize parameters
        self.imp_ctrl_publisher = rospy.Publisher('/desired_joint_pos', JointState, queue_size=1)
        self.imp_ctrl_release_spring_pub = rospy.Publisher('/release_spring', Float32, queue_size=10)


    def move_with_impedance(self, des_joint_angles):
        """
        non-blocking
        """
        js = JointState()
        js.name = self._limb.joint_names()
        js.position = [des_joint_angles[n] for n in js.name]
        self.imp_ctrl_publisher.publish(js)


    def move_with_impedance_sec(self, cmd, duration=2.0):
        jointnames = self._limb.joint_names()
        prev_joint = [self._limb.joint_angle(j) for j in jointnames]
        new_joint = np.array([cmd[j] for j in jointnames])
        control_rate = rospy.Rate(self._rate)
        start_time = rospy.get_time()  # in seconds
        finish_time = start_time + duration  # in seconds

        while rospy.get_time() < finish_time:
            int_joints = prev_joint + (rospy.get_time()-start_time)/(finish_time-start_time)*(new_joint-prev_joint)
            # print int_joints
            cmd = dict(list(zip(self._limb.joint_names(), list(int_joints))))
            self.move_with_impedance(cmd)
            control_rate.sleep()

def execute_action(action_msg):
    action = action_msg.angles
    thresh =action_msg.thresh
    joint_space_impd = action_msg.joint_space_impd
    joint_names = arm.joint_names()
    joint_to_values = dict(zip(joint_names, action))
    if joint_space_impd:
        controller.move_with_impedance_sec(joint_to_values, duration=0.35)
    else:
        if thresh:
            t = 0.5
        else:
            t = 15

        arm.move_to_joint_positions(joint_to_values, timeout = t)
    return angle_actionResponse(True)

def angle_action_server():
    rospy.init_node('angle_action_server', anonymous=True)
    global arm
    global controller
    arm = ii.Limb('right')
    arm.set_joint_position_speed(0.3)
    controller = JointController(arm)
    s = rospy.Service('angle_action', angle_action, execute_action)
    rospy.spin()









if __name__ == '__main__':
    angle_action_server()

