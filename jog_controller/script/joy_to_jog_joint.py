#!/usr/bin/env python

# Take joystick cmds. Republish them as JogJoint

# RB: +x, LB: -x
# L on L dpad: +x
# Up on L dpad: +z

# R on R stick: +Rx
# Up on R stick: +Ry
# B: +Rz, A: -Rz

import rospy
from jog_msgs.msg import JogJoint
from sensor_msgs.msg import Joy

class joy_to_jog_joint:

    def __init__(self):
        self.pub = rospy.Publisher('jog_joint', JogJoint, queue_size=1)

    # Convert to JogJoint and republish
    def callback(self, joy):

        msg = JogJoint()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        # msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
	msg.joint_names = rospy.get_param("controller_joint_names","['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']")
        # These buttons are binary
        msg.deltas = [0]*6
        msg.deltas[0] = 0.05*(joy.axes[3])
        msg.deltas[1] = 0.05*(joy.axes[2])
        msg.deltas[2] = 0.05*(joy.axes[1])
        msg.deltas[3] = 0.05*(joy.axes[0])
        msg.deltas[4] = 0.05*(joy.buttons[5] - joy.buttons[4])
        msg.deltas[5] = 0.05*(joy.buttons[7] - joy.buttons[6])

        self.pub.publish(msg)
        
    def republish(self):
        rospy.Subscriber("joy", Joy, self.callback)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joy_to_jog_joint', anonymous=True)
    republisher = joy_to_jog_joint()
    republisher.republish()
