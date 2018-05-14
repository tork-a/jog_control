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
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        # These buttons are binary
        msg.displacement = [0]*6
        msg.displacement[0] = 0.05*(-joy.buttons[4] + joy.buttons[5])
        msg.displacement[1] = 0.05*(joy.axes[0])
        msg.displacement[2] = 0.05*(joy.axes[1])
        msg.displacement[3] = 0.05*(joy.buttons[0] - joy.buttons[1])
        msg.displacement[4] = 0.05*(joy.buttons[2] - joy.buttons[3])
        msg.displacement[5] = 0.05*(joy.buttons[7] - joy.buttons[6])

        self.pub.publish(msg)
        
    def republish(self):
        rospy.Subscriber("joy", Joy, self.callback)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joy_to_jog_joint', anonymous=True)
    republisher = joy_to_jog_joint()
    republisher.republish()
