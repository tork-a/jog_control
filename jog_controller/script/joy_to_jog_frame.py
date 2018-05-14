#!/usr/bin/env python

# Take joystick cmds. Republish them as JogFrame

# RB: +x, LB: -x
# L on L dpad: +x
# Up on L dpad: +z

# R on R stick: +Rx
# Up on R stick: +Ry
# B: +Rz, A: -Rz

import rospy
from jog_msgs.msg import JogFrame
from sensor_msgs.msg import Joy

class joy_to_jog_frame:

    def __init__(self):
        self.pub = rospy.Publisher('jog_frame', JogFrame, queue_size=1)

    # Convert to JogFrame and republish
    def callback(self, joy):

        msg = JogFrame()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.group_name = rospy.get_param('~group_name', 'arm')

        # These buttons are binary
        msg.linear.x = 0.05*(-joy.buttons[4] + joy.buttons[5])
        # Double buttons
        msg.linear.y = 0.05*(joy.axes[0])
        msg.linear.z = 0.05*(joy.axes[1])

        msg.angular.x = 0
        msg.angular.y = 0
        # These buttons are binary
        msg.angular.z = 0
        msg.avoid_collisions = True

        self.pub.publish(msg)
        
    def republish(self):
        rospy.Subscriber("joy", Joy, self.callback)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joy_to_jog_frame', anonymous=True)
    republisher = joy_to_jog_frame()
    republisher.republish()
