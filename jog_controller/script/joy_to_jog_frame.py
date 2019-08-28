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
        self.enable_button = rospy.get_param('~enable_button', 4)
        self.angular_button = rospy.get_param('~angular_button', 5)
        self.axis_linear = rospy.get_param('~axis_linear', {'x': 0, 'y': 1, 'z': 4})
        self.axis_angular = rospy.get_param('~axis_angular', {'x': 0, 'y': 1, 'z': 4})
        self.scale_linear = rospy.get_param('~scale_linear', {'x': 0.05, 'y': 0.05, 'z': 0.05})
        self.scale_angular = rospy.get_param('~scales_angular', {'x': 0.05, 'y': 0.05, 'z': 0.05})

        self.pub = rospy.Publisher('jog_frame', JogFrame, queue_size=1)

    # Convert to JogFrame and republish
    def callback(self, joy):

        if not joy.buttons[self.enable_button]:
            return

        msg = JogFrame()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = rospy.get_param('~frame_id', 'base_link')
        msg.group_name = rospy.get_param('~group_name', 'manipulator')
        msg.link_name = rospy.get_param('~link_name', 'tool0')

        if joy.buttons[self.angular_button]:
            msg.angular_delta.x = self.scale_angular['x']*joy.axes[self.axis_angular['x']]
            msg.angular_delta.y = self.scale_angular['y']*joy.axes[self.axis_angular['y']]
            msg.angular_delta.z = self.scale_angular['z']*joy.axes[self.axis_angular['z']]
        else:
            # These buttons are binary
            msg.linear_delta.x = self.scale_linear['x']*joy.axes[self.axis_linear['x']]
            msg.linear_delta.y = self.scale_linear['y']*joy.axes[self.axis_linear['y']]
            msg.linear_delta.z = self.scale_linear['z']*joy.axes[self.axis_linear['z']]
        
        msg.avoid_collisions = True
        self.pub.publish(msg)
        
    def republish(self):
        rospy.Subscriber(rospy.get_param('~sub_topic', 'joy'), Joy, self.callback)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joy_to_jog_frame', anonymous=True)
    republisher = joy_to_jog_frame()
    republisher.republish()
