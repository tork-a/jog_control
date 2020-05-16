#!/usr/bin/env python

import rospy
from jog_msgs.msg import JogFrame
from jog_msgs.msg import JogJoint
from sensor_msgs.msg import Joy


class joy_to_jog_frame:

    def __init__(self):
        self.enable_button = rospy.get_param('~enable_button', 4)
        self.angular_button = rospy.get_param('~angular_button', 5)
        self.frame_mode_button = rospy.get_param('~frame_mode_button', 8)
        self.joint_mode_button = rospy.get_param('~joint_mode_button', 9)
        self.axis_linear = rospy.get_param(
            '~axis_linear', {'x': 0, 'y': 1, 'z': 4})
        self.axis_angular = rospy.get_param(
            '~axis_angular', {'x': 0, 'y': 1, 'z': 4})
        self.axis_joints = rospy.get_param(
            '~axis_joints', {'J1': 1, 'J2': 2, 'J3': 3, 'J4': 4, 'J5': 5, 'J6': 6})
        self.scale_linear = rospy.get_param(
            '~scale_linear', {'x': 0.05, 'y': 0.05, 'z': 0.05})
        self.scale_angular = rospy.get_param(
            '~scales_angular', {'x': 0.05, 'y': 0.05, 'z': 0.05})
        self.scale_joints = rospy.get_param('~scale_joints', {
                                            'J1': 0.05, 'J2': 0.05, 'J3': 0.05, 'J4': 0.05, 'J5': 0.05, 'J6': 0.05})

        self.pub_frm = rospy.Publisher('jog_frame', JogFrame, queue_size=1)
        self.pub_jnt = rospy.Publisher('jog_joint', JogJoint, queue_size=1)

        self.mode = True  # True = frame, False = joint

    # Convert to JogFrame and republish
    def callback(self, joy):

        if joy.buttons[self.frame_mode_button]:
            self.mode = True
            rospy.loginfo('Mode: Frame')

        if joy.buttons[self.joint_mode_button]:
            self.mode = False
            rospy.loginfo('Mode: Joint')
            
        if not joy.buttons[self.enable_button]:
            return   

        if self.mode: #Frame_jog     

            msg_frm = JogFrame()

            msg_frm.header.stamp = rospy.Time.now()
            msg_frm.header.frame_id = rospy.get_param('~frame_id', 'base_link')
            msg_frm.group_name = rospy.get_param('~group_name', 'manipulator')
            msg_frm.link_name = rospy.get_param('~link_name', 'tool0')

            if joy.buttons[self.angular_button]:
                msg_frm.angular_delta.x = self.scale_angular['x']*joy.axes[self.axis_angular['x']]
                msg_frm.angular_delta.y = self.scale_angular['y']*joy.axes[self.axis_angular['y']]
                msg_frm.angular_delta.z = self.scale_angular['z']*joy.axes[self.axis_angular['z']]
            else:
                # These buttons are binary
                msg_frm.linear_delta.x = self.scale_linear['x']*joy.axes[self.axis_linear['x']]
                msg_frm.linear_delta.y = self.scale_linear['y']*joy.axes[self.axis_linear['y']]
                msg_frm.linear_delta.z = self.scale_linear['z']*joy.axes[self.axis_linear['z']]
            
            msg_frm.avoid_collisions = True
            self.pub_frm.publish(msg_frm)
        
        else: #Joint_jog

            msg_jnt = JogJoint()

            msg_jnt.header.stamp = rospy.Time.now()
            msg_jnt.header.frame_id = rospy.get_param('~frame_id', 'base_link')
            msg_jnt.joint_names = rospy.get_param("/jog_joint_node/joint_names","['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']")
            # These buttons are binary
            msg_jnt.deltas = [0]*6
            msg_jnt.deltas[0] = self.scale_joints['J1']*(joy.axes[self.axis_joints['J1']])
            msg_jnt.deltas[1] = self.scale_joints['J2']*(joy.axes[self.axis_joints['J2']])
            msg_jnt.deltas[2] = self.scale_joints['J3']*(joy.axes[self.axis_joints['J3']])
            msg_jnt.deltas[3] = self.scale_joints['J4']*(joy.axes[self.axis_joints['J4']])
            msg_jnt.deltas[4] = self.scale_joints['J5']*(joy.axes[self.axis_joints['J5']])
            msg_jnt.deltas[5] = self.scale_joints['J6']*(joy.axes[self.axis_joints['J6']])

            self.pub_jnt.publish(msg_jnt)

    def republish(self):
        rospy.Subscriber(rospy.get_param('~sub_topic', 'joy'), Joy, self.callback)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joy_to_jog_frame', anonymous=True)
    republisher = joy_to_jog_frame()
    republisher.republish()
