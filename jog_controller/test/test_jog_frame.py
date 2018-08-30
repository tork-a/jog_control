#! /usr/bin/env python
# -*- coding: utf-8 -*-
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
import math
import rospy
import sys
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from jog_msgs.msg import JogFrame
import rospy
import rostest
from sensor_msgs.msg import JointState
import tf
import unittest

class TestJogFrameNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_jog_frame_node')

    def setUp(self):
        # Get parameters
        self.controller_name = rospy.get_param('~controller_name', None)
        self.action_name = rospy.get_param('~action_name', None)
        self.joint_names = rospy.get_param('~joint_names', [])
        self.home_positions = rospy.get_param('~home_positions', [])
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.group_name = rospy.get_param('~group_name', 'manipulator')
        self.link_name = rospy.get_param('~link_name', 'ee_link')
        self.linear_delta = rospy.get_param('~linear_delta', 0.005)
        self.angular_delta = rospy.get_param('~angular_delta', 0.01)
        # TF listener
        self.listener = tf.TransformListener()
        # Publishers
        self.pub = rospy.Publisher('jog_frame', JogFrame, queue_size=10)
        # Actionlib
        self.client = None
        if self.controller_name and self.action_name:
            self.client = actionlib.SimpleActionClient(
                self.controller_name + '/' + self.action_name, FollowJointTrajectoryAction)
            self.client.wait_for_server()
            # Move to home position
            self.move_to_home()
        rospy.sleep(3.0)
        
    def move_to_home(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal_time_tolerance = rospy.Time(0.1)
        goal.goal_time_tolerance = rospy.Time(0.1)
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_positions
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)

        if self.client:
            self.client.send_goal(goal)
            self.assertTrue(self.client.wait_for_result(timeout=rospy.Duration(5.0)))
            rospy.sleep(1.0)
        
    def test_a_jog_with_linear_delta(self):
        '''Test to jog a command with linear delta'''
        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time(), rospy.Duration(10))
        (start_pos, start_rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))
            
        # Create jog frame command
        jog = JogFrame()
        jog.header.stamp = rospy.Time.now()
        jog.header.frame_id = self.frame_id
        jog.group_name = self.group_name
        jog.link_name = self.link_name
        jog.linear_delta.x = self.linear_delta
        jog.linear_delta.y = self.linear_delta
        jog.linear_delta.z = self.linear_delta
        self.pub.publish(jog)
        rospy.sleep(3.0)

        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time(), rospy.Duration(10))
        (pos, rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))
        # Check if the position is jogged by delta
        for i in range(3):
            self.assertAlmostEqual(pos[i], start_pos[i] + self.linear_delta, delta=0.0001)
        # Check if the rotaion is not changed
        rot_diff = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_inverse(start_rot), rot)
        for i in range(4):
            self.assertAlmostEqual(start_rot[i], rot[i], delta=0.0001)

    def test_a_jog_with_angular_delta(self):
        '''Test to jog a command with angular delta'''
        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time(), rospy.Duration(10))
        (start_pos, start_rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))
            
        # Create jog frame command
        jog = JogFrame()
        jog.header.stamp = rospy.Time.now()
        jog.header.frame_id = self.frame_id
        jog.group_name = self.group_name
        jog.link_name = self.link_name
        jog.angular_delta.x = self.angular_delta / math.sqrt(3.0)
        jog.angular_delta.y = self.angular_delta / math.sqrt(3.0)
        jog.angular_delta.z = self.angular_delta / math.sqrt(3.0)
        self.pub.publish(jog)
        rospy.sleep(3.0)
        
        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time.now(), rospy.Duration(10))
        (pos, rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))
        # Check if the position is not changed
        for i in range(3):
            self.assertAlmostEqual(pos[i], start_pos[i], delta=0.0001)
        # Check if the rotaion is jogged by delta
        jog_q = tf.transformations.quaternion_about_axis(self.angular_delta, [1,1,1])
        ans_rot = tf.transformations.quaternion_multiply(jog_q, start_rot)
        for i in range(4):
            self.assertAlmostEqual(rot[i], ans_rot[i], delta=0.0001)

    def test_ten_jogs_with_linear_delta(self):
        '''Test to jog ten commands with linear delta'''
        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time(), rospy.Duration(10))
        (start_pos, start_rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))

        for i in range(10):
            # Create jog frame command
            jog = JogFrame()
            jog.header.stamp = rospy.Time.now()
            jog.header.frame_id = self.frame_id
            jog.group_name = self.group_name
            jog.link_name = self.link_name
            jog.linear_delta.x = self.linear_delta
            jog.linear_delta.y = self.linear_delta
            jog.linear_delta.z = self.linear_delta
            self.pub.publish(jog)
        rospy.sleep(3.0)
        
        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time(), rospy.Duration(10))
        (pos, rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))
        # Check if the position is jogged by delta
        for i in range(3):
            self.assertAlmostEqual(pos[i], start_pos[i] + self.linear_delta*10, delta=0.0001)
        # Check if the rotaion is not changed
        for i in range(4):
            self.assertAlmostEqual(start_rot[i], rot[i], delta=0.0001)

    def test_ten_jogs_with_angular_delta(self):
        '''Test to jog ten commands with angular delta'''
        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time(), rospy.Duration(10))
        (start_pos, start_rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))

        for i in range(10):
            # Create jog frame command
            jog = JogFrame()
            jog.header.stamp = rospy.Time.now()
            jog.header.frame_id = self.frame_id
            jog.group_name = self.group_name
            jog.link_name = self.link_name
            jog.angular_delta.x = self.angular_delta / math.sqrt(3.0)
            jog.angular_delta.y = self.angular_delta / math.sqrt(3.0)
            jog.angular_delta.z = self.angular_delta / math.sqrt(3.0)
            self.pub.publish(jog)
        rospy.sleep(3.0)

        # Get current posture of link_name
        self.listener.waitForTransform(
            self.frame_id, self.link_name, rospy.Time(), rospy.Duration(10))
        (pos, rot) = self.listener.lookupTransform(
            self.frame_id, self.link_name, rospy.Time(0))
        # Check if the position is not changed
        for i in range(3):
            self.assertAlmostEqual(pos[i], start_pos[i], delta=0.0001)
        # Check if the rotaion is jogged by delta
        jog_q = tf.transformations.quaternion_about_axis(self.angular_delta*10, [1,1,1])
        ans_rot = tf.transformations.quaternion_multiply(jog_q, start_rot)
        for i in range(4):
            self.assertAlmostEqual(rot[i], ans_rot[i], delta=0.0001)
            
if __name__ == '__main__':
    rostest.rosrun('jog_controller', 'test_jog_frame_node', TestJogFrameNode)
