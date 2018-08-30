#! /usr/bin/env python
# -*- coding: utf-8 -*-
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
import rospy
import sys
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
import sys
from jog_msgs.msg import JogJoint
import rospy
import rostest
from sensor_msgs.msg import JointState
import unittest

        

class TestJogJointNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_jog_joint_node')

    def setUp(self):
        # Get parameters
        self.controller_name = rospy.get_param('~controller_name', None)
        self.action_name = rospy.get_param('~action_name', None)
        self.joint_names = rospy.get_param('~joint_names', [])
        self.home_positions = rospy.get_param('~home_positions', [])
        self.delta = rospy.get_param('~delta', 0.01)
        # Subscribers
        self.joint_state_ = rospy.wait_for_message('joint_states', JointState, timeout=10.0)
        rospy.Subscriber('joint_states', JointState, self.cb_joint_states, queue_size=10)
        # Publishers
        self.pub = rospy.Publisher('jog_joint', JogJoint, queue_size=50)
        # Actionlib
        self.client = None
        if self.controller_name and self.action_name:
            self.client = actionlib.SimpleActionClient(
                self.controller_name + '/' + self.action_name, FollowJointTrajectoryAction)
            self.client.wait_for_server()
            # Move to home position
            self.move_to_home()
        
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
        
    def cb_joint_states(self, msg):
        self.joint_state_ = msg
    
    def test_one_jog_for_all_joint(self):
        '''Test to jog a jog command'''
        joint_state = self.joint_state_
        jog = JogJoint()
        jog.header.stamp = rospy.Time.now()
        jog.joint_names = self.joint_names
        jog.deltas = [self.delta]*len(jog.joint_names)
        self.pub.publish(jog)
        rospy.sleep(1.0)
        # Check if the robot is jogged by delta
        for joint in self.joint_names:
            index = joint_state.name.index(joint)
            self.assertAlmostEqual(
                joint_state.position[index] + self.delta, self.joint_state_.position[index], delta=0.0001)

    def test_ten_jogs_for_all_joint(self):
        '''Test to jog continuous 10 jog commands'''
        joint_state = self.joint_state_
        for i in range(10):
            jog = JogJoint()
            jog.header.stamp = rospy.Time.now()
            jog.joint_names = self.joint_names
            jog.deltas = [self.delta]*len(jog.joint_names)
            self.pub.publish(jog)
        rospy.sleep(1.0)
        # Check if the robot is jogged by 10*delta
        for joint in self.joint_names:
            index = joint_state.name.index(joint)
            self.assertAlmostEqual(
                joint_state.position[index] + 10*self.delta, self.joint_state_.position[index], delta=0.0001)

    def test_loop_jogs_for_all_joint(self):
        '''Test to jog same amount for forward and backward'''
        joint_state = self.joint_state_
        for i in range(10):
            jog = JogJoint()
            jog.header.stamp = rospy.Time.now()
            jog.joint_names = self.joint_names
            jog.deltas = [self.delta]*len(jog.joint_names)
            self.pub.publish(jog)
        for i in range(10):
            jog = JogJoint()
            jog.header.stamp = rospy.Time.now()
            jog.joint_names = self.joint_names
            jog.deltas = [-self.delta]*len(jog.joint_names)
            self.pub.publish(jog)
        rospy.sleep(1.0)
        # Check if the robot is not moved
        for joint in self.joint_names:
            index = joint_state.name.index(joint)
            self.assertAlmostEqual(
                joint_state.position[index],
                self.joint_state_.position[index],
                delta=0.0001)

    def test_empty_jog_for_all_joint(self):
        '''Test to publish empty jonit_names/positions jog command'''
        joint_state = self.joint_state_
        jog = JogJoint()
        jog.header.stamp = rospy.Time.now()
        self.pub.publish(jog)
        rospy.sleep(1.0)
        # Check if the robot is not moved
        for joint in self.joint_names:
            index = joint_state.name.index(joint)
            self.assertAlmostEqual(
                joint_state.position[index],
                self.joint_state_.position[index],
                delta=0.0001)
            
if __name__ == '__main__':
    rostest.rosrun('jog_joint', 'test_jog_joint_node', TestJogJointNode)
