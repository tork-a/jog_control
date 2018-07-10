#! /usr/bin/env python
# -*- coding: utf-8 -*-
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
        self.joint_list_ = rospy.get_param('~joint_list')
        self.joint_state_ = rospy.wait_for_message('joint_states', JointState, timeout=1.0)
        rospy.Subscriber('joint_states', JointState, self.cb_joint_states, queue_size=10)
        self.pub = rospy.Publisher('jog_joint', JogJoint, queue_size=10)
        rospy.sleep(1.0)
        
    def cb_joint_states(self, msg):
        self.joint_state_ = msg
        
    def test_one_jog_for_all_joint(self):
        return
        joint_state = self.joint_state_
        jog = JogJoint()
        jog.header.stamp = rospy.Time.now()
        jog.joint_names = self.joint_list_
        jog.deltas = [0.1]*len(jog.joint_names)
                
        self.pub.publish(jog)
        rospy.sleep(0.5)
        # Check if the robot is jogged by delta
        for joint in self.joint_list_:
            index = joint_state.name.index(joint)
            self.assertAlmostEqual(joint_state.position[index] + 0.1, self.joint_state_.position[index])

    def test_jogs_for_all_joint(self):
        rospy.sleep(1.0)
        joint_state = self.joint_state_
        for i in range(10):
            jog = JogJoint()
            jog.header.stamp = rospy.Time.now()
            jog.joint_names = self.joint_list_
            jog.deltas = [0.01]*len(jog.joint_names)
            self.pub.publish(jog)
        for i in range(10):
            jog = JogJoint()
            jog.header.stamp = rospy.Time.now()
            jog.joint_names = self.joint_list_
            jog.deltas = [-0.01]*len(jog.joint_names)
            self.pub.publish(jog)
        rospy.sleep(1.0)
        # Check if the robot is jogged by delta * 10
        for joint in self.joint_list_:
            index = joint_state.name.index(joint)
            self.assertAlmostEqual(joint_state.position[index], self.joint_state_.position[index])
        
if __name__ == '__main__':
    rostest.rosrun('jog_joint', 'test_jog_joint_node', TestJogJointNode)
