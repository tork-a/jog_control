#!/usr/bin/env python

# Take spacemouse twist. Republish them as JogFrame

import rospy
from jog_msgs.msg import JogFrame
from geometry_msgs.msg import Twist
import numpy as np

class twist_to_jog_frame:

    def __init__(self):
        self.scale_linear = rospy.get_param('~scale_linear', {'x': 0.05, 'y': 0.05, 'z': 0.05})
        self.scale_angular = rospy.get_param('~scales_angular', {'x': 0.05, 'y': 0.05, 'z': 0.05})
        self.rotation_matrix = np.matrix(rospy.get_param('~rotation_matrix', [[1,0,0],[0,1,0],[0,0,1]]))
        self.pub = rospy.Publisher('jog_frame', JogFrame, queue_size=1)

    # Analyze a Twist_msg and return only the dominant axis
    def dominantAxisMode(self, twist_msg):
        axes = {twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z, twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z}
        axes = self.hasDuplicate(axes)
        for axis in axes:
            abs(axis)
        highest = axes.index(max(axes))

        dominantTwist = Twist()
        # Add the dominant value to the dominant axis
        if highest == 0:
            dominantTwist.linear.x = twist_msg.linear.x
        elif highest == 1:
            dominantTwist.linear.y = twist_msg.linear.y
        elif highest == 2:
            dominantTwist.linear.z = twist_msg.linear.z
        elif highest == 3:
            dominantTwist.angular.x = twist_msg.angular.x
        elif highest == 4:
            dominantTwist.angular.y = twist_msg.angular.y
        elif highest == 5:
            dominantTwist.angular.z = twist_msg.angular.z

        return dominantTwist

    def axesRemap(self, twist_msg):
        # Computed axis remap
        linear_mat = [twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z] * self.rotation_matrix
        angular_mat = [twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z] * self.rotation_matrix

        # Remapped values allocation
        remappedTwist = Twist()
        remappedTwist.linear.x = linear_mat[0,0]
        remappedTwist.linear.y = linear_mat[0,1]
        remappedTwist.linear.z = linear_mat[0,2]
        remappedTwist.angular.x = angular_mat[0,0]
        remappedTwist.angular.y = angular_mat[0,1]
        remappedTwist.angular.z = angular_mat[0,2]

        return remappedTwist

    # Check if two axis are identical
    # return a list with all members set to zero if true
    def hasDuplicate(self, duplicate): 
        final_list = [] 
        for num in duplicate: 
            if num not in final_list: 
                final_list.append(num) 
        
        if len(final_list) != len(duplicate):
            final_list = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} 
        return final_list

    # Convert to JogFrame and republish
    def callback(self, twist):

        msg = JogFrame()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = rospy.get_param('~frame_id', 'base_link')
        msg.group_name = rospy.get_param('~group_name', 'manipulator')
        msg.link_name = rospy.get_param('~link_name', 'tool0')

        if rospy.get_param('~dominant_mode', True):
            twist = self.dominantAxisMode(twist)

        if rospy.get_param('~axes_remap', True):
            twist = self.axesRemap(twist)

        msg.angular_delta.x = self.scale_angular['x']*twist.angular.x
        msg.angular_delta.y = self.scale_angular['y']*twist.angular.y
        msg.angular_delta.z = self.scale_angular['z']*twist.angular.z
    
        msg.linear_delta.x = self.scale_linear['x']*twist.linear.x
        msg.linear_delta.y = self.scale_linear['y']*twist.linear.y
        msg.linear_delta.z = self.scale_linear['z']*twist.linear.z
        
        msg.avoid_collisions = True

        self.pub.publish(msg)
        
    def republish(self):
        rospy.Subscriber(rospy.get_param('~sub_topic', 'spacenav/twist'), Twist, self.callback)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('twist_to_jog_frame', anonymous=True)
    republisher = twist_to_jog_frame()
    republisher.republish()
