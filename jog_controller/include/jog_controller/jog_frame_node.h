#ifndef JOG_FRAME_H
#define JOG_FRAME_H

#include <string>
#include <ros/ros.h>
#include <jog_msgs/JogFrame.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient < control_msgs::FollowJointTrajectoryAction > TrajClient;

namespace jog_frame
{

/**
 * Class JogFrameNode - Provides the jog_frame
 */
  class JogFrameNode
  {
  public:
  /**
   * @breif: Default constructor for JogFrameNode Class.
   */
    JogFrameNode ();
    void jog_frame_cb (jog_msgs::JogFrameConstPtr msg);
    void joint_state_cb (sensor_msgs::JointStateConstPtr msg);

  protected:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_, jog_frame_sub_;
    ros::ServiceClient fk_client_, ik_client_;

    TrajClient *traj_client_;
    ros::Publisher traj_pub_;

    sensor_msgs::JointState joint_state_;
    geometry_msgs::PoseStamped pose_stamped_;
    sensor_msgs::JointState ref_joint_state_;
    std::string target_link_;
  };

}                               // namespace jog_frame

#endif                          // JOG_FRAME_NODE_H
