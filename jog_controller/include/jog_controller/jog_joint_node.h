#ifndef JOG_JOINT_H
#define JOG_JOINT_H

#include <string>
#include <ros/ros.h>
#include <jog_msgs/JogJoint.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction > TrajClient;

namespace jog_joint
{

typedef struct
{
public:
  std::string action_ns;
  std::string type;
  std::vector<std::string> joints;
  
} Controller;

/**
 * Class JogJointNode - Provides the jog_joint
 */
class JogJointNode
{
public:
  /**
   * @breif: Default constructor for JogJointNode Class.
   */
  JogJointNode ();
  int get_controller_list();
  void jog_joint_cb (jog_msgs::JogJointConstPtr msg);
  void joint_state_cb (sensor_msgs::JointStateConstPtr msg);

protected:
  ros::Subscriber joint_state_sub_, jog_joint_sub_;
    
  std::map<std::string, Controller> cinfo_map_;
  std::map<std::string, TrajClient*> traj_clients_;
  std::map<std::string, ros::Publisher> traj_pubs_;
  ros::Publisher joint_state_pub_;

  std::map<std::string, double> joint_map_;
  sensor_msgs::JointState joint_state_;
    
  ros::Time last_stamp_;
  double time_from_start_;
  bool use_action_;
  bool intermittent_;
};

} // namespace jog_joint

#endif // JOG_JOINT_NODE_H
