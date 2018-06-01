#include <jog_controller/jog_joint_node.h>

namespace jog_joint {

JogJointNode::JogJointNode()
  : nh_("~")
{
  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &JogJointNode::joint_state_cb, this);
  jog_joint_sub_ = nh_.subscribe("/jog_joint", 1, &JogJointNode::jog_joint_cb, this);
  ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

  std::string controller_name;
  nh_.param<std::string>("controller_name", controller_name, "joint_trajectory_controller");
  nh_.param<std::string>("target_link", target_link_, "link_6");

#if 0  
  traj_client_ = new TrajClient(controller_name + "/follow_joint_trajectory", true);
    
  while(!traj_client_->waitForServer(ros::Duration(60)))
  {
    ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
  }
  ROS_INFO_STREAM("Action server is ok!");
#endif

  // For MoveIt! fake controller
  traj_client_ = new TrajClient("/execute_trajectory", true);
  while(!traj_client_->waitForServer(ros::Duration(60)))
  {
    ROS_INFO_STREAM("Waiting for the MoveIt! fake joint controller");
  }
  ROS_INFO_STREAM("Action server is ok!");
  
  traj_pub_ =  nh_.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 10);
  
}

/**
 * @brief Callback function for the topic cmd_vel
 *
 */
void JogJointNode::jog_joint_cb(jog_msgs::JogJointConstPtr msg)
{
  if (msg->name.size() != msg->displacement.size())
  {
    ROS_ERROR("JogJoint: size mismatch");
    return;
  }
  ref_joint_state_.name = msg->name;
  ref_joint_state_.position.resize(msg->name.size());
  
  for (int i=0; i<ref_joint_state_.name.size(); i++)
  {
    // C++ is insane...
    std::vector<std::string>::iterator it = \
      std::find(joint_state_.name.begin(),
                joint_state_.name.end(), ref_joint_state_.name[i]);
    size_t index = std::distance(joint_state_.name.begin(), it);
    if (index == joint_state_.name.size())
    {
      continue;
    }
    ref_joint_state_.position[i] = joint_state_.position[index] + msg->displacement[i];
  }
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
  traj.header.frame_id = "base_link";
  traj.joint_names = ref_joint_state_.name;
    
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = ref_joint_state_.position;
  point.velocities.resize(ref_joint_state_.name.size());
  point.accelerations.resize(ref_joint_state_.name.size());
  point.time_from_start = ros::Duration(0.1);
  traj.points.push_back(point);

#define USE_MOVEIT 0
#if USE_MOVEIT
  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory.joint_trajectory = traj;
  traj_client_->sendGoal(goal);
#else
  traj_pub_.publish(traj);
#endif

}

/**
 * @brief Callback function for the topic joint_state
 *
 */
void JogJointNode::joint_state_cb(sensor_msgs::JointStateConstPtr msg)
{
  // Check that the msg contains joints
  if (msg->name.empty())
  {
    ROS_ERROR("JogJointNode::joint_state_cb - joint_state is empty.");
    return;
  }
  joint_state_ = *msg;
}

} // namespace jog_arm

/**
 * @brief Main function of the node
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "jog_joint_node");
  jog_joint::JogJointNode node;

  ros::Rate loop_rate(100);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

