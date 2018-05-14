#include <jog_controller/jog_frame_node.h>

namespace jog_frame {

JogFrameNode::JogFrameNode()
  : nh_("~")
{
  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &JogFrameNode::joint_state_cb, this);
  jog_frame_sub_ = nh_.subscribe("/jog_frame", 1, &JogFrameNode::jog_frame_cb, this);
  fk_client_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");  
  ik_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");  
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
  
  traj_pub_ =  nh_.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 10);
  
}

/**
 * @brief Callback function for the topic cmd_vel
 *
 */
void JogFrameNode::jog_frame_cb(jog_msgs::JogFrameConstPtr msg)
{
  moveit_msgs::GetPositionIK srv;
  srv.request.ik_request.group_name = msg->group_name;
  if (msg->link_name.empty())
  {
    srv.request.ik_request.ik_link_name = msg->link_name;
  }
  srv.request.ik_request.robot_state.joint_state = joint_state_;
  srv.request.ik_request.avoid_collisions = msg->avoid_collisions;

  geometry_msgs::Pose act_pose = pose_stamped_.pose;
  geometry_msgs::PoseStamped ref_pose;
  
  ref_pose.header.frame_id = msg->header.frame_id;
  ref_pose.header.stamp = ros::Time::now();
  ref_pose.pose.position.x = act_pose.position.x + msg->linear.x;
  ref_pose.pose.position.y = act_pose.position.y + msg->linear.y;
  ref_pose.pose.position.z = act_pose.position.z + msg->linear.z;
  
  ref_pose.pose.orientation.x = act_pose.orientation.x;
  ref_pose.pose.orientation.y = act_pose.orientation.y;
  ref_pose.pose.orientation.z = act_pose.orientation.z;
  ref_pose.pose.orientation.w = act_pose.orientation.w;
  
  ROS_INFO_STREAM("ref_pose" << ref_pose);

  srv.request.ik_request.pose_stamped = ref_pose;

  if (ik_client_.call(srv))
  {
    ROS_INFO_STREAM("response: " << srv.response);
  }
  else
  {
  ROS_ERROR("Failed to call service /compute_ik");
  }
  if (srv.response.error_code.val < 0)
  {
    ROS_ERROR("IK error");
    return;
  }
  ref_joint_state_ = srv.response.solution.joint_state;

  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
  traj.header.frame_id = "base_link";
  traj.joint_names = ref_joint_state_.name;
    
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = srv.response.solution.joint_state.position;
  point.velocities.resize(ref_joint_state_.name.size());
  point.accelerations.resize(ref_joint_state_.name.size());
  point.time_from_start = ros::Duration(1.0);
  traj.points.push_back(point);

  traj_pub_.publish(traj);
}

/**
 * @brief Callback function for the topic joint_state
 *
 */
void JogFrameNode::joint_state_cb(sensor_msgs::JointStateConstPtr msg)
{
  // Check that the msg contains joints
  if (msg->name.empty())
  {
    ROS_ERROR("JogFrameNode::joint_state_cb - joint_state is empty.");
    return;
  }
  joint_state_ = *msg;

  moveit_msgs::GetPositionFK srv;
  srv.request.fk_link_names.clear();
  srv.request.fk_link_names.push_back(target_link_);
  srv.request.robot_state.joint_state = joint_state_;
  if (fk_client_.call(srv))
  {
    if(srv.response.error_code.val < 0)
    {
      ROS_ERROR("****FK error %d", srv.response.error_code.val);
      return;
    }
    pose_stamped_ = srv.response.pose_stamped[0];
    for (int i=0; i<srv.response.fk_link_names.size(); i++)
    {
      // ROS_INFO_STREAM(srv.response.fk_link_names[i] << ":" << srv.response.pose_stamped[i]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service /computte_fk");
  }    
}

} // namespace jog_arm

/**
 * @brief Main function of the node
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "jog_frame_node");
  jog_frame::JogFrameNode node;

  ros::Rate loop_rate(100);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

