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
  //nh_.param<std::string>("controller_name", controller_name, "joint_trajectory_controller");
  nh_.param<std::string>("target_link", target_link_, "link_6");
  nh_.param<std::string>("group", group_name_);
  //nh_.param<std::vector <std::string> >("joint_names", joint_names_);
  nh_.getParam("joint_names", joint_names_);

  nh_.param<bool>("use_action", use_action_, false);

  if (!nh_.hasParam("/move_group/controller_list"))
  {
    ROS_ERROR_STREAM("No controller_list specified.");
  }
  // Get controller information from move_group/controller_list
  XmlRpc::XmlRpcValue controller_list;
  nh_.getParam("/move_group/controller_list", controller_list);

  /* actually create each controller */
  for (int i = 0; i < controller_list.size(); ++i)
  {
    if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
    {
      ROS_ERROR_STREAM("Name and joints must be specifed for each controller");
      continue;
    }
    try
    {
      std::string name = std::string(controller_list[i]["name"]);

      if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM("The list of joints for controller " << name << " is not specified as an "
                         "array");
        continue;
      }
      if (!controller_list[i].hasMember("type"))
      {
        ROS_ERROR_STREAM("No type specified for controller " << name);
        continue;
      }
      std::string type = std::string(controller_list[i]["type"]);
      if (type == "FollowJointTrajectory")
      {
        joint_map_[name].resize(controller_list[i]["joints"].size());
        for (int j = 0; j < joint_map_[name].size(); ++j)
        {
          joint_map_[name][j] = std::string(controller_list[i]["joints"][j]);
          // controller_map_[std::string(controller_list[i]["joints"][j])] = name;
        }
      }
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Caught unknown exception while parsing controller information");
    }
  }

  ROS_INFO_STREAM("controller_map:");
  for (auto it=controller_map_.begin(); it!=controller_map_.end(); it++)
  {
    ROS_INFO_STREAM(it->first << "," << it->second);
  }

#if 0
  /* Get robot state */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  robot_state_ = new robot_state::RobotState(robot_model);
  const robot_state::JointModelGroup* joint_group = robot_model->getJointModelGroup(group_name_);

  /* Get all groups */
  joint_groups_.resize(group_names_.size());
  for (int i=0; i<joint_groups_.size(); i++)
  {
    joint_groups_[i] = robot_model->getJointModelGroup(group_names_[i]);
  }
#endif
  
  if (use_action_)
  {
    traj_client_ = new TrajClient(controller_name + "/joint_trajectory_action", true);
    
    while(!traj_client_->waitForServer(ros::Duration(60)))
    {
      ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
    }
    ROS_INFO_STREAM("Action server is ok!");
  }
  else
  {
    // Create publisher for each controller
    for (auto it=joint_map_.begin(); it!=joint_map_.end(); it++)
    {
      auto name = it->first;
      traj_pubs_[name] = nh_.advertise<trajectory_msgs::JointTrajectory>("/" + name + "/command", 10);
    }
  }
}

/**
 * @brief Callback function for the topic jog_frame
 * 
*/
void JogFrameNode::jog_frame_cb(jog_msgs::JogFrameConstPtr msg)
{
  // Update forward kinematics
  moveit_msgs::GetPositionFK fk;

  fk.request.header.frame_id = msg->header.frame_id;
  fk.request.header.stamp = ros::Time::now();
  fk.request.fk_link_names.clear();
  fk.request.fk_link_names.push_back(msg->link_name);
  fk.request.robot_state.joint_state = joint_state_;

  if (fk_client_.call(fk))
  {
    if(fk.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_WARN("****FK error %d", fk.response.error_code.val);
      return;
    }
    if (fk.response.pose_stamped.size() != 1)
    {
      for (int i=0; i<fk.response.pose_stamped.size(); i++)
      {
        ROS_ERROR_STREAM("fk[" << i << "]:\n" << fk.response.pose_stamped[0]);
      }
    }
    pose_stamped_ = fk.response.pose_stamped[0];
  }
  else
  {
    ROS_ERROR("Failed to call service /computte_fk");
    return;
  }

  // Solve inverse kinematics
  moveit_msgs::GetPositionIK ik;

  ik.request.ik_request.group_name = msg->group_name;
  ik.request.ik_request.ik_link_name = msg->link_name;
  ik.request.ik_request.robot_state.joint_state = joint_state_;
  //ik.request.ik_request.avoid_collisions = msg->avoid_collisions;
  ik.request.ik_request.avoid_collisions = true;
  
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
  
  ik.request.ik_request.pose_stamped = ref_pose;

  // ROS_INFO_STREAM("ik:\n" << ik.request);

  if (!ik_client_.call(ik))
  {
    ROS_ERROR("Failed to call service /compute_ik");
    return;
  }
  if (ik.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_WARN("****IK error %d", ik.response.error_code.val);
    return;
  }

  auto state = ik.response.solution.joint_state;
  geometry_msgs::PoseStamped pose_check;
  
  // Make sure the solution is valid in joint space
  double error = 0;
  for (int i=0; i<state.name.size(); i++)
  {
    for (int j=0; j<joint_state_.name.size(); j++)
    {
      if (state.name[i] == joint_state_.name[j])
      {
        double e = fabs(state.position[i] - joint_state_.position[j]);
        if (e > error)
        {
          error = e;
        }
        break;
      }
    }
  }
  if (error > 0.5)
  {
    ROS_WARN_STREAM("**** Validation check Failed: " << error);
    return;
  }
  
  // Make trajectory message for each controller
  for (auto it=joint_map_.begin(); it!=joint_map_.end(); it++)
  {
    std::vector<double> positions, velocities, accelerations;
    auto joint_names = it->second;
    auto controller_name = it->first;

    positions.resize(joint_names.size());
    velocities.resize(joint_names.size());
    accelerations.resize(joint_names.size());

    for (int i=0; i<joint_names.size(); i++)
    {
      size_t index = std::distance(state.name.begin(),
                                   std::find(state.name.begin(),
                                             state.name.end(), joint_names[i]));
      positions[i] = state.position[index];
      velocities[i] = 0;
      accelerations[i] = 0;
    }
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = velocities;
    point.accelerations = accelerations;
    point.time_from_start = ros::Duration(1.0);

    if (use_action_)
    {
      //point.time_from_start = ros::Duration(0.2);

      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.header.stamp = ros::Time::now();
      goal.trajectory.header.frame_id = "base_link";
      goal.trajectory.joint_names = joint_names;

      goal.trajectory.points.push_back(point);
    
      traj_client_->sendGoal(goal);
    }
    else
    {
      trajectory_msgs::JointTrajectory traj;
      traj.header.stamp = ros::Time::now();
      traj.header.frame_id = "base_link";
      traj.joint_names = joint_names;
      traj.points.push_back(point);
      
      traj_pubs_[controller_name].publish(traj);
    }
  }
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
}

} // namespace jog_arm

/**
 * @brief Main function of the node
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "jog_frame_node");
  jog_frame::JogFrameNode node;

  ros::Rate loop_rate(10);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

