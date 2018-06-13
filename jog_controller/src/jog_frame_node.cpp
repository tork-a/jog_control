#include <cfloat>
#include <jog_controller/jog_frame_node.h>

namespace jog_frame {

int JogFrameNode::get_controller_list()
{
  ros::NodeHandle gnh;

  // Get controller information from move_group/controller_list
  if (!gnh.hasParam("move_group/controller_list"))
  {
    ROS_ERROR_STREAM("move_group/controller_list is not specified.");
    return -1;
  }
  XmlRpc::XmlRpcValue controller_list;
  gnh.getParam("move_group/controller_list", controller_list);
  for (int i = 0; i < controller_list.size(); i++)
  {
    if (!controller_list[i].hasMember("name"))
    {
      ROS_ERROR("name must be specifed for each controller.");
      return -1;
    }
    if (!controller_list[i].hasMember("joints"))
    {
      ROS_ERROR("joints must be specifed for each controller.");
      return -1;
    }
    try
    {
      // get name member
      std::string name = std::string(controller_list[i]["name"]);
      // get action_ns member if exists
      std::string action_ns = std::string("");
      if (controller_list[i].hasMember("action_ns"))
      {
        action_ns = std::string(controller_list[i]["action_ns"]);
      }
      // get joints member
      if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM("joints for controller " << name << " is not specified as an array");
        return -1;
      }
      auto joints = controller_list[i]["joints"];
      // Get type member
      std::string type = std::string("FollowJointTrajectory");
      if (!controller_list[i].hasMember("type"))
      {
        ROS_WARN_STREAM("type is not specifed for controller " << name << ", using default FollowJointTrajectory");
      }
      type = std::string(controller_list[i]["type"]);
      if (type != "FollowJointTrajectory")
      {
        ROS_ERROR_STREAM("controller type " << type << " is not supported");
        return -1;
      }
      // Create controller map
      cinfo_map_[name].action_ns = action_ns;
      cinfo_map_[name].joints.resize(joints.size());
      for (int j = 0; j < cinfo_map_[name].joints.size(); ++j)
      {
        cinfo_map_[name].joints[j] = std::string(joints[j]);
      }
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Caught unknown exception while parsing controller information");
      return -1;
    }
  }
  return 0;
}

JogFrameNode::JogFrameNode()
{
  ros::NodeHandle gnh, pnh("~");

  std::string controller_name;
  pnh.param<std::string>("target_link", target_link_, "link_6");
  pnh.param<std::string>("group", group_name_);
  pnh.param<double>("time_from_start", time_from_start_, 0.5);
  pnh.param<bool>("use_action", use_action_, false);

  if (get_controller_list() < 0)
  {
    ROS_ERROR("get_controller_list faild. Aborted.");
    ros::shutdown();
    return;
  }
  ROS_INFO_STREAM("controller_list:");
  for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
  {
    auto cinfo = it->second;
    ROS_INFO_STREAM("- " << it->first);
    for (int i=0; i<cinfo.joints.size(); i++)
    {
      ROS_INFO_STREAM("  - " << cinfo.joints[i]);
    }    
  }  

  // Create subscribers
  joint_state_sub_ = gnh.subscribe("joint_states", 1, &JogFrameNode::joint_state_cb, this);
  jog_frame_sub_ = gnh.subscribe("jog_frame", 1, &JogFrameNode::jog_frame_cb, this);
  fk_client_ = gnh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");  
  ik_client_ = gnh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");  
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  
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
    // Create action client for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      auto controller_info = it->second;
      auto action_name = controller_name + "/" + controller_info.action_ns;
      
      traj_clients_[controller_name] = new TrajClient(action_name, true);
    }
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      auto controller_info = it->second;
      auto action_name = controller_name + "/" + controller_info.action_ns;
      
      for(;;)
      {
        if (traj_clients_[controller_name]->waitForServer(ros::Duration(1)))
        {
          ROS_INFO_STREAM(action_name << " is ready.");
          break;
        }
        ROS_WARN_STREAM("Waiting for " << action_name << " server...");
      }
    }
  }
  else
  {
    // Create publisher for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      traj_pubs_[controller_name] = gnh.advertise<trajectory_msgs::JointTrajectory>(controller_name + "/command", 10);
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
  ref_pose.pose.position.x = act_pose.position.x + msg->linear_delta.x;
  ref_pose.pose.position.y = act_pose.position.y + msg->linear_delta.y;
  ref_pose.pose.position.z = act_pose.position.z + msg->linear_delta.z;

  // Apply orientation jog
  tf::Quaternion q_ref, q_act, q_jog;
  tf::quaternionMsgToTF(act_pose.orientation, q_act);
  double angle = sqrt(msg->angular_delta.x*msg->angular_delta.x +
                      msg->angular_delta.y*msg->angular_delta.y +
                      msg->angular_delta.z*msg->angular_delta.z);
  tf::Vector3 axis(0,0,1);
  if (fabs(angle) < DBL_EPSILON)
  {
    angle = 0.0;
  }
  else
  {
    axis.setX(msg->angular_delta.x/angle);
    axis.setY(msg->angular_delta.y/angle);
    axis.setZ(msg->angular_delta.z/angle);
  }
  //ROS_INFO_STREAM("axis: " << axis.x() << ", " << axis.y() << ", " << axis.z());
  //ROS_INFO_STREAM("angle: " << angle);

  q_jog.setRotation(axis, angle);
  q_ref = q_jog*q_act;
  quaternionTFToMsg(q_ref, ref_pose.pose.orientation);

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
  
  // Publish trajectory message for each controller
  for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
  {
    auto controller_name = it->first;
    auto joint_names = it->second.joints;

    std::vector<double> positions, velocities, accelerations;

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
    point.time_from_start = ros::Duration(time_from_start_);

    if (use_action_)
    {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.header.stamp = ros::Time::now();
      goal.trajectory.header.frame_id = "base_link";
      goal.trajectory.joint_names = joint_names;
      goal.trajectory.points.push_back(point);

      traj_clients_[controller_name]->sendGoal(goal);
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

