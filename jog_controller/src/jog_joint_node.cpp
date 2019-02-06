#include <jog_controller/jog_joint_node.h>

namespace jog_joint {

int JogJointNode::get_controller_list()
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

JogJointNode::JogJointNode()
{
  ros::NodeHandle gnh, pnh("~");

  pnh.param<double>("time_from_start", time_from_start_, 0.2);
  pnh.param<bool>("use_action", use_action_, false);
  pnh.param<bool>("intermittent", intermittent_, false);

  if (not use_action_ && intermittent_)
  {
    ROS_WARN("'intermittent' param should be true with 'use_action'. Assuming to use action'");
    use_action_ = true;
  }
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
  joint_state_sub_ = gnh.subscribe("joint_states", 20, &JogJointNode::joint_state_cb, this);
  jog_joint_sub_ = gnh.subscribe("jog_joint", 20, &JogJointNode::jog_joint_cb, this);

  // Reference joint_state publisher
  joint_state_pub_ = pnh.advertise<sensor_msgs::JointState>("reference_joint_states", 10);

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
        ROS_WARN_STREAM("Waiting for " << action_name << "server...");
      }
    }
  }
  else
  {
    // Create publisher for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      traj_pubs_[controller_name] = gnh.advertise<trajectory_msgs::JointTrajectory>(controller_name + "/command", 1);
    }
  }
}

/**
 * @brief Callback function for the topic jog_joint
 *
 */
void JogJointNode::jog_joint_cb(jog_msgs::JogJointConstPtr msg)
{
  // Validate the message
  if (msg->joint_names.size() != msg->deltas.size())
  {
    ROS_ERROR("Size mismatch of joint_names and deltas");
    return;
  }
  // In intermittent mode, confirm to all of the action is completed
  if (intermittent_)
  {
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      actionlib::SimpleClientGoalState state = traj_clients_[controller_name]->getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE)
      {
        return;
      }
    }    
  }
  
  // Update only if the stamp is older than last_stamp_ + time_from_start
  if (msg->header.stamp > last_stamp_ + ros::Duration(time_from_start_))
  {
    ROS_INFO("start joint state updated");
    // Update reference joint_state
    joint_state_.name.clear();
    joint_state_.position.clear();
    joint_state_.velocity.clear();
    joint_state_.effort.clear();
    for (auto it=joint_map_.begin(); it!=joint_map_.end(); it++)
    {
      joint_state_.name.push_back(it->first);
      joint_state_.position.push_back(it->second);
      joint_state_.velocity.push_back(0.0);
      joint_state_.effort.push_back(0.0);
    }
    // Publish reference joint state for debug
    joint_state_pub_.publish(joint_state_);
  }  
  // Update timestamp of the last jog command
  last_stamp_ = msg->header.stamp;
  
  // Publish trajectory message for each controller
  for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
  {
    auto controller_name = it->first;
    auto joint_names = it->second.joints;

    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(time_from_start_);

    for (int i=0; i<joint_names.size(); i++)
    {
      size_t jog_index = std::distance(msg->joint_names.begin(),
                                       std::find(msg->joint_names.begin(),
                                                 msg->joint_names.end(), joint_names[i]));
      if (jog_index == msg->joint_names.size())
      {
        ROS_INFO_STREAM("Cannot find joint in jog_joint: " << joint_names[i]);
        continue;
      }
      size_t state_index = std::distance(joint_state_.name.begin(),
                                         std::find(joint_state_.name.begin(),
                                                   joint_state_.name.end(), joint_names[i]));
      if (state_index == joint_state_.name.size())
      {
        ROS_ERROR_STREAM("Cannot find joint " << joint_names[i] << " in joint_states_");
        continue;
      }
      // Update start joint position
      joint_state_.position[state_index] += msg->deltas[jog_index];
      // Fill joint trajectory joint_names and positions
      traj.joint_names.push_back(joint_names[i]);
      point.positions.push_back(joint_state_.position[state_index]);
      point.velocities.push_back(0.0);
      point.accelerations.push_back(0.0);
    }
    // Fill joint trajectory members
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = "base_link";
    traj.points.push_back(point);
    if (use_action_)
    {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory = traj;
      traj_clients_[controller_name]->sendGoal(goal);
    }
    else
    {
      traj_pubs_[controller_name].publish(traj);
    }
  }
  // Publish start joint state for debug
  joint_state_pub_.publish(joint_state_);
}

/**
 * @brief Callback function for the topic joint_state
 *
 */
void JogJointNode::joint_state_cb(sensor_msgs::JointStateConstPtr msg)
{
  // Check that the msg contains joints
  if (msg->name.empty() || msg->name.size() != msg->position.size())
  {
    ROS_WARN("Invalid JointState message");
    return;
  }
  for (int i=0; i<msg->name.size(); i++)
  {
    joint_map_[msg->name[i]] = msg->position[i];
  }
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

