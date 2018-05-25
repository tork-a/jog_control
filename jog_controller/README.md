# Subscribed topic

- joint_states (sensor_msgs/JointState)

  Current joint states (angle, velocity, acceleration)

- jog_frame (jog_msgs/JogFrame)
  
  Jog command for target frame

- jog_joint (jog_msgs/JogJoint)
  
  Jog command for robot joints

# Required service / action

This package depends on move_group of MoveIt! to compute
kinematics. 

- /compute_ik
  
  Service to get inverse kinematics by MoveIt!

- compute_fk
  
  Service to get forward kinematics by MoveIt!
  
`jog_frame_node` uses both of the service. `jog_joing_node` uses only
`compute_ik_` service.

Jog contorller uses `PositionController` which is provided by
`ros_controllers/joint_trajectory_controller` package.
  
- follow_joint_trajecotry/goal (control_msgs/FollowJointTrajectoryAction)

  The goal joint angle to control
  
# Process

1. Calculate current pose by joint_state and compute_fk
2. Calculate target pose as current pose + msg.linear
3. Carculate target joint angle by comput_ik
4. publish joint_trajectory_controller/command

## Note

- Should we use service call (ik,fk) in loop? Can be dangerous?
- Should we call service in message callback? Or in main loop?
