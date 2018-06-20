# Overview

[ROS](http://www.ros.org) and [MoveIt!](http://moveit.ros.org) are
very powerful tools for industrial manipulators. Many people use ROS
packages to control their own industrial manipulators. You can see
many manipulators are available on
[ROS-Industrial project](https://rosindustrial.org).

However, ROS has several missing features for industrial usage. The
one is "jogging". Jogging is to make the actual robot move by small
amount of distance. We can repeat jogging to adjust the robot to teach
target position and posture.

Most commercial industrial robots have their own jog control in the
teaching pendants. Some ROS oriented robot has no teaching pendant and
no jog control, so it can be a big barrier to use ROS for industrial
usage.

This `jog_control` repositry has packages for jog control
(reasonably). You can jog your robot by rviz jog panel, joypads,
keyboards, and teaching pendants using these packages.

# Quick start

You can see the idea of jog_control package by demo with simulation
and MoveIt!. Some robots are from
[ROS-Industrial repositry](https://github.com/ros-industrial), which
you need to build from source code.

## UR5 

![UR5 jog control](image/ur5_jog.png)

Launch simulation and MoveIt!

```
$ roslaunch ur_gazebo ur5_joint_limited.launch
$ roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
$ roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

And launch jog nodes.

```
$ roslaunch jog_controller ur5.launch
```

## TRA1

![TRA1 jog control](image/tra1_jog.png)

Launch simulation and MoveIt!

```
$ roslaunch tra1_bringup tra1_bringup.launch simulation:=true
$ roslaunch tra1_bringup tra1_moveit.launch 
```

And launch jog nodes.

```
$ roslaunch jog_controller tra1.launch
```

## Denso VS060

![Denso VS060 jog control](image/vs060_jog.png)

Launch simulation and MoveIt!

```
$ roslaunch denso_launch denso_vs060_moveit_demo_simulation.launch 
```

And launch jog nodes.

```
$ roslaunch jog_controller vs060.launch
```

## NEXTAGE Open

![NEXTAGE Open jog control](image/nextage_jog.png)

Launch simulation and MoveIt!

```
$ rtmlaunch nextage_ros_bridge nextage_ros_bridge_simulation.launch
$ roslaunch nextage_moveit_config moveit_planning_execution.launch 
```

And launch jog nodes.

```
$ roslaunch jog_controller nextage.launch
```

## ABB IRB2400

![ABB IRB2400 jog control](image/abb_irb2400_jog.png)

(CAUTION: The model of this robot is a bit strange in joint limits.
The jog may not move because of joints flipping.)

Launch simulation and MoveIt!

```
$ roslaunch abb_irb2400_moveit_config moveit_planning_execution.launch 
```

And launch jog nodes.

```
$ roslaunch jog_controller abb_irb2400.launch 
```

## MOTOMAN SIA20D

![MOTOMAN SIA20D jog control](image/motoman_sia20d_jog.png)

Launch simulation and MoveIt!

```
$ roslaunch motoman_sia20d_moveit_config moveit_planning_execution.launch sim:=true
```

And launch jog nodes.

```
$ roslaunch jog_controller motoman_sia20d.launch
```

## MOTOMAN SDA10F

![MOTOMAN SDA10F jog control](image/motoman_sda10f_jog.png)

This is another dual arm robot by Yaskawa MOTOMAN.

Launch simulation and MoveIt!

```
$ roslaunch motoman_sda10f_moveit_config moveit_planning_execution.launch sim:=true
```

And launch jog nodes.

```
$ roslaunch jog_controller motoman_sda10f.launch 
```

## rviz JogFramePanel Pugin

You ca add new panel JogFramePanel in rviz. 

## Joypad control

You can also use a joypad.
TBA.

## Teaching pendant

You can also use a teaching pendant.
TBA.

# Packages

## [jog_msgs](jog_msgs/README.md)

`jog_msgs` is a ROS message package for jog control.

## [jog_controller](jog_controller/README.md)

`jog_controller` contains ROS nodes for jog control.

# TODO

- For precise jog and stability, joint position should updated only when the jog start
- Wiser target picking (group name, target link, etc)
- Marker visualization for target link and base link

