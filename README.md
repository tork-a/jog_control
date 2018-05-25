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

## Launch MoveIt!

You can see the idea of jog_control package by demo with simulation
and MoveIt!.

You can use UR5:

```
$ roslaunch jog_controller ur5.launch
```

![UR5 jog control](image/ur5_jog.png)

or TRA1:

```
$ roslaunch jog_controller tra1.launch
```

![TRA1 jog control](image/tra1_jog.png)

or NEXTAGE Open (not working yet):

```
$ rtmlaunch nextage_ros_bridge nextage_ros_bridge_simulation.launch
$ roslaunch jog_controller nextage.launch
```

## rviz JogFramePanel Pugin

You should add new panel JogFramePanel in rviz.

## Joypad control

You can also use a joypad.
TBA.

# Packages

## [jog_msgs](jog_msgs/README.md)

`jog_msgs` is a ROS message package for jog control.

## [jog_controller](jog_controller/README.md)

`jog_controller` contains ROS nodes for jog control.

# TODO

- Orientation(pose) jogging (work in progress)
- Wiser target picking (group name, target link, etc)
- Marker visualization for target link and base link

