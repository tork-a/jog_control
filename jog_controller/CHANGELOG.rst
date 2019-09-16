^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jog_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2019-09-16)
------------------
* Add `intermittent` parameter (`#31 <https://github.com/tork-a/jog_control/issues/31>`_ )
  - When this parameter is true, jog motions wait for the end of previous action
* Implemented a twist_to_jogframe controller (`#39 <https://github.com/tork-a/jog_control/issues/39>`_ )
  - added optional arg to lauch the spacenav
  - added missing dependency in package.
  - Added Twist to jog frame missing launch file arg
  - Update README.md
  - Fix typos, renamed argument. Mod def scale values
  - Asked modifications for pr.
  - New method to tf from spacenav axis to EEF.
  - Added missing launch file arg to ReadMe
  - clean up of twist script
  - fix to make it work
  - tested joy control using space mouse
  - Change validation message as ROS_ERROR
* Contributors: Alexandre Francoeur, LazyEngineerToBe, Ryosuke Tajima

0.0.1 (2018-08-30)
------------------
* Prepare for releasing (#28)
  - Add further dpendency
* Migrating rostests from jog_controller to jog_launch (#27)
  - Add waitForTransform for test code
  - make it optional to specify controller_name (should be remove later)
  - Remove unused variable
  - add UR3 and UR3 rostest
  - Adjust joypad scale for more safer control
* Add jog_launch package (#26)
  - Remove running tests(transfer to jog_launch afterwards)
  - Add settings for UR and tra1
* Add joypad descriptions (#24)
* Fix readme (#21)
  - Add how-to description to jog_controller/README.md
  - Add description into tra1_jog.yaml
* Add ur5 tests (#14)
  - Expand queue size for solve test failure
  - Add frame jog test for UR5
  - Add test_jog_frame.py to test jog frame
  - Add UR5 rostest for CMakeLists.txt
  - Add UR5 gazebo test for joint_jog
  - Change to test joint jog around a home position
  - Change to handle the jog command with partial joint names
  - Add initial test file for UR5
* Add config and launch files for hironx (#10)
  - Set default use_joy argument false
* Support precise jog control (#13)
  - Modify vs060 test
  - Modify for precise jog_frame control
  - Add test_jog_joint.py to test precise jog
  - Checking with VS060
* Change to avoid loop in jog control
  - Update reference joint state only if the jog command is not continuous
  - Change jog panel not to publish command when the command is all zero
* Add param exclude joints (#9)
* add config and launch files for hironx.
* Add motoman_sda10f and update README
* Add baxter_fake.launch which use fake_joint_driver
* Add exclude_joint_names param to exclude joint
* Add support for multiple joint_states (#8)
* Add tests and travis (#7)
* Add test for baxter (hztest only)
  - baxter_moveit_config is not released, so it remain disabled
  - Give up TRA1 test until minas is fixed
* Add support for multiple joint_states
  - ex. Baxter produces separated JointStates for grippers
* Add dependency on nextage_moveit_config
* Add exec dependency for nextage_ros_bridge
* Remove launch check templary for travis
* Add Nextage tests (hztest only)
* Fix build dependency
* Add hztest for TRA1
* Fix to obtain correct cinfo for jog_frame_node
* Fix to get first joint state message
* Fix the joy_to_jog_frame node to work
* Fix bug of invalid action name for use_action
  - Baxter will get error because this bug
* Add orientation jog (#5)
  - Improve display panel
  - Add rotation jog
* Change the naming of JogJoint and JogFrame messages (#1)
  - Change jog_msgs member name
* Add baxter settings, improve joint jog (#3)
* Improve joint jog node
  - Support multiple controllers
  - We just specify the list of joint to jog
  - Fix Baxter settings
* Remove controller_name from joint jog setting
* Add baxter settings
* Add action option (#2)
* Add motoman_sia20d jog settings
* Change abb_irb2400 jog settings
* Add time_from_start parameter
* Add use_action option for jog joint node
* Change panel timer to 10Hz
  - To avoid oscilation
  - It may change the response
* Add abb_irb2400 settings
* Add use_action option
* Changes for jog(frame and joint) controllers
  - Organize parameters
  - Check to invalid IK solution
* Add settings for NEXTAGE Open
* Add settings for DENSO VS060
* Modify settings for MOTOMAN sia5d
* Modify setting for UR5 gazebo
* Modify setting for MELFA rv7fl
* Modify setting for TRA1 manipulator
* jog_frame_panel: move_group from list in parameter groups
* Add motoman_sia5d example
* Add jog_joint_panel rviz plugin
* Add documents, quickstart launch files
* Add jog_frame rviz panel plugin
* Add prototype of jog_joint_node
* Add jog_controller/jog_frame_node proto
* Contributors: Ryosuke Tajima, aislab
