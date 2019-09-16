^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jog_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2019-09-16)
------------------
* Add `intermittent` parameter (`#31 <https://github.com/tork-a/jog_control/issues/31>`_ )
  - When this parameter true, jog motions wait for the end of previous action
  - Add i611 files
* Fixed launch files for motoman robots (`#35 <https://github.com/tork-a/jog_control/issues/35>`_ )
  - Fixed launch files for motoman robots.
* Contributors: Alexandre Francoeur, LazyEngineerToBe, Ryosuke Tajima

0.0.1 (2018-08-30)
------------------
* Prepare for releasing (#28)
* Migrating rostests from jog_controller to jog_launch (#27)
  - Add missing dependency
* Modify nextage's test
  - It expects start_position feature in fake_joint_driver
* Add tra1 tests
  - Purge tra1_bringup from dependency
* Add UR3 rostest
* Change use_rviz default false for test
* add UR3 and UR3 rostest
* Fix wrong link_name for joypad control
* Add jog_launch package (#26)
* Modify document, check it work as it saids
* Reduce launch check for the files not ready
* Move setting files to jog_launch package
* Add settings for UR and tra1
* Contributors: Ryosuke Tajima
