^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_proc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2021-01-26)
------------------
* Use C-style strings for RCLCPP\_ macros. (`#12 <https://github.com/ros-perception/laser_proc/issues/12>`_)
* Contributors: Chris Lalancette

1.0.1 (2020-03-24)
------------------
* Make sure to add necessary build dependencies. (`#9 <https://github.com/ros-perception/laser_proc/issues/9>`_)
* Contributors: Chris Lalancette

1.0.0 (2020-03-17)
------------------
* export symbols in laser_proc component (`#8 <https://github.com/ros-perception/laser_proc/issues/8>`_)
* Make the laser_proc library shared. (`#7 <https://github.com/ros-perception/laser_proc/issues/7>`_)
* Ros2 devel (`#6 <https://github.com/ros-perception/laser_proc/issues/6>`_)
* Contributors: Chris Lalancette, Karsten Knese, Brett, Gu, Chao Jie, Marc-Antoine Testier

0.1.5 (2018-08-09)
------------------
* update to use non deprecated pluginlib macro
* Contributors: Jonathan Binney, Mikael Arguedas

0.1.4 (2014-03-30)
------------------
* Install fix for Android.
* Contributors: Chad Rockey

0.1.3 (2013-08-21)
------------------
* No more Willow Garage email.
* Contributors: Chad Rockey

0.1.2 (2013-03-14)
------------------
* Uncaught except when subscribing to most_intense on non-intensity scans.
* Updated function to be const.
* Contributors: chadrockey

0.1.1 (2013-03-08)
------------------
* Merge pull request `#1 <https://github.com/ros-perception/laser_proc/issues/1>`_ from ros-perception/fix_pluginlib_depend
  Add missing dependency on pluginlib
* Add missing dependency on pluginlib
* Contributors: Chad Rockey, William Woodall

0.1.0 (2013-03-04)
------------------
* Minor fixes.
* Added ROS nodelet/node.
* Adjusted advertiseLaser to be more standard.  Fixed seg fault with first and laser publishers.
* Preparing to test with urg_node
* Initial commit
* Contributors: Chad Rockey, chadrockey
