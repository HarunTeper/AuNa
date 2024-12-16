# TODO List

## General
- Look up every used launch file and determine if all parameters are still used in Humble
- Rewrite every launch file to have a `generate_launch_description` function and include launch description function
- Look up the launch files provided by Gazebo and determine parameters that should be set by the `gazebo_world.launch.py` launch file
- Add checks that make sure that every car is spawned with a namespace
- Use GroupActions in launch files if multiple actions are added
- Apply remappings in launch files using SetRemap
- Apply namespaces using PushROSNamespace


## Renaming
- Rename `gazebo_model` to `robot_name_publisher`
- Rename `spawn_robot_state_publisher.launch.py` to `robot_state_publisher.launch.py`
- Rename `spawn_car` to `spawn_robot`
- Rename `gazebo_pose` to `ground_truth_localization`
- Rename `simulation_pose` to `ground_truth_cam`
- Rename `localization_pose` to `localization_pose_publisher`

## Type Changes
- Change the type of `simulation_pose` to `etsi_its_msgs CAM`