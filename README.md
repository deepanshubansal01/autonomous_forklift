# forklift_noovelia

Contains ros packages for forklift project.

## Gazebo simulation environment:
NOTE: this should now be encoded directly into the devel/setup.bash script.

Update the GAZEBO_MODEL_PATH variable, add the following to your ~/.bashrc file:

```
export GAZEBO_MODEL_PATH=/home/<YOUR USER NAME>/catkin_ws/src/forklift_robot/gazebo_noovelia/gazebo_models_noovelia/models:$GAZEBO_MODEL_PATH
```
This is needed in order to be able to load the different environments (otherwise you get "no namespace found" messages and an empty world

## Control in Gazebo:

```
1. roslaunch forklift_gazebo forklift_warehouse.launch
2. rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Localization and Mapping(SLAM):

* Using gmapping(*uses particle filer*)

```
1. roslaunch forklift_mapping forklift_gmapping.launch
2. roslaunch ira_laser_tools laserscan_multi_merger.launch
```

* Using slam_toolbox(*uses graph based slam*)

```
1. roslaunch forklift_mapping forklift_slam_toolbox.launch
2. roslaunch ira_laser_tools laserscan_multi_merger.launch
```

The laserscan_multi_merger.launch publishes on **/scan_multi** topic after merging data from three lasers. The /scan_multi  is subscribed by mapping node.
