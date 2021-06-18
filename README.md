# noovelia_forklift_robot

### Gazebo simulation environment
NOTE: this should now be encoded directly into the devel/setup.bash script.

Update the GAZEBO_MODEL_PATH variable, add the following to your ~/.bashrc file:

```
export GAZEBO_MODEL_PATH=/home/<YOUR USER NAME>/catkin_ws/src/noovelia_forklift_robot/gazebo_noovelia/gazebo_models_noovelia/models:$GAZEBO_MODEL_PATH
```
This is needed in order to be able to load the different environments (otherwise you get "no namespace found" messages and an empty world
