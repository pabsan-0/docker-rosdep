#! /usr/bin/bash

echo 'yaml file:///catkin_ws/src/my_utils_pkg/rosdep.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list
echo 'yaml file:///catkin_ws/src/my_python_pkg/rosdep.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update
apt update
rosdep install --from-paths /catkin_ws/src/ -y

rosrun my_utils_pkg tmuxinator.yml