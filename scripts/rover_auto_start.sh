#!/bin/bash
BASH_ENV="$HOME/.profile" 

pkill ros2
pkill stream_compress
pkill rgb_node
pkill rover_control
pkill rplidar_composi
pkill MicroXRCEAgent

cd "$(dirname "$0")";
CWD="$(pwd)"
echo $CWD

#colcon build
source /opt/ros/galactic/setup.bash
source install/setup.bash

ros2 launch ark_rover jetson.launch.py
