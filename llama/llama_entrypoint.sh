#!/bin/bash
cd llama_ws/
colcon build
source /opt/ros/humble/setup.bash 
source install/local_setup.bash
cd ..
ros2 launch language language_launch.py
