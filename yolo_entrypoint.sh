#!/bin/bash
cd ros_ws/
colcon build
source install/setup.bash
ros2 launch vision vision_launch.py
