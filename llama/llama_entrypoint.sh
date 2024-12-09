#!/bin/bash
cd llama_ws/
colcon build
source install/setup.bash
ros2 launch language language_launch.py
