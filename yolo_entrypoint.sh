cd ros_ws/
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash
cd ..
ros2 launch vision vision_launch.py &
ros2 launch carter_navigation hospital_navigation.launch.py &
wait