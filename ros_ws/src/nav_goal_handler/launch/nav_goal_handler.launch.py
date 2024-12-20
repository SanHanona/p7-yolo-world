# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    map_yaml_file = LaunchConfiguration(
        "map_yaml_path",
        default=os.path.join(
            get_package_share_directory("nav_goal_handler"), "assets", "carter_warehouse_navigation.yaml"
        ),
    )

    goal_text_file = LaunchConfiguration(
        "goal_text_file_path",
        default=os.path.join(get_package_share_directory("nav_goal_handler"), "assets", "goals.txt"),
    )

    nav_goal_node = Node(
        name="set_nav_goal",
        package="nav_goal_handler",
        executable="SetNavGoal",
        parameters=[
            {
                "map_yaml_path": map_yaml_file,
                "iteration_count": 3,
                "goal_generator_type": "RandomGoalGenerator", # GoalReader
                "action_server_name": "navigate_to_pose",
                "obstacle_search_distance_in_meters": 0.2,
                "goal_text_file_path": goal_text_file,
                "initial_pose": [-6.4, -1.04, 0.0, 0.0, 0.0, 0.99, 0.02],
            }
        ],
        output="screen",
    )

    return LaunchDescription([nav_goal_node])
