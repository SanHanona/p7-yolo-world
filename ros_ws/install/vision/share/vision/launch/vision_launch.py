import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vision',
            executable='rgb_subscriber',
            name='rgb_subscriber'),
        launch_ros.actions.Node(
            package='vision',
            executable='depth_subscriber',
            name='depth_subscriber'),
        launch_ros.actions.Node(
            package='vision',
            executable='action_decision',
            name='action_decision'),
        launch_ros.actions.Node(
            package='vision',
            executable='gesture_handler',
            name='gesture_handler'),
        launch_ros.actions.Node(
            package='vision',
            executable='nav_to_pose',
            name='nav_to_pose')
  ])