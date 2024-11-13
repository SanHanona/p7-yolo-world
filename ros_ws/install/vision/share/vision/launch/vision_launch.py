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
            name='depth_subscriber')
  ])