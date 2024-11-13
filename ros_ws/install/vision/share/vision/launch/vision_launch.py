import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # launch_ros.actions.Node(
        #     package='vision',
        #     executable='JohnsFunciton',
        #     name='JohnsFunciton'),
        launch_ros.actions.Node(
            package='vision',
            executable='yolo11_subscriber',
            name='yolo11_subscriber'),
        launch_ros.actions.Node(
            package='vision',
            executable='depth_subscriber',
            name='depth_subscriber'),
        # launch_ros.actions.Node(
        #     package='vision',
        #     executable='image_subscriber',
        #     name='image_subscriber')
  ])