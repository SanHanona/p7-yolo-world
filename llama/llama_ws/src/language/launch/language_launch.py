import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.ExecuteProcess(
            cmd=[ # edit for language 
                "torchrun",
                "path/to/my_torch_script.py",
                "--arg1", "value1"
            ],
            output="screen",
        ),
        launch_ros.actions.Node(
            package='language',
            executable='llama_handler',
            name='llama_handler'),
        launch_ros.actions.Node(
            package='language',
            executable='robot_input_publisher',
            name='robot_input_publisher'),

  ])