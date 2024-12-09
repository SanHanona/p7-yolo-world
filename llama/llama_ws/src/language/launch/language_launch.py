import launch
from launch.actions import ExecuteProcess
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Execute the custom process for running the model (Torch-based execution)
        ExecuteProcess(
            cmd=[  # Command to run the model
                "torchrun",  # Ensure torchrun is available in your environment
                "src/language/language/llm.py",  # Path to the model execution script
                "--ckpt_dir", "/llama/checkpoints/Llama3.2-1B"  # Checkpoint directory
            ],
            output="screen",  # Output to screen
        ),
        
        # Launch the text-to-speech (TTS) node from the language package
        launch_ros.actions.Node(
            package='language',  # Package that contains the TTS node
            executable='tts',  # TTS executable
            name='tts',  # Node name
            output='screen',  # Output to screen
            parameters=[  # Optional parameters can go here
                # Example: {"param_name": "value"}
            ]
        ),
    ])