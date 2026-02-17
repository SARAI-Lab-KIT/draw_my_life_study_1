from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    nodes_to_launch = []

    # Get argument value at runtime
    speech_recognition = LaunchConfiguration('speech_recognition').perform(context)
    vlm_model = LaunchConfiguration('vlm_model').perform(context)
    pressure_recording = LaunchConfiguration('pressure_recording').perform(context)

    if speech_recognition == "enabled":
        nodes_to_launch.append(Node(package='sarai_speech_recognition',
                                    executable='sarai_speech_recognition_node'))
        
    nodes_to_launch.append(Node(package='sarai_tts_playsound',
                                executable='sarai_tts_playsound_node'))
    
    nodes_to_launch.append(Node(package='pixelbot_vlm',
                                executable='pixelbot_vlm_node',
                                parameters=[{'vlm_model': vlm_model}]))
    
    nodes_to_launch.append(Node(package='pixelbot_display',
                                executable='pixelbot_display_node'))
    
    nodes_to_launch.append(Node(package='pixelbot_tablet',
                                executable='draw_node',
                                parameters=[{'pressure_recording': pressure_recording}]))

    return nodes_to_launch


def generate_launch_description():
    # Declare a launch argument
    speech_recognition_arg = DeclareLaunchArgument(
        'speech_recognition',
        default_value='enabled',
        description='Whether to include SR as input to the VLM or not.'
    )

    vlm_model_arg = DeclareLaunchArgument(
        'vlm_model',
        default_value='qwen3-vl:235b-instruct-cloud',
        description='Which model to use for the vlm.'
    )

    pressure_recording_arg = DeclareLaunchArgument(
        'pressure_recording',
        default_value='enabled',
        description='Whether to record the pressure or not. Can be used only if the wacom tablet is being used.'
    )

    return LaunchDescription([
        speech_recognition_arg,
        vlm_model_arg,
        pressure_recording_arg,
        OpaqueFunction(function=launch_setup)
    ])
