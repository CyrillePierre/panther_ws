from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('demo_config_directory'),
        DeclareLaunchArgument('robot_config_directory'),
        DeclareLaunchArgument('robot_namespace', default_value='panther'),
        DeclareLaunchArgument('mode', default_value='live'),
        DeclareLaunchArgument('trajectory_filename'),
    ]

    # Include the original launch, passing along the CLI-provided arguments
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('tirrex_demo') + '/launch/robot/robot_path_following.launch.py'
        ),
        launch_arguments={
            'demo_config_directory': LaunchConfiguration('demo_config_directory'),
            'robot_config_directory': LaunchConfiguration('robot_config_directory'),
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'mode': LaunchConfiguration('mode'),
            'trajectory_filename': LaunchConfiguration('trajectory_filename'),
        }.items(),
    )

    # Relay node to remap cmd_vel
    source_topic = str(LaunchConfiguration('robot_namespace')) + '/base/controller/cmd_skid_steering'
    target_topic = str(LaunchConfiguration('robot_namespace')) + '/cmd_vel'
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=[
            source_topic,  # input topic
            target_topic  # output topic
        ],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + [
            base_launch,
            cmd_vel_relay,
        ]
    )