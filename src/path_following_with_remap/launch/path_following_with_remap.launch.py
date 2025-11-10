from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

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

    preprocess_node = Node(
        package='path_following_with_remap',
        executable='preprocess_node',
        name='path_following_preprocess_node',
        parameters=[{
            'robot_config_directory': LaunchConfiguration('robot_config_directory'),
            'trajectory_filename': LaunchConfiguration('trajectory_filename'),
        }],
        output='screen',
    )

    start_path_follower_after_preproc = RegisterEventHandler(
        OnProcessExit(
            target_action=preprocess_node,
            on_exit=[
                LogInfo(msg='[LAUNCH] Preprocess finished, starting robot_path_following.launch.py'),
                base_launch],
        )
    )

    # Relay node to remap cmd_vel
    source_topic = '/panther/path_following/cmd_vel'
    target_topic = '/panther/cmd_vel'
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
            cmd_vel_relay,
            preprocess_node,
            start_path_follower_after_preproc,
        ]
    )