from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_dir = LaunchConfiguration(
        'map',
        default=PathJoinSubstitution([FindPackageShare('rover_sim'), "maps", "slam_map.yaml"])
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([FindPackageShare('rover_sim'), "config", "nav2_config.yaml"])
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", 'bringup_launch.py'])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package="topic_tools",
            executable="relay",
            arguments=["/rover/scan", "/scan"]
        ),
        Node(
            package="topic_tools",
            executable="relay",
            arguments=["/cmd_vel", "/rover/cmd_vel"]
        ),
        Node(
            package="topic_tools",
            executable="relay",
            arguments=["/rover/odom", "/odom"]
        )
    ])
