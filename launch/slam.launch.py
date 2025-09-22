import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg_path = get_package_share_directory("rover_sim")
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='rover_sim')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/camera.urdf.xacro')
    
    slam_param_path = PathJoinSubstitution([FindPackageShare('rover_sim'), "config", "mapper_params_online_async.yaml"])

    slam_params_arg = DeclareLaunchArgument('slam_params_file',
                                             description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
                                             default_value=slam_param_path)

    map_path = os.path.join(pkg_path, "maps", "slam_map_serial")

    world_path = PathJoinSubstitution([FindPackageShare('rover_sim'), "worlds", "slam.world"])

    world_arg = DeclareLaunchArgument("world",
                                      description="Full path to the gazebo world",
                                      default_value=world_path)

    rviz_config_arg = DeclareLaunchArgument("rviz_config_name", 
                                            description="Path to rviz config file",
                                            default_value="slam.rviz")
    rviz_config_path = PathJoinSubstitution([FindPackageShare("rover_sim"), "rviz", LaunchConfiguration("rviz_config_name")])

    return LaunchDescription([
        use_sim_time_arg,
        package_arg,
        model_arg,
        slam_params_arg,
        rviz_config_arg,
        world_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("rover_sim"), "launch", "diff_drive.launch.py"])),
            launch_arguments={
                "use_sim_time" : LaunchConfiguration('use_sim_time'),
                "urdf_package" : LaunchConfiguration('urdf_package'),
                "urdf_package_path" : LaunchConfiguration('urdf_package_path'),
                "world" : LaunchConfiguration('world')
            }.items()
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time' : LaunchConfiguration("use_sim_time")},
                {"map_file_name": map_path}
            ]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_node",
            output="screen",
            arguments=["-d", rviz_config_path]
        )
    ])
