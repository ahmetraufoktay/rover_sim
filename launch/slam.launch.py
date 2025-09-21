from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
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

    slam_params_file = DeclareLaunchArgument('slam_params_file',
                                             description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
                                             default_value=slam_param_path)

    world_path = PathJoinSubstitution([FindPackageShare('rover_sim'), "worlds", "slam.world"])

    world_arg = DeclareLaunchArgument("world",
                                      description="Full path to the gazebo world",
                                      default_value=world_path)

    rviz_config_file = PathJoinSubstitution([FindPackageShare('rover_sim'), "rviz", "slam.rviz"])

    return LaunchDescription([
        use_sim_time_arg,
        package_arg,
        model_arg,
        slam_params_file,
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])),
            launch_arguments={
                "use_sim_time" : LaunchConfiguration('use_sim_time'),
                "slam_params_file" : LaunchConfiguration('slam_params_file')
            }.items()
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_node",
            output="screen",
            arguments=["-d", rviz_config_file]
        )
    ])
