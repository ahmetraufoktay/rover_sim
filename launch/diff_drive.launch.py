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
                                      default_value='urdf/model.urdf.xacro')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_worlds/empty.world',
        description='Optional world file to load in Gazebo'
    )

    return LaunchDescription([
        use_sim_time_arg,
        package_arg,
        model_arg,
        world_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("rover_sim"), "launch", "gazebo.launch.py"])),
            launch_arguments={
                "use_sim_time" : LaunchConfiguration('use_sim_time'),
                "urdf_package" : LaunchConfiguration('urdf_package'),
                "urdf_package_path" : LaunchConfiguration('urdf_package_path'),
                "world" : LaunchConfiguration('world'),
            }.items()
        ),
        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            remappings=[
                ('/cmd_vel', '/rover/cmd_vel')
            ]
        )
    ])
