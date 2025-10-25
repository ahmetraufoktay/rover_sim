from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='rover_sim')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/camera.urdf.xacro')
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    return LaunchDescription([
        package_arg,
        model_arg,
        use_sim_time_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("rover_sim"), "launch", "gazebo.launch.py"])
            ),
            launch_arguments={
                "urdf_package" : LaunchConfiguration("urdf_package"),
                "urdf_package_path": LaunchConfiguration("urdf_package_path"),
                "world": "mars.world",
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }.items()
        )
    ])
