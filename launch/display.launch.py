from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    urdf_path = PathJoinSubstitution([FindPackageShare('rover_sim'), 'urdf', 'model.urdf.xacro'])
    rviz_path = PathJoinSubstitution([FindPackageShare('rover_sim'), 'rviz', 'urdf.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=urdf_path),
        DeclareLaunchArgument('rvizconfig', default_value=rviz_path),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'rover_sim',
                'urdf_package_path': LaunchConfiguration('model'),
                'rviz_config': LaunchConfiguration('rvizconfig'),
                'jsp_gui': LaunchConfiguration('gui')
            }.items()
        )
    ])
