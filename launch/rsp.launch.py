from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import xacro, os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    urdf_package = LaunchConfiguration('urdf_package').perform(context)
    urdf_package_path = LaunchConfiguration('urdf_package_path').perform(context)

    pkg_path = get_package_share_directory(urdf_package)
    xacro_file = os.path.join(pkg_path, urdf_package_path)
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_package', default_value='rover_sim'),
        DeclareLaunchArgument('urdf_package_path', default_value='urdf/model.urdf.xacro'),
        OpaqueFunction(function=launch_setup)  # Burada path'i çözüyor
    ])
