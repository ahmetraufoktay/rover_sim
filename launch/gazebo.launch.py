from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )

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

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )

    rsp_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('rover_sim'), 'launch', 'rsp.launch.py']),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "urdf_package": LaunchConfiguration("urdf_package"),
            "urdf_package_path": LaunchConfiguration("urdf_package_path"),
        }.items(),
    )

    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'rover', '-z', '0.5', '-unpause'],
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        use_sim_time_arg,
        empty_world_launch,
        rsp_launch_py,
        urdf_spawner_node,
    ])
