from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true")

    world_path = PathJoinSubstitution([FindPackageShare('rover_sim'), "worlds", "slam.world"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rover_sim"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world" : world_path,
            "urdf_package_path" : "urdf/camera.urdf.xacro",
            "use_sim_time" : LaunchConfiguration("use_sim_time")
        }.items()
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "frame_id" : "camera_frame_link",
                "use_sim_time" : LaunchConfiguration("use_sim_time"),
                "approx_sync" : True,
            }
        ],
        remappings=[
            ("/odom", "/rover/odom"),
            ("/rgb/image", "/rover/camera/image_raw"),
            ("/depth/image", "/rover/camera/depth/image_raw"),
            ("/rgb/camera_info", "/rover/camera/camera_info"),
        ]
    )
    return LaunchDescription([
        use_sim_time,
        gazebo,
        rtabmap
    ])
