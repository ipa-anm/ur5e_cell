from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={"device_type": "d435", "publish_tf": "true", "align_depth.enable": "true"}.items(),
    )

    realsense_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.017", "-0.055", "-0.0228963", "-0.498002", "0.523098", "-0.484214","-0.493861", "tool0", "camera_link"]
    )



    ld = LaunchDescription(
        [realsense_launch, realsense_tf_node]
    )

    return ld
