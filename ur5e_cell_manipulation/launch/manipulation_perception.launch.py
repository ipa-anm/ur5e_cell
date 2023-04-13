from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    realsence_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={"device_type": "d435", "publish_tf": "false"}.items(),
    )

    realsence_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.016018", "-0.0554009", "-0.0228963", "-0.498002", "0.523098", "-0.484214","-0.493861", "tool0", "camera_link"]
    )

    aruco_marker_publisher_params = {
        'image_is_rectified': True,
        'marker_size': 0.02,
        'reference_frame': 'world',
        'camera_frame': 'camera_color_optical_frame',
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', '/camera/color/camera_info'),
                    ('/image', '/camera/color/image_raw')],
    )


    ld = LaunchDescription(
        [realsence_launch, realsence_tf_node, aruco_marker_publisher]
    )

    return ld
