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
        arguments=["-0.0176314", "-0.0623444", "-0.0165192", "0.517004", "-0.501707", "0.496051","0.484696", "tool0", "camera_link"]
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
