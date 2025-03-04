import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5e_workcell", package_name="ur5e_cell_moveit_config")
        .robot_description(file_path="config/ur5e_workcell.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("ur5e_cell_advanced_pick_n_place")
            + "/config/moveitcpp.yaml"
        )
        .to_moveit_configs()
    )

    point_to_point_demo = Node(
        package="ur5e_cell_advanced_pick_n_place",
        executable="point_to_point_demo",
        name="point_to_point_task",
        output="screen",
        # prefix=["gdb -ex run --args"],
        # prefix=["xterm -e gdb -ex run --args"],
        parameters=[
            os.path.join(
                get_package_share_directory("ur5e_cell_advanced_pick_n_place"),
                "config",
                "point_to_point_ur5e.yaml",
            ),
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([point_to_point_demo])
