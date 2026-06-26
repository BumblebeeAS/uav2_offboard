import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("uav2_offboard"),
        "config",
        "params.yaml",
    )

    # Parameter file for all uav2_offboard nodes. Override per deployment, e.g. the
    # multivehicle sim passes a config with /x500/fmu/... topics and mav_sys_id: 2.
    params_file = LaunchConfiguration("params_file")

    nodes = [
        DeclareLaunchArgument("params_file", default_value=default_config),
        PushRosNamespace("uav2"),
        Node(
            package="uav2_offboard",
            executable="offboard_node",
            name="offboard_node",
            parameters=[params_file],
        ),
        Node(
            package="uav2_offboard",
            executable="landing_target_pose_node",
            name="landing_target_pose_node",
            parameters=[params_file],
        ),
        Node(
            package="uav2_offboard",
            executable="actuator_control_node",
            name="actuator_control_node",
            parameters=[params_file],
        ),
    ]

    return LaunchDescription(nodes)
