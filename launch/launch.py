from pathlib import Path
from typing import List

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package="jellyfish2_offboard",
        executable="offboard_node",
        name="offboard_node",
        # DEBUG topic names are not set yet, need to look at terminal
        parameters=[
            {
                "vehicle_status_topic": "/fmu/out/vehicle_status",
                "offboard_heartbeat_topic": "/fmu/in/offboard_control_mode",
                "takeoff_status_topic": "/fmu/out/takeoff_status",
                "vehicle_command_topic": "/fmu/in/vehicle_command",
                "traj_setpoint_topic": "/fmu/in/trajectory_setpoint",
                "local_pos_topic": "/fmu/out/vehicle_local_position",
                "go_to_setpoint_topic": "/fmu/in/goto_setpoint",
            }
        ],
    )

    ld.add_action(node)
    return ld
