from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    nodes = [
        # PushRosNamespace("uav2"),
        Node(
            package="uav2_offboard",
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
                }
            ],
        ),
        Node(
            package="uav2_offboard",
            executable="go_to_position_action_server",
            name="go_to_position_action_server",
        ),
    ]

    return LaunchDescription(nodes)
