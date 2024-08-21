#!/usr/bin/env python3
import numpy as np
import rclpy
from px4_msgs.msg import (
    GotoSetpoint,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class OffboardNode(Node):
    def __init__(self):
        super().__init__("offboard_node")
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.declare_parameter("vehicle_status_topic", "")
        self.declare_parameter("offboard_heartbeat_topic", "")
        self.declare_parameter("vehicle_command_topic", "")
        self.declare_parameter("traj_setpoint_topic", "")
        self.declare_parameter("local_pos_topic", "")
        self.declare_parameter("go_to_setpoint_topic", "")
        self.vehicle_status_topic = (
            self.get_parameter("vehicle_status_topic")
            .get_parameter_value()
            .string_value
        )
        self.offboard_heartbeat_topic = (
            self.get_parameter("offboard_heartbeat_topic")
            .get_parameter_value()
            .string_value
        )
        self.vehicle_command_topic = (
            self.get_parameter("vehicle_command_topic")
            .get_parameter_value()
            .string_value
        )
        self.traj_setpoint_topic = (
            self.get_parameter("traj_setpoint_topic").get_parameter_value().string_value
        )
        self.local_pos_topic = (
            self.get_parameter("local_pos_topic").get_parameter_value().string_value
        )
        self.go_to_topic = (
            self.get_parameter("go_to_setpoint_topic")
            .get_parameter_value()
            .string_value
        )
        self.vehicle_status_sub_ = self.create_subscription(
            VehicleStatus,
            self.vehicle_status_topic,
            self.vehicle_status_callback,
            self.qos_profile,
        )
        self.offboard_heartbeat_pub_ = self.create_publisher(
            OffboardControlMode, self.offboard_heartbeat_topic, self.qos_profile
        )
        self.vehicle_command_pub_ = self.create_publisher(
            VehicleCommand, self.vehicle_command_topic, self.qos_profile
        )
        self.traj_setpoint_pub_ = self.create_publisher(
            TrajectorySetpoint, self.traj_setpoint_topic, self.qos_profile
        )
        self.local_pos_sub_ = self.create_subscription(
            VehicleLocalPosition,
            self.local_pos_topic,
            self.local_pos_callback,
            self.qos_profile,
        )
        self.offboard_setpoint_counter_ = 0
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """
        Continuously publish heartbeat and traj setpoint.
        PX4 requires that the vehicle is already receiving
        OffboardControlMode messages before it will arm in offboard mode,
        or before it will switch to offboard mode when flying
        """
        if self.offboard_setpoint_counter_ == 50:
            # self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter_ < 300:
            position = [0.0, 0.0, -10.0]
            self.publish_go_to_setpoint(
                position=position, vert_speed=1.0, hori_speed=4.0
            )

        if (
            self.offboard_setpoint_counter_ > 300
            and self.offboard_setpoint_counter_ < 600
        ):
            position = [5.0, 5.0, -10.0]
            self.publish_go_to_setpoint(
                position=position, vert_speed=1.0, hori_speed=4.0
            )

        if self.offboard_setpoint_counter_ >= 600:
            self.engage_land_mode()

        self.publish_offboard_heartbeat()
        self.offboard_setpoint_counter_ += 1

    def publish_go_to_setpoint(self, position, vert_speed: float, hori_speed: float):
        msg = GotoSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = position
        msg.flag_set_max_vertical_speed = True
        msg.flag_set_max_horizontal_speed = True
        msg.max_vertical_speed = vert_speed
        msg.max_horizontal_speed = hori_speed
        self.get_logger().info(
            f"Publishing setpoint {msg.position} with max vert: {msg.max_vertical_speed} max hori: {msg.max_horizontal_speed}"
        )

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.offboard_status = msg.nav_state == 14
        self.get_logger().info(
            f"Vehicle Status Timestamp: {msg.timestamp} Offboard status(14):{msg.nav_state}"
        )

    def local_pos_callback(self, msg: VehicleLocalPosition):
        self.home_lat = msg.ref_lat
        self.home_lon = msg.ref_lon
        self.home_alt = msg._ref_alt
        # self.get_logger().info(
        #     f"Local pos Timestamp: {msg.timestamp} ref:{[msg.ref_lat, msg.ref_lon, msg.ref_alt]}"
        # )

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub_.publish(msg)

    def arm(self):
        """Arm drone. Param1=1.0 for arm."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent....")

    def disarm(self):
        """Disarm drone. Param1=0.0 for disarm."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent....")

    def publish_offboard_heartbeat(self):
        """Publish offboard heartbeat."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_heartbeat_pub_.publish(msg)
        self.get_logger().info("Publishing offboard heartbeat....")

    def engage_offboard_mode(self):
        """Switch mode to offboard mode"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Switching to offboard mode....")

    def engage_land_mode(self):
        """
        Switch mode to landmode with opportunistic landing.
        Precision landing can be initiated as part of a mission using
        MAV_CMD_NAV_LAND with param2 set appropriately:

        0: Normal landing without using the target.
        1: Opportunistic precision landing.
        2: Required precision landing.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param2=1.0)
        self.get_logger().info("Sent Land command....")


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
