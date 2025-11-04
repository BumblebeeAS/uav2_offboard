#!/usr/bin/env python
import numpy as np
import rclpy
from bb_uav_msgs.action import GoToPosition, Takeoff
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class GoToPositionActionServer(Node):
    """Action server for navigating to a target position with PX4 offboard control"""

    def __init__(self):
        super().__init__("go_to_position_action_server")

        # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Parameters
        vehicle_status_topic = (
            self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status")
            .get_parameter_value()
            .string_value
        )
        vehicle_local_position_topic = (
            self.declare_parameter(
                "vehicle_local_position_topic", "/fmu/out/vehicle_local_position"
            )
            .get_parameter_value()
            .string_value
        )
        offboard_control_mode_topic = (
            self.declare_parameter(
                "offboard_control_mode_topic", "/fmu/in/offboard_control_mode"
            )
            .get_parameter_value()
            .string_value
        )
        trajectory_setpoint_topic = (
            self.declare_parameter(
                "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint"
            )
            .get_parameter_value()
            .string_value
        )
        vehicle_command_topic = (
            self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
            .get_parameter_value()
            .string_value
        )

        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            vehicle_status_topic,
            self.vehicle_status_callback,
            qos_profile_sub,
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self.local_position_callback,
            qos_profile_sub,
        )

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, offboard_control_mode_topic, qos_profile_pub
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, trajectory_setpoint_topic, qos_profile_pub
        )
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand, vehicle_command_topic, qos_profile_pub
        )

        # Control timer
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(
            timer_period, self.control_loop_callback, callback_group=self.callback_group
        )
        self.dt = timer_period

        # Vehicle state
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.position_valid = False

        # Action server
        self._action_server = ActionServer(
            self,
            GoToPosition,
            "go_to_position",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # Takeoff action server
        self._takeoff_action_server = ActionServer(
            self,
            Takeoff,
            "takeoff",
            execute_callback=self.execute_takeoff_callback,
            goal_callback=self.takeoff_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # Goal tracking
        self.current_goal = None
        self.goal_handle = None
        self.start_time = None
        self.absolute_target = None  # Stores the computed absolute target position

        self.get_logger().info("GoToPosition action server started")

    # -------------------- Vehicle Command Helpers --------------------
    def publish_vehicle_command(self, command, **params) -> None:
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
        self.publisher_vehicle_command.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent....")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Switching to offboard mode....")

    def vehicle_status_callback(self, msg):
        """Update vehicle navigation and arming state"""
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def local_position_callback(self, msg):
        """Update current position from vehicle"""
        self.current_position = np.array([msg.x, msg.y, msg.z])
        self.position_valid = True

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        mode = "relative" if goal_request.relative else "absolute"
        self.get_logger().info(
            f"Received goal request ({mode}): target=({goal_request.x}, {goal_request.y}, {goal_request.z})"
        )

        # Validate thresholds
        if (
            goal_request.x_threshold <= 0
            or goal_request.y_threshold <= 0
            or goal_request.z_threshold <= 0
        ):
            self.get_logger().warn("Invalid thresholds, must be positive")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def takeoff_goal_callback(self, goal_request):
        """Validate Takeoff goal"""
        self.get_logger().info(
            f"Received takeoff request: altitude={goal_request.altitude:.2f} m"
        )

        if goal_request.altitude <= 0:
            self.get_logger().warn("Invalid altitude: must be positive meters")
            return GoalResponse.REJECT

        # Optional: reject if another goal in progress
        if self.current_goal is not None:
            self.get_logger().warn("Another goal is in progress; rejecting takeoff")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    async def execute_takeoff_callback(self, goal_handle):
        self.get_logger().info("Executing takeoff goal...")

        start_time = self.get_clock().now()
        feedback = Takeoff.Feedback()

        if not self.position_valid:
            self.get_logger().error("Failed to get valid position data")
            goal_handle.abort()
            result = Takeoff.Result()
            result.success = False
            result.message = "Failed to get valid position data"
            return result

        self.current_goal = goal_handle.request

        # Ensure offboard mode and arm
        self.engage_offboard_mode()
        self.arm()

        # Compute absolute target: climb up 'altitude' meters (NED: z negative up)
        altitude = float(goal_handle.request.altitude)
        target = self.current_position.copy()
        target[2] = self.current_position[2] - altitude
        self.absolute_target = target

        rate = self.create_rate(20)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.current_goal = None
                self.absolute_target = None
                self.destroy_rate(rate)
                result = Takeoff.Result()
                result.success = False
                result.final_z = float(self.current_position[2])
                result.time_elapsed = (
                    self.get_clock().now() - start_time
                ).nanoseconds / 1e9
                result.message = "Takeoff canceled"
                self.get_logger().info(result.message)
                return result

            # Errors
            distance_vector = self.absolute_target - self.current_position
            x_error = abs(distance_vector[0])
            y_error = abs(distance_vector[1])
            z_error = abs(distance_vector[2])

            # Feedback
            feedback.current_z = float(self.current_position[2])
            feedback.altitude_to_goal = float(
                altitude - (self.current_position[2] - target[2])
            )
            goal_handle.publish_feedback(feedback)

            # Completion check
            if (
                x_error <= self.current_goal.x_threshold
                and y_error <= self.current_goal.y_threshold
                and z_error <= self.current_goal.z_threshold
            ):
                goal_handle.succeed()
                self.current_goal = None
                self.absolute_target = None
                self.destroy_rate(rate)
                result = Takeoff.Result()
                result.success = True
                result.final_x = float(self.current_position[0])
                result.final_y = float(self.current_position[1])
                result.final_z = float(self.current_position[2])
                result.time_elapsed = (
                    self.get_clock().now() - start_time
                ).nanoseconds / 1e9
                result.message = "Takeoff reached target altitude"
                self.get_logger().info(result.message)
                return result

            rate.sleep()

    async def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info("Executing goal...")

        self.current_goal = goal_handle.request
        self.start_time = self.get_clock().now()

        feedback_msg = GoToPosition.Feedback()

        if not self.position_valid:
            self.get_logger().error("Failed to get valid position data")
            goal_handle.abort()
            result = GoToPosition.Result()
            result.success = False
            result.message = "Failed to get valid position data"
            return result

        self.goal_handle = goal_handle

        # Compute absolute target position
        if self.current_goal.relative:
            # Relative mode: add offsets to current position
            self.absolute_target = self.current_position + np.array(
                [self.current_goal.x, self.current_goal.y, self.current_goal.z]
            )
            self.get_logger().info(
                f"Relative mode: current=({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}), "
                f"offset=({self.current_goal.x:.2f}, {self.current_goal.y:.2f}, {self.current_goal.z:.2f}), "
                f"absolute_target=({self.absolute_target[0]:.2f}, {self.absolute_target[1]:.2f}, {self.absolute_target[2]:.2f})"
            )
        else:
            # Absolute mode: use goal coordinates directly
            self.absolute_target = np.array(
                [self.current_goal.x, self.current_goal.y, self.current_goal.z]
            )
            self.get_logger().info(
                f"Absolute mode: target=({self.absolute_target[0]:.2f}, {self.absolute_target[1]:.2f}, {self.absolute_target[2]:.2f})"
            )

        rate = self.create_rate(20)

        while rclpy.ok():
            # Check if goal is canceling
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = GoToPosition.Result()
                result.success = False
                result.final_x = self.current_position[0]
                result.final_y = self.current_position[1]
                result.final_z = self.current_position[2]
                result.time_elapsed = (
                    self.get_clock().now() - self.start_time
                ).nanoseconds / 1e9
                result.message = "Goal canceled"
                self.get_logger().info("Goal canceled")
                self.current_goal = None
                self.goal_handle = None
                self.absolute_target = None
                self.destroy_rate(rate)
                return result

            # Calculate distance to goal
            distance_vector = self.absolute_target - self.current_position
            distance_to_goal = np.linalg.norm(distance_vector)

            # Calculate individual axis errors
            x_error = abs(distance_vector[0])
            y_error = abs(distance_vector[1])
            z_error = abs(distance_vector[2])

            # Calculate time elapsed
            time_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            # Publish feedback
            feedback_msg.current_x = self.current_position[0]
            feedback_msg.current_y = self.current_position[1]
            feedback_msg.current_z = self.current_position[2]
            feedback_msg.distance_to_goal = float(distance_to_goal)
            goal_handle.publish_feedback(feedback_msg)

            # Check if goal is reached
            if (
                x_error <= self.current_goal.x_threshold
                and y_error <= self.current_goal.y_threshold
                and z_error <= self.current_goal.z_threshold
            ):

                goal_handle.succeed()
                result = GoToPosition.Result()
                result.success = True
                result.final_x = self.current_position[0]
                result.final_y = self.current_position[1]
                result.final_z = self.current_position[2]
                result.time_elapsed = time_elapsed
                result.message = "Goal reached successfully"
                self.get_logger().info(
                    f"Goal reached! Final position: ({result.final_x:.2f}, {result.final_y:.2f}, {result.final_z:.2f})"
                )
                self.current_goal = None
                self.goal_handle = None
                self.absolute_target = None
                self.destroy_rate(rate)
                return result

            # Continue commanding position in control_loop_callback
            rate.sleep()

    def control_loop_callback(self):
        """High-rate control loop for publishing offboard commands"""
        # Always publish offboard control mode to keep the connection alive
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # Only publish trajectory if we have an active goal and vehicle is in offboard mode
        if (
            self.current_goal is not None
            and self.absolute_target is not None
            and (
                self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            )
        ):
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = self.absolute_target[0]
            trajectory_msg.position[1] = self.absolute_target[1]
            trajectory_msg.position[2] = self.absolute_target[2]
            trajectory_msg.yaw = float("nan")  # Let PX4 handle yaw
            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    action_server = GoToPositionActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
