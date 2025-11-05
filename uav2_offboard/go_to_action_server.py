#!/usr/bin/env python
from typing import Callable

import numpy as np
import rclpy
from bb_uav_msgs.action import GoToPosition, Takeoff
from bb_uav_msgs.msg import GoToFeedback, GoToResult
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
from uav2_offboard.utils.goto import GeneralGoal


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

        # Continuously publish heartbeat and traj setpoint.
        # PX4 requires that the vehicle is already receiving
        # OffboardControlMode messages before it will arm in offboard mode,
        # or before it will switch to offboard mode when flying
        self.timer_ = self.create_timer(
            0.02, self.control_loop_callback, callback_group=self.callback_group
        )

        # Vehicle state
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.position_valid = False

        # Action servers
        self._goto_position_action_server = ActionServer(
            self,
            GoToPosition,
            "go_to_position",
            execute_callback=self.execute_goto_position_callback,
            goal_callback=self.goto_position_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
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

    def vehicle_status_callback(self, msg: VehicleStatus):
        """Update vehicle navigation and arming state"""
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def local_position_callback(self, msg: VehicleLocalPosition):
        """Update current position from vehicle"""
        self.current_position = np.array([msg.x, msg.y, msg.z])
        self.position_valid = True

    def control_loop_callback(self):
        """High-rate control loop for publishing offboard commands"""
        # Always publish offboard control mode to keep the connection alive
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

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

    # -------------------- Action Server Callbacks --------------------

    def validate_goal(self, goal: GeneralGoal) -> bool:
        if goal.x_threshold <= 0 or goal.y_threshold <= 0 or goal.z_threshold <= 0:
            self.get_logger().warn("Invalid thresholds, must be positive")
            return False

        if self.current_goal is not None:
            self.get_logger().warn("Another goal is in progress; rejecting...")
            return False

        return True

    def goto_position_goal_callback(self, goal_request: GoToPosition.Goal):
        """Accept or reject a GoToPosition goal"""
        mode = "relative" if goal_request.relative else "absolute"
        self.get_logger().info(
            f"Received goal request ({mode}): target=({goal_request.x}, {goal_request.y}, {goal_request.z})"
        )

        is_goal_valid = self.validate_goal(GeneralGoal.from_position_goal(goal_request))
        return GoalResponse.ACCEPT if is_goal_valid else GoalResponse.REJECT

    def takeoff_goal_callback(self, goal_request: Takeoff.Goal):
        """Accept or reject a Takeoff goal"""
        self.get_logger().info(
            f"Received takeoff request: altitude={goal_request.altitude:.2f} m"
        )

        if goal_request.altitude <= 0:
            self.get_logger().warn("Invalid altitude: must be positive meters")
            return GoalResponse.REJECT

        is_valid_goal = self.validate_goal(GeneralGoal.from_takeoff_goal(goal_request))
        return GoalResponse.ACCEPT if is_valid_goal else GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(
        self,
        goal_handle,
        goal: GeneralGoal,
        publish_feedback_fn: Callable[[GoToFeedback], None],
    ) -> GoToResult:
        """Execute the goal"""
        self.current_goal = goal
        self.start_time = self.get_clock().now()

        if not self.position_valid:
            self.get_logger().error("Failed to get valid position data")
            goal_handle.abort()
            result = GoToResult()
            result.success = False
            result.message = "Failed to get valid position data"
            return result

        self.goal_handle = goal_handle

        # Compute absolute target position from relative or absolute goal
        if self.current_goal.relative:
            self.absolute_target = self.current_position + np.array(
                [self.current_goal.x, self.current_goal.y, self.current_goal.z]
            )
            self.get_logger().info(
                f"Relative mode: current=({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}), "
                f"offset=({self.current_goal.x:.2f}, {self.current_goal.y:.2f}, {self.current_goal.z:.2f}), "
                f"absolute_target=({self.absolute_target[0]:.2f}, {self.absolute_target[1]:.2f}, {self.absolute_target[2]:.2f})"
            )
        else:
            self.absolute_target = np.array(
                [self.current_goal.x, self.current_goal.y, self.current_goal.z]
            )
            self.get_logger().info(
                f"Absolute mode: target=({self.absolute_target[0]:.2f}, {self.absolute_target[1]:.2f}, {self.absolute_target[2]:.2f})"
            )

        rate = self.create_rate(20)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = GoToResult()
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
            feedback_msg = GoToFeedback()
            feedback_msg.current_x = self.current_position[0]
            feedback_msg.current_y = self.current_position[1]
            feedback_msg.current_z = self.current_position[2]
            feedback_msg.distance_to_goal = float(distance_to_goal)
            publish_feedback_fn(feedback_msg)

            # Check if goal is reached
            if (
                x_error <= self.current_goal.x_threshold
                and y_error <= self.current_goal.y_threshold
                and z_error <= self.current_goal.z_threshold
            ):

                goal_handle.succeed()
                result = GoToResult()
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

    async def execute_goto_position_callback(self, goal_handle) -> GoToPosition.Result:
        """Execute GoToPosition action"""
        general_goal = GeneralGoal.from_position_goal(goal_handle.request)
        publish_feedback_fn = lambda feedback: goal_handle.publish_feedback(
            GoToPosition.Feedback(feedback=feedback)
        )
        goto_result = await self.execute_callback(
            goal_handle, general_goal, publish_feedback_fn=publish_feedback_fn
        )
        return GoToPosition.Result(result=goto_result)

    async def execute_takeoff_callback(self, goal_handle) -> Takeoff.Result:
        """Execute Takeoff action"""
        self.arm()
        self.engage_offboard_mode()
        general_goal = GeneralGoal.from_takeoff_goal(goal_handle.request)
        publish_feedback_fn = lambda feedback: goal_handle.publish_feedback(
            Takeoff.Feedback(feedback=feedback)
        )
        goto_result = await self.execute_callback(
            goal_handle, general_goal, publish_feedback_fn=publish_feedback_fn
        )
        return Takeoff.Result(result=goto_result)


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
