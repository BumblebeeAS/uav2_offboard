#!/usr/bin/env python
import numpy as np
import rclpy
from bb_uav_msgs.action import GoToPosition
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

        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_sub,
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.local_position_callback,
            qos_profile_sub,
        )

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", qos_profile_pub
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos_profile_pub
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

        # Goal tracking
        self.current_goal = None
        self.goal_handle = None
        self.start_time = None

        self.get_logger().info("GoToPosition action server started")

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
        self.get_logger().info(
            f"Received goal request: target=({goal_request.x}, {goal_request.y}, {goal_request.z})"
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

    async def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info("Executing goal...")

        self.goal_handle = goal_handle
        self.current_goal = goal_handle.request
        self.start_time = self.get_clock().now()

        feedback_msg = GoToPosition.Feedback()

        # Wait for position to be valid
        timeout_counter = 0
        while not self.position_valid and timeout_counter < 100:
            await rclpy.task.sleep(0.1)
            timeout_counter += 1

        if not self.position_valid:
            self.get_logger().error("Failed to get valid position data")
            goal_handle.abort()
            result = GoToPosition.Result()
            result.success = False
            result.message = "Failed to get valid position data"
            return result

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
                self.destroy_rate(rate)
                return result

            # Calculate distance to goal
            target_position = np.array(
                [self.current_goal.x, self.current_goal.y, self.current_goal.z]
            )
            distance_vector = target_position - self.current_position
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
        if self.current_goal is not None and (
            self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        ):
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = self.current_goal.x
            trajectory_msg.position[1] = self.current_goal.y
            trajectory_msg.position[2] = self.current_goal.z
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
