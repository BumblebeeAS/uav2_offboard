# UAV2 Offboard

Offboard package for drone behaviors and missions.

## Quickstart

0. Ensure that **PX4 is running** and connected via XRCE-DDS

1. In a separate terminal,

```bash
ros2 launch uav2_offboard launch.py
```

2. Takeoff

```bash
ros2 action send_goal /takeoff bb_uav_msgs/action/Takeoff "{altitude: 3.0, x_threshold: 0.1, y_threshold: 0.1, z_threshold: 0.1}" --feedback
```

3. Move

```bash
ros2 action send_goal /go_to_position bb_uav_msgs/action/GoToPosition "{x: 3.0, y: 3.0, z: -2.0, relative: false, x_threshold: 0.1, y_threshold: 0.1, z_threshold: 0.1}" --feedback
```

4. Land

```bash
ros2 service call /offboard_node/land std_srvs/srv/Trigger "{}"
```

## Usage

The vehicle can be in any mode or the `Takeoff` action but it must be armed and in "Offboard" flight mode for the `GoToPosition` action.

## How It Works

1. The action server continuously publishes `OffboardControlMode` messages at 50 Hz to maintain offboard mode
2. When a goal is received, it starts commanding the target position via `TrajectorySetpoint` messages
3. It monitors the vehicle's current position from `VehicleLocalPosition` messages
4. Feedback is published showing current position, distance to goal, and time remaining
5. The action succeeds when all position errors are within their respective thresholds
6. The action aborts if the timeout is reached before arriving at the target
7. The action can be canceled at any time
