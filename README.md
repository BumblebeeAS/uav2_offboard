# UAV2 Offboard

Offboard package for drone behaviours and missions

## Running the sim

On separate terminals:

1. Start the Default Gazebo sim
   ```bash
   make px4_sitl gz_x500
   ```
2. Start px4-ros2 bridge
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```
3. Launch the package
   ```bash
   ros2 launch uav2_offboard launch.py
   ```

## GoToPosition Action Server

An action server for PX4 offboard control that commands the vehicle to navigate to a target position with configurable thresholds and timeout.

### Usage

Before using the action server, ensure:

1. **PX4 is running** and connected via XRCE-DDS
2. **Vehicle is armed** and in **offboard mode**

In one terminal:

```bash
ros2 run uav2_offboard go_to_position_action_server
```

In another terminal:

```bash
ros2 action send_goal /go_to_position uav2_offboard/action/GoToPosition "{x: 5.0, y: 5.0, z: -2.0, x_threshold: 0.5, y_threshold: 0.5, z_threshold: 0.5}" --feedback
```

### How It Works

1. The action server continuously publishes `OffboardControlMode` messages at 50 Hz to maintain offboard mode
2. When a goal is received, it starts commanding the target position via `TrajectorySetpoint` messages
3. It monitors the vehicle's current position from `VehicleLocalPosition` messages
4. Feedback is published showing current position, distance to goal, and time remaining
5. The action succeeds when all position errors are within their respective thresholds
6. The action aborts if the timeout is reached before arriving at the target
7. The action can be canceled at any time
