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
