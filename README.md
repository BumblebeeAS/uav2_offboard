# Jellyfish2-Offboard
Offboard package for drone behaviours and missions

## Dependencies
To run the sim and packages, you would need:
1. This package
2. PX4 autopilot repo from: https://github.com/PX4/PX4-Autopilot/tree/v1.14.3
   Note: Make sure the repo is fixed to this tag(v1.14.3)
3. PX4 msgs from: https://github.com/PX4/px4_msgs
   Replace the msgs from the PX4 autopilot repo instead
   Possible lines to replace msg definitions: 
    ```bash
    rm px4_msgs/msg/*
    cp PX4-Autopilot/msg/* px4_msgs/msg/
    ```
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
    ros2 launch jellyfish2_offboard launch.py
    ```
    

