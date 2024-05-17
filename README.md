# visual_tools

ROS package to include runner scripts for common RGB-D tools including transformation and detection

## Environment Setup

For more details about environment setup instructions, check `docs/env_Setup`

### Hardware environment

- Franka Emika Cobot connected via Ethernet to panda machine.
- RealSense D435 RGB-D camera connected to dev machine through a USB3 Port.
- ROS communication between the two machines.

### Software environment

- panda machine
    - Ubuntu 20 + 5.9 rt kernel
    - ROS Noetic

- Dev machine
    - Ubuntu 20
    - ROS Noetic
    - CUDA 12

## Features

This repo serves to collect multiple image processing tools and interface them to the RealSense D435 Camera. This could be expanded simply by cloning the add-on package in the workspace src, then adding any associated configs, launch files, or wrapper scripts in this package.

### Core Features

launch_camera to launch realsense with depth, camera calibration, and transformation to robot hand.

### Add-ons

#### AR Marker

- Description: TODO
- Interfaces:
    - Published Topics
        - visualization_marker (visualization_msgs/Marker): This is an rviz message that when subscribed to (as a Marker in rviz), will display a colored square block at the location of each identified AR tag, and will also overlay these blocks in a camera image. Currently, it is set to display a unique color for markers 0-5 and a uniform color for all others.
        - ar_pose_marker (ar_track_alvar/AlvarMarkers): This is a list of the poses of all the observed AR tags, with respect to the output frame
    - tf Transforms
        - Camera frame (from Camera info topic param) → AR tag frame
        Provides a transform from the camera frame to each AR tag frame, named ar_marker_x, where x is the ID number of the tag.

#### EgoHOS: Hand-Object Segmentation

TODO

#### GA-DDPG: Grasp pose generation of arbitrary objects

TODO

## Package structure


## Tests

1. Run the launch file and verify no errors
2. echo ar tag topic and see the marker id and pose
3. visualize the pose on rviz and check that it's accurate
4. run pnp_demo_ARTag and start debugging!


## Todos
