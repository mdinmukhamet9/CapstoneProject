# Clearpath Simulation with YOLOv8, Navigation, and RealSense in ROS 2 (Humble)

This repository provides a setup for running Clearpath's simulation in Gazebo, YOLOv8 for object detection, Nav2 for navigation, and RealSense camera integration in ROS 2 Humble.

## Setup & Execution

In each terminal, source the necessary environment variables:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Launch Clearpath Simulation in Gazebo:

```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/debug_clear_ws
```

### Launch YOLOv8:

```bash
ros2 launch yolo_bringup yolo.launch.py
```

### Launch YOLOv8 with 3D Bounding Boxes:

```bash
ros2 launch yolo_bringup yolov8.launch.py use_3d:=True
```

### Launch RViz:

```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=j100_0001
```

### Launch Localization:

```bash
ros2 launch clearpath_nav2_demos localization.launch.py setup_path:=$HOME/debug_clear_ws/ use_sim_time:=true
```

### Launch Nav2:

```bash
ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/debug_clear_ws/ use_sim_time:=true
```

### Launch Jackal in RViz:

```bash
ros2 launch clearpath_viz view_robot.launch.py namespace:=j100_0001
```

### Check ROS 2 Topics:

```bash
ros2 topic list
ros2 topic echo /yolo/detections
```

### Launch RealSense Camera:

```bash
ros2 launch realsense2_camera rs_launch.py
```

## Notes

- Ensure your ROS 2 workspace is correctly built and sourced before launching any nodes.
- The `setup_path` should point to your Clearpath simulation workspace.
- Modify `use_3d` as needed for your application.
- The namespace `j100_0001` should match your robot’s configuration.

# topic
ros2 topic list
ros2 topic echo /yolo/detections


# launch YoloV8:
ros2 launch yolo_bringup yolo.launch.py
# launch YoloV8 with 3d bounding boxes
ros2 launch yolo_bringup yolov8.launch.py use_3d:=True

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_0_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_0_color_optical_frame

# detected jump back in time

ros2 daemon stop
ros2 daemon start






