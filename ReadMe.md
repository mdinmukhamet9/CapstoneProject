# in each terminal:
source /opt/ros/humble/setup.bash
source install/setup.bash

# launch Clearpath simulation in Gazebo:
ros2 launch clearpath_gz simulation.launch.py

# launch RViz
ros2 launch clearpath_viz view_navigation.launch.py namespace:=j100_0001

# launch localization
ros2 launch clearpath_nav2_demos localization.launch.py setup_path:=$HOME/debug_clear_ws/ use_sim_time:=true

# launch Nav2
ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/debug_clear_ws/ use_sim_time:=true

# launch YoloV8:
ros2 launch yolo_bringup yolo.launch.py

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_0_link

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_0_color_optical_frame

# MPC activate
ros2 run mpc_low low_solver

# launch jackal in Rviz
ros2 launch clearpath_viz view_robot.launch.py namespace:=j100_0001

# topic
ros2 topic list
ros2 topic echo /yolo/detections

# camera launching
ros2 launch realsense2_camera rs_launch.py

# if there is error with TF timeout
ros2 daemon stop
ros2 daemon start

