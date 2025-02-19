# in each terminal:
source /opt/ros/humble/setup.bash
source install/setup.bash

# launch Clearpath simulation in Gazebo:
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/debug_clear_ws 

# launch RViz
ros2 launch clearpath_viz view_navigation.launch.py namespace:=j100_0001

# launch localization
ros2 launch clearpath_nav2_demos localization.launch.py setup_path:=$HOME/debug_clear_ws/ use_sim_time:=true

# launch Nav2
ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/debug_clear_ws/ use_sim_time:=true

# launch jackal in Rviz
ros2 launch clearpath_viz view_robot.launch.py namespace:=j100_0001

# topic
ros2 topic list
ros2 topic echo /yolo/detections


# launch YoloV8:
ros2 launch yolo_bringup yolo.launch.py
# launch YoloV8 with 3d bounding boxes
ros2 launch yolo_bringup yolov8.launch.py use_3d:=True

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_0_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_0_color_optical_frame






