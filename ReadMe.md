# in each terminal:
source /opt/ros/humble/setup.bash
source install/devel/setup.bash

# launch Clearpath simulation in Gazebo:
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/debug_clear_ws 

# launch YoloV8:
ros2 launch yolov8_bringup yolov8.launch.py 
# launch YoloV8 with 3d bounding boxes
ros2 launch yolov8_bringup yolov8.launch.py use_3d:=True