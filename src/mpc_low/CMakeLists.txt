cmake_minimum_required(VERSION 3.8)
project(mpc_low)

cmake_policy(SET CMP0057 NEW)  # Fix policy warning

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

# Enable C++17 for ROS 2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Silence Acados messages
add_compile_definitions(ACADOS_SILENT)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}  
  ${rcl_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  ${rosgraph_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/solver
)

include_directories(
  /home/robot/acados/include
  /home/robot/acados/acados/utils
  /home/robot/acados/acados
  /home/robot/acados/include/acados/utils 
  solver/
  solver/acados_libs/include
  solver/acados_libs/include/acados
  solver/acados_libs/include/acados_c
  /home/robot/acados/include
  /home/robot/acados/include/blasfeo/include
  /home/robot/acados/include/hpipm/include
)

# Find Acados libraries
find_library(ACADOS_LIB NAMES acados PATHS /home/robot/acados/build/lib /home/robot/acados/lib)
find_library(HPIPM_LIB NAMES hpipm PATHS /home/robot/acados/build/lib /home/robot/acados/lib)
find_library(BLASFEO_LIB NAMES blasfeo PATHS /home/robot/acados/build/lib /home/robot/acados/lib)

if(NOT ACADOS_LIB OR NOT HPIPM_LIB OR NOT BLASFEO_LIB)
  message(FATAL_ERROR "One or more Acados libraries were not found! Check /home/robot/acados/lib")
endif()

# Create library
add_library(mpc_low SHARED src/low_solver.cpp)
target_link_libraries(mpc_low ${ACADOS_LIB} ${HPIPM_LIB} ${BLASFEO_LIB})

# Add executable nodes

add_executable(custom_odometry_node_sim src/custom_odometry_sim.cpp)
ament_target_dependencies(mpc_low 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  gazebo_msgs 
  rosgraph_msgs 
  Eigen3
)
target_link_libraries(custom_odometry_node_sim mpc_low)

add_executable(mpc_low_node src/low_solver.cpp)
ament_target_dependencies(mpc_low_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  gazebo_msgs 
  rosgraph_msgs 
  Eigen3
)
target_link_libraries(mpc_low_node mpc_low)

# Install
install(TARGETS mpc_low mpc_low_node custom_odometry_node_sim
  DESTINATION lib/${PROJECT_NAME})

# Required for ROS 2 packages
ament_package()
