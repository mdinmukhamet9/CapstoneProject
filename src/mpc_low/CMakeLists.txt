cmake_minimum_required(VERSION 3.8)
project(mpc_low)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

# Include directories
include_directories(
  include
  solver/
  solver/Jackal_model
  solver/Jackal_constraints
  ${rclcpp_INCLUDE_DIRS}
  /home/robot/acados/include
  /home/robot/acados/include/acados
  /home/robot/acados/include/blasfeo/include
  /home/robot/acados/include/hpipm/include
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
)

# Locate solver sources
file(GLOB SOLVER_SOURCES "solver/*.c")

# Ensure files were found
if(NOT SOLVER_SOURCES)
  message(FATAL_ERROR "No solver source files found in solver/")
endif()

# Define executable
add_executable(low_solver 
  src/low_solver.cpp
  solver/Jackal_model/Jackal_expl_ode_fun.c
  solver/Jackal_model/Jackal_expl_vde_forw.c
  solver/Jackal_model/Jackal_expl_vde_adj.c
  solver/Jackal_constraints/Jackal_constr_h_fun_jac_uxt_zt.c
  solver/Jackal_constraints/Jackal_constr_h_fun.c
  solver/acados_solver_Jackal.c
  solver/acados_sim_solver_Jackal.c
  solver/acados_solver_holder.cpp
)



# Link directories
link_directories(/home/robot/acados/lib)

# Link libraries
target_link_libraries(low_solver
  ${rclcpp_LIBRARIES}
  /home/robot/acados/lib/libacados.so
  /home/robot/acados/lib/libhpipm.so
  /home/robot/acados/lib/libblasfeo.so
)

# Set compile features
target_compile_features(low_solver PUBLIC c_std_99 cxx_std_17)

# Define dependencies
ament_target_dependencies(low_solver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
  rosgraph_msgs
  Eigen3
)

# Install targets
install(TARGETS low_solver
  DESTINATION lib/${PROJECT_NAME})

# Enable testing if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()