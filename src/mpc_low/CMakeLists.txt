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
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yolo_msgs REQUIRED)

message("yolo_msgs include dir: ${yolo_msgs_INCLUDE_DIRS}")

# Find acados libraries
find_library(ACADOS_LIBRARY NAMES acados PATHS /home/dimash/acados/lib NO_DEFAULT_PATH)
find_library(HPIPM_LIBRARY NAMES hpipm PATHS /home/dimash/acados/lib NO_DEFAULT_PATH)
find_library(BLASFEO_LIBRARY NAMES blasfeo PATHS /home/dimash/acados/lib NO_DEFAULT_PATH)

# Check if acados libraries were found
if(NOT ACADOS_LIBRARY)
  message(FATAL_ERROR "libacados.so not found in /home/dimash/acados/lib. Please ensure acados is installed.")
endif()
if(NOT HPIPM_LIBRARY)
  message(FATAL_ERROR "libhpipm.so not found in /home/dimash/acados/lib. Please ensure acados is installed.")
endif()
if(NOT BLASFEO_LIBRARY)
  message(FATAL_ERROR "libblasfeo.so not found in /home/dimash/acados/lib. Please ensure acados is installed.")
endif()

# Include directories
include_directories(
  include
  solver
  solver/Jackal_model
  solver/Jackal_constraints
  /home/dimash/acados/include
  /home/dimash/acados/include/acados
  /home/dimash/acados/include/blasfeo/include
  /home/dimash/acados/include/hpipm/include
  ${EIGEN3_INCLUDE_DIR}
)

# Solver sources
set(SOLVER_SOURCES
  solver/acados_solver_Jackal.c
  solver/acados_sim_solver_Jackal.c
  solver/acados_solver_holder.cpp
  solver/Jackal_model/Jackal_expl_ode_fun.c
  solver/Jackal_model/Jackal_expl_vde_forw.c
  solver/Jackal_model/Jackal_expl_vde_adj.c
  solver/Jackal_constraints/Jackal_constr_h_fun_jac_uxt_zt.c
  solver/Jackal_constraints/Jackal_constr_h_fun.c
)

# Define dependencies
set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav2_costmap_2d
  nav2_core
  nav2_util
  pluginlib
  Eigen3
  yolo_msgs
)

# Build library
add_library(${PROJECT_NAME}_plugin SHARED
  src/mpc_controller.cpp
  ${SOLVER_SOURCES}
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}_plugin ${dependencies})

# Link acados libraries
target_link_libraries(${PROJECT_NAME}_plugin
  ${ACADOS_LIBRARY}
  ${HPIPM_LIBRARY}
  ${BLASFEO_LIBRARY}
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_core mpc_controller_plugin.xml)

# Install targets
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install solver headers and sources
install(DIRECTORY solver/
  DESTINATION include/${PROJECT_NAME}/solver
)

# Install plugin description
install(FILES mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(${dependencies})

ament_package()