cmake_minimum_required(VERSION 3.16)
project(robot_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  # Ensure rclcpp_action is found
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Set include directory path
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}  # Include rclcpp_action
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS})

# Add robot interface library
set(${PROJECT_NAME}_SOURCES
  src/control_base.cpp
  src/control_ur.cpp
) 
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  rclcpp_action  # Link with rclcpp_action
  sensor_msgs 
  geometry_msgs 
  tf2_ros 
  controller_manager 
  control_msgs 
  trajectory_msgs)

# Add executables if needed (e.g., tests or demo)
# add_executable(<your_executable> src/<your_source>.cpp)
# ament_target_dependencies(<your_executable> <dependencies>)

ament_export_include_directories(include ${Eigen3_INCLUDE_DIRS})
ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)  # Export rclcpp_action
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files  
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files, if applicable
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

