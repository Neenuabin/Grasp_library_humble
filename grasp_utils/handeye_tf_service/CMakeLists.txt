cmake_minimum_required(VERSION 3.8)  # Humble generally uses at least CMake 3.8

project(handeye_tf_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
set(srv_files
  "srv/HandeyeTF.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

# Export rosidl dependencies
ament_export_dependencies(rosidl_default_runtime)

# Add the handeye_tf_server executable
add_executable(handeye_tf_server 
  src/handeye_tf_server.cpp
)

# Link the executable with the required dependencies
ament_target_dependencies(handeye_tf_server
  rclcpp tf2_ros geometry_msgs std_msgs
)

# Install the handeye_tf_server binary
install(TARGETS handeye_tf_server
    DESTINATION lib/${PROJECT_NAME})

# Ensure service support
rosidl_target_interfaces(handeye_tf_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_package()

