cmake_minimum_required(VERSION 3.8)
project(handeye_target_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optional OpenCL support (OFF by default)
set(WITH_OPENCL OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # Added tf2_geometry_msgs
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Check for OpenCV version
if(OpenCV_VERSION VERSION_LESS "3.3")
  message(WARNING "handeye_target_detection works better with OpenCV version >= 3.3")
endif()

# Set include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Define source files for pose estimation
set(SOURCES_POSEESTIMATION
  src/pose_estimation_node.cpp 
  src/pose_estimator.cpp
)

# Create the pose_estimation executable
add_executable(pose_estimation ${SOURCES_POSEESTIMATION})

# Link dependencies for pose_estimation
ament_target_dependencies(pose_estimation
  rclcpp
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs  # Linked tf2_geometry_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Link OpenCV libraries
target_link_libraries(pose_estimation
  ${OpenCV_LIBS}
)

# Install the executable
install(TARGETS pose_estimation
  DESTINATION lib/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install other resources like launch files
install(DIRECTORY
  launch cfg data
  DESTINATION share/${PROJECT_NAME}/
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package
ament_package()

