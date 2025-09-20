cmake_minimum_required(VERSION 3.8)
project(ardupilot_survey_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(survey_mission_node src/survey_mission_node.cpp)
ament_target_dependencies(survey_mission_node
  rclcpp
  mavros_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install C++ executables
install(TARGETS
  survey_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/survey_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()