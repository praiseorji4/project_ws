cmake_minimum_required(VERSION 3.5)
project(my_robot)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(pluginlib REQUIRED)


# Include directories
include_directories(
  hardware/include
)

# Executable
add_library(my_robot_hardware SHARED
  hardware/src/my_robot_hardware.cpp
)

pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)
# Link libraries
target_link_libraries(my_robot_hardware pigpiod_if2)

ament_target_dependencies(my_robot_hardware
  rclcpp
  hardware_interface
  controller_manager
  diff_drive_controller
  joint_state_broadcaster
  ros2_control
  ros2_control_test_assets
)

# Install
install(TARGETS
  my_robot_hardware
  DESTINATION lib
)

install(
  DIRECTORY bringup description hardware
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rclcpp hardware_interface pluginlib pigpiod_if2)

ament_package()
