cmake_minimum_required(VERSION 3.8)
project(recon_sim)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- servo model plugin (PD torque control of every joint, like an MG90S) ---
add_library(recon_servo_plugin SHARED src/recon_servo_plugin.cpp)
ament_target_dependencies(recon_servo_plugin gazebo_dev gazebo_ros rclcpp std_msgs sensor_msgs)
target_link_libraries(recon_servo_plugin ${GAZEBO_LIBRARIES})
ament_export_libraries(recon_servo_plugin)
install(TARGETS recon_servo_plugin LIBRARY DESTINATION lib)

install(DIRECTORY urdf launch config worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/recon_gait.py
  scripts/teleop_keys.py
  scripts/recon_odom.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
