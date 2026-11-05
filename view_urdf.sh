#!/usr/bin/env bash
# Opens the robot model in RViz with a slider window to move every joint (no physics).
source /opt/ros/humble/setup.bash
source "$HOME/recon_ws/install/setup.bash"
export QT_QPA_PLATFORM=xcb
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
PKG=$(ros2 pkg prefix recon_sim)/share/recon_sim
xacro "$PKG/urdf/recon.urdf.xacro" -o /tmp/recon_view.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/recon_view.urdf &
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
ros2 run rviz2 rviz2 -d "$PKG/config/view_urdf.rviz"
kill %1 %2 2>/dev/null
