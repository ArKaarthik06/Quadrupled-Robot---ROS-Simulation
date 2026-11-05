#!/usr/bin/env bash
# Saves the map currently built by slam_toolbox into recon_ws/maps/ (office_map.pgm + office_map.yaml)
source /opt/ros/humble/setup.bash
source "$HOME/recon_ws/install/setup.bash"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/maps"
ros2 run nav2_map_server map_saver_cli -f "$HERE/maps/office_map" --ros-args -p save_map_timeout:=10.0 -p use_sim_time:=true
