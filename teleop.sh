#!/usr/bin/env bash
# Keyboard control of the simulated Recon (run in a second terminal while the simulation is running).
source /opt/ros/humble/setup.bash
source "$HOME/recon_ws/install/setup.bash"
exec ros2 run recon_sim teleop_keys.py
