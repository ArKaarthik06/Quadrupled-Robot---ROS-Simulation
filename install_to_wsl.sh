#!/usr/bin/env bash
# Copies recon_sim from the Windows folder into a native WSL workspace (~/recon_ws) and builds it.
# Usage (from Windows PowerShell):
#   wsl -d Ubuntu-22.04 -- bash "/mnt/c/Users/<you>/.../End Semester Project/recon_ws/install_to_wsl.sh"
set -e
SRC_DIR="$(cd "$(dirname "$0")" && pwd)/src/recon_sim"
WS="$HOME/recon_ws"
mkdir -p "$WS/src/recon_sim"
# incremental copy (keeps timestamps, so only changed files are rebuilt)
if command -v rsync >/dev/null 2>&1; then
  rsync -a --delete "$SRC_DIR/" "$WS/src/recon_sim/"
else
  rm -rf "$WS/src/recon_sim" && cp -rp "$SRC_DIR" "$WS/src/recon_sim"
fi
# Windows line endings would break the python shebangs
find "$WS/src/recon_sim" \( -name '*.py' -o -name '*.sh' \) -exec sed -i 's/\r$//' {} \;
chmod +x "$WS"/src/recon_sim/scripts/*.py
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --packages-select recon_sim
echo
echo "Built.  Run:  source ~/recon_ws/install/setup.bash && ros2 launch recon_sim recon_gazebo.launch.py"
