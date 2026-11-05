#!/usr/bin/env bash
# Starts Gazebo + the Recon robot + the gait node inside WSL.
# Extra arguments are passed to the launch file, e.g.  ./run_sim.sh gui:=false demo:=false period_ms:=600.0
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
if [ ! -f "$HOME/recon_ws/install/setup.bash" ]; then
  bash "$HERE/install_to_wsl.sh"
fi
source "$HOME/recon_ws/install/setup.bash"
# WSLg "COPY MODE" workaround: force plain X11 + software OpenGL for the Gazebo window
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
# Use the NVIDIA GPU through Mesa's D3D12 layer (default would be CPU llvmpipe, because the
# laptop also has an AMD iGPU).  Set RECON_SOFTWARE_GL=1 to fall back to the CPU renderer.
if [ "${RECON_SOFTWARE_GL:-0}" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
fi
unset WAYLAND_DISPLAY
# make sure no stale simulator from a previous run is still around
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
sleep 1
exec ros2 launch recon_sim "${RECON_LAUNCH:-recon_gazebo.launch.py}" "$@"
