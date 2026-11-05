# Recon Mark 3 – Gazebo simulation (ROS 2 Humble, WSL2)

Gazebo Classic model of the Recon quadruped (8 hip/leg servos + neck), driven by a Python port of the
firmware's `Octosnake` oscillators and the gaits in `Recon.cpp`.

## Run (from Windows PowerShell, in this folder)

```powershell
.\run_sim.ps1 -Build      # first time / after editing src\  (builds in WSL ~/recon_ws)
.\run_sim.ps1             # opens Gazebo (WSLg) and plays the demo: walk, turn, back, head scan, sit, push-up, hello
.\run_sim.ps1 -NoDemo     # stand and wait for commands
.\run_sim.ps1 -Teleop     # keyboard control, in a SECOND PowerShell window
```

Inside WSL directly: `ros2 launch recon_sim recon_gazebo.launch.py gui:=true demo:=false period_ms:=500.0`

## Commands
Publish a letter on `/recon/command` (same letters as the firmware) or a `Twist` on `/cmd_vel`:

```bash
ros2 topic pub --once /recon/command std_msgs/msg/String "{data: f}"
```
`f` walk forward (`walk(0)`) · `b` back (`walk(1)`) · `l`/`r` turn · `s` stand · `k` sit · `g` run ·
`z` hello · `p` push-up · `u` up-down · `q` front-back · `w`/`t` body side · `d` dance · `x` twerk ·
`m` moonwalk · `n` wheel mode · `c` crab stand · `i` sit-wiggle · `y` toggle head scan.

Reset the world (e.g. if a gait tips the robot): `ros2 service call /reset_simulation std_srvs/srv/Empty`

## How it works
* `urdf/recon.urdf.xacro` – model rebuilt from your photos / MG90S sizes. Joint value = `radians(servo_angle-90)`.
* `src/recon_servo_plugin.cpp` – every joint is a torque-limited PD servo (≈MG90S 0.25 N·m); commands on `/recon/joint_commands`, states on `/joint_states`.
* `scripts/recon_gait.py` – oscillator gaits at 100 Hz, 600°/s servo speed limit.
* `config/gazebo_params.yaml` – raises `/clock` to 200 Hz (default 10 Hz made the gait stutter).

Measured: forward ≈ 9 cm/s, turn ≈ 58°/s, stable in all ported gaits.

## Assumptions / limits
* Dimensions are estimates from photos (no URDF/STL was available); the real STL meshes can replace the boxes.
* Servo trims from the firmware are not applied (the sim is perfectly calibrated).
* Direction convention follows `gaits()` in the .ino (FORWARD→`walk(0)`); the obstacle-avoidance code uses the opposite meaning for `walk(1)`.
* Chassis CoM is set 5 mm behind centre so the diagonal trot balances.
* Not ported: `omniWalk`, `jump`, WiFi/ultrasonic logic.

## SLAM mapping + robot camera (miniature office)

```powershell
.\run_sim.ps1 -Slam         # Gazebo (office world) + RViz: live map, laser scan, robot camera image
.\run_sim.ps1 -Teleop       # second window: drive with w / x / a / d / space
.\run_sim.ps1 -SaveMap      # third window / when finished: writes maps\office_map.pgm + .yaml
```
Drive slowly and stop (`space`) before turning; the map fills in as the robot explores the 3 rooms
(doorways are 0.8 m wide).  `maps\office_map_sample.*` is an example map from a short test drive.

* Sensors added to the robot (massless, so the gait is unchanged): 2D lidar `/scan` (360 samples, 0.1–2 m, 8 Hz)
  on a post at the rear, camera `/camera/image_raw` (320x240, 10 Hz) between the eyes, IMU `/imu`.
* `recon_odom.py` publishes `/odom` and TF `odom -> base_footprint -> base_link` from Gazebo's pose
  (ground truth, planar part only).  Add drift with `ros2 launch recon_sim recon_slam.launch.py odom_noise:=0.03`.
* `slam_toolbox` (online async) builds the `/map`; settings in `config/slam_params.yaml`.
* Limitation: the body rocks 2-3 degrees while walking, which can put a few false points on the map near 2-3 m range.
