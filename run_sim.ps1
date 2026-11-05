# Windows launcher for the Recon Gazebo simulation (runs in WSL2 + WSLg).
#
#   .\run_sim.ps1                 Gazebo arena, wait for keyboard commands
#   .\run_sim.ps1 -Slam           miniature OFFICE + lidar SLAM mapping + camera view in RViz
#   .\run_sim.ps1 -Build          force a rebuild first (after editing files in src\)
#   .\run_sim.ps1 -Demo           play the scripted demo (arena only)
#   .\run_sim.ps1 -Teleop         keyboard control (run this in a SECOND PowerShell window)
#   .\run_sim.ps1 -SaveMap        save the SLAM map to maps\office_map.pgm/.yaml (while -Slam is running)
param(
    [switch]$Build,
    [switch]$Demo,
    [switch]$NoGui,
    [switch]$Teleop,
    [switch]$Slam,
    [switch]$SaveMap,
    [switch]$ViewUrdf,
    [string]$Distro = "Ubuntu-22.04"
)

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$wslHere = (wsl -d $Distro -- wslpath -a "$here").Trim()

if ($Teleop) {
    wsl -d $Distro -- bash "$wslHere/teleop.sh"
    return
}
if ($ViewUrdf) {
    wsl -d $Distro -- bash "$wslHere/view_urdf.sh"
    return
}
if ($SaveMap) {
    wsl -d $Distro -- bash "$wslHere/save_map.sh"
    return
}
if ($Build) {
    wsl -d $Distro -- bash "$wslHere/install_to_wsl.sh"
}
$guiArg = if ($NoGui) { "false" } else { "true" }
if ($Slam) {
    wsl -d $Distro -- env RECON_LAUNCH=recon_slam.launch.py bash "$wslHere/run_sim.sh" "gui:=$guiArg"
} else {
    $demoArg = if ($Demo) { "true" } else { "false" }
    wsl -d $Distro -- bash "$wslHere/run_sim.sh" "gui:=$guiArg" "demo:=$demoArg"
}
