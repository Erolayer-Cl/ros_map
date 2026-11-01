#!/bin/bash
# Stack completo SIN Unreal ni GPU: PX4 + lidar falsos (mismo escenario YAML
# que en Unreal), mapper y misión. Sirve en cualquier PC con ROS 2.
#
#   ./launch_fake_sim.sh 06_habitacion          # y después: ros2 service call /mission/map std_srvs/srv/Trigger
#   AUTO=true ./launch_fake_sim.sh 05_trampa_U  # arranca sola la misión del escenario
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
ESCENARIO="${1:-06_habitacion}"
source "$ROS_SETUP"
source "$SCRIPT_DIR/install/setup.bash"
exec ros2 launch uav_sim fake_sim.launch.py escenario:="$ESCENARIO" auto:="${AUTO:-false}" \
  lidar_pitch:="${LIDAR_PITCH:-0.0}"
