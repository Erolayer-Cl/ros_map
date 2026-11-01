#!/bin/bash
# Simulación con Unreal + Cosys-AirSim + PX4 v1.17 SITL + este stack, en tmux.
#
#   ESCENARIO=06_habitacion ./launch_sim_unreal.sh
#   ./launch_sim_unreal.sh            # sin escenario: paredes puestas a mano en Unreal
#
# Ventanas:
#   infra  -> PX4 SITL (none_iris), MicroXRCEAgent, airsim_node, foxglove_bridge
#   stack  -> mapper + misión (ros2 launch uav_sim unreal.launch.py) y un panel
#             libre con los comandos para crear el escenario y lanzar la misión
#
# Mismas lecciones que el repo de referencia: pane IDs estables, rtprio en el
# servidor tmux, todos los parámetros de airsim_node explícitos.
set -e
SESSION="uav_map"
ESCENARIO="${ESCENARIO:-}"
LIDAR_PITCH="${LIDAR_PITCH:-0.0}"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
UAV_LAB_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WS_DIR="$SCRIPT_DIR"
PX4_DIR="$UAV_LAB_DIR/dependencias/PX4-Autopilot"
AIRSIM_ROS_DIR="$UAV_LAB_DIR/dependencias/Cosys-AirSim/ros2"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "❌ Falta compilar el workspace: cd $WS_DIR && source $ROS_SETUP && colcon build"
  exit 1
fi

echo "🔧 Límite de prioridad real-time (PX4 SITL muere sin esto bajo carga)..."
sudo prlimit --pid=$$ --rtprio=99:99 2>/dev/null || echo "⚠️  no se pudo ajustar rtprio"
TMUX_SERVER_PID=$(tmux display-message -p '#{pid}' 2>/dev/null || true)
if [ -n "$TMUX_SERVER_PID" ]; then
  sudo prlimit --pid="$TMUX_SERVER_PID" --rtprio=99:99 2>/dev/null || true
fi

"$SCRIPT_DIR/stop_sim.sh" >/dev/null 2>&1 || true
sleep 1

echo "⚠️  Abrí el proyecto en Unreal (con AirSimGameMode y el settings.json de"
echo "   src/uav_sim/airsim/settings.json copiado a Documents\\AirSim\\) y dale Play."
read -p "Enter cuando Unreal esté en Play... " _

SRC="source $ROS_SETUP && source $WS_DIR/install/setup.bash"
AIRSIM_PARAMS="-p host_ip:=localhost -p host_port:=41451 -p is_vulkan:=true -p publish_clock:=false -p enable_api_control:=false -p enable_object_transforms_list:=true -p update_airsim_control_every_n_sec:=0.01 -p update_airsim_img_response_every_n_sec:=0.05 -p update_lidar_every_n_sec:=0.05 -p update_gpulidar_every_n_sec:=0.05 -p update_echo_every_n_sec:=0.05"
AIRSIM_REMAPS="-r /airsim_node/PX4/front_cam_Scene/image:=/camera -r /airsim_node/PX4/front_cam_Scene/camera_info:=/camera_info"

PANE_PX4=$(tmux new-session -d -s $SESSION -n infra -P -F '#{pane_id}')
tmux send-keys -t "$PANE_PX4" "cd $PX4_DIR && make px4_sitl_default none_iris" C-m
PANE_AGENT=$(tmux split-window -h -t "$PANE_PX4" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_AGENT" "MicroXRCEAgent udp4 -p 8888" C-m
PANE_AIRSIM=$(tmux split-window -v -t "$PANE_PX4" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_AIRSIM" "source $ROS_SETUP && cd $AIRSIM_ROS_DIR && source install/setup.bash && sleep 5 && ros2 run airsim_ros_pkgs airsim_node --ros-args $AIRSIM_PARAMS $AIRSIM_REMAPS" C-m
PANE_FOX=$(tmux split-window -v -t "$PANE_AGENT" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_FOX" "$SRC && sleep 5 && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m
tmux select-layout -t $SESSION:infra tiled

PANE_STACK=$(tmux new-window -t $SESSION -n stack -P -F '#{pane_id}')
tmux send-keys -t "$PANE_STACK" "$SRC && sleep 12 && ros2 launch uav_sim unreal.launch.py escenario:=$ESCENARIO lidar_pitch:=$LIDAR_PITCH" C-m
PANE_CMD=$(tmux split-window -v -t "$PANE_STACK" -P -F '#{pane_id}')
if [ -n "$ESCENARIO" ]; then
  HINT="ros2 run uav_sim spawn_scenario --escenario $ESCENARIO --limpiar"
else
  HINT="# (sin escenario: poné las paredes a mano en el editor de Unreal)"
fi
tmux send-keys -t "$PANE_CMD" "$SRC && clear && echo '1) $HINT' && echo '2) ros2 service call /mission/map std_srvs/srv/Trigger   (o /mission/start)' && echo '3) ros2 topic echo /mission/status --field message'" C-m
tmux select-layout -t $SESSION:stack even-vertical

tmux select-window -t $SESSION:stack
tmux attach -t $SESSION
