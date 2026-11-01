#!/bin/bash
# Arma la estructura UAV_lab/{Code/ros_map, dependencias/...} y clona las
# dependencias externas en las versiones que usa este repo:
#   - PX4-Autopilot v1.17.0         (firmware + SITL)
#   - Micro-XRCE-DDS-Agent v2.4.3   (puente uXRCE-DDS <-> ROS 2)
#   - Cosys-AirSim                  (plugin de Unreal + wrapper ROS 2), sólo con UNREAL=1
# y deja inicializado el submódulo src/px4_msgs (rama release/1.17).
#
# Idempotente: lo que ya existe no se toca.
#   ./setup_workspace.sh
#   UNREAL=1 ./setup_workspace.sh
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
UAV_LAB_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
DEPS_DIR="$UAV_LAB_DIR/dependencias"
PX4_TAG="${PX4_TAG:-v1.17.0}"

echo "UAV_lab detectado en: $UAV_LAB_DIR"
echo "  (se espera <UAV_lab>/Code/ros_map; si el repo está en otro lado, movelo ahí"
echo "   o ajustá las rutas a mano)"
mkdir -p "$DEPS_DIR/micro"

clone_if_missing() {
  local target="$1" url="$2"
  shift 2
  if [ -d "$target/.git" ]; then
    echo "OK  $target ya existe, no se toca."
  else
    echo "--> Clonando $url en $target ..."
    git clone "$@" "$url" "$target"
  fi
}

clone_if_missing "$DEPS_DIR/PX4-Autopilot" https://github.com/PX4/PX4-Autopilot.git \
  --branch "$PX4_TAG" --recursive
clone_if_missing "$DEPS_DIR/micro/Micro-XRCE-DDS-Agent" \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git --branch v2.4.3
if [ "${UNREAL:-0}" = "1" ]; then
  clone_if_missing "$DEPS_DIR/Cosys-AirSim" https://github.com/Cosys-Lab/Cosys-AirSim.git
fi

cd "$SCRIPT_DIR"
if [ -e src/px4_msgs/.git ]; then
  echo "OK  src/px4_msgs ya está inicializado (no se pisa una sincronización a mano)."
else
  echo "--> Inicializando submódulo src/px4_msgs (release/1.17)..."
  git submodule update --init --recursive
fi

cat <<EOT

Listo. Próximos pasos (detalle en README.md):
  1. PX4:   cd $DEPS_DIR/PX4-Autopilot && bash ./Tools/setup/ubuntu.sh --no-sim-tools && make px4_sitl
  2. Agent: cd $DEPS_DIR/micro/Micro-XRCE-DDS-Agent && mkdir -p build && cd build && cmake .. && make && sudo make install && sudo ldconfig
  3. ROS:   cd $SCRIPT_DIR && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
  4. Probar sin nada más:  ./run_tests.sh   y   ./launch_fake_sim.sh 06_habitacion
  5. Unreal: ver docs/casos-de-prueba-unreal.md
EOT
