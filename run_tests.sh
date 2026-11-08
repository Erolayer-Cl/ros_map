#!/bin/bash
# Todas las pruebas, sin ROS ni Unreal (sólo python3 + numpy + pyyaml).
#   ./run_tests.sh              # unitarias + robustez + 6 escenarios de lazo cerrado (~2 min)
#   TODOS=1 ./run_tests.sh      # los 14 escenarios (~3 min)
#   ./run_tests.sh rapido       # sólo unitarias (segundos)
#   ./run_tests.sh robustez     # fuzzer + los 3 nodos con mensajes basura en vuelo (~1 min)
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
robustez() {
  echo "== robustez: fuzzer de la lógica pura"
  python3 qa/fuzz/fuzz_core.py | tail -1
  echo "== robustez: mapper_node + mission_node + fake_px4_world con basura en vuelo (ROS falso)"
  python3 qa/nodos/run_inject.py 06_habitacion 300 4 | grep -E 'RESULTADO|setpoints|distancia'
}
if [ "${1:-}" = "robustez" ]; then
  robustez
  exit 0
fi
for pkg in uav_wall_mapping uav_mission; do
  echo "== $pkg"
  python3 -m unittest discover -s "src/$pkg/test" -p 'test_*.py'
done
if [ "${1:-}" != "rapido" ]; then
  robustez
  echo "== uav_sim (lazo cerrado: misión + mapper + lidar simulado)"
  ROS_MAP_TODOS="${TODOS:-}" python3 -m unittest discover -s src/uav_sim/test -p 'test_*.py' -v
fi
