#!/bin/bash
# Todas las pruebas, sin ROS ni Unreal (sólo python3 + numpy + pyyaml).
#   ./run_tests.sh            # unitarias + 6 escenarios de lazo cerrado (~2 min)
#   TODOS=1 ./run_tests.sh    # los 14 escenarios (~3 min)
#   ./run_tests.sh rapido     # sólo unitarias (segundos)
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
for pkg in uav_wall_mapping uav_mission; do
  echo "== $pkg"
  python3 -m unittest discover -s "src/$pkg/test" -p 'test_*.py'
done
if [ "${1:-}" != "rapido" ]; then
  echo "== uav_sim (lazo cerrado: misión + mapper + lidar simulado)"
  ROS_MAP_TODOS="${TODOS:-}" python3 -m unittest discover -s src/uav_sim/test -p 'test_*.py' -v
fi
