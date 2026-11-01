#!/bin/bash
# Para la simulación. mission_node NO se mata con -9: con -9 no corre su
# land_on_shutdown() y el dron quedaría armado. Cerralo con Ctrl-C en su panel.
tmux kill-session -t uav_map 2>/dev/null || true
for p in px4_sitl "PX4-Autopilot/build" MicroXRCEAgent airsim_node foxglove_bridge mapper_node fake_px4_world; do
  pkill -9 -f "$p" 2>/dev/null || true
done
echo "✅ Todo parado."
