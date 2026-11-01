# Estado de la misión publicado por uav_mission/mission_node en /mission/status.
#
# Todos los campos comparten header.stamp: un panel Plot de Foxglove puede
# superponer series (obstacle_distance vs ground_speed, coverage vs tiempo...).
# Distancias en metros; NaN = "no aplica" (Foxglove deja un hueco en vez de
# aplastar el eje con un cero o un infinito).

std_msgs/Header header

# --- Máquina de estados (mismos valores que uav_mission.mission_core.State) ---
uint8 STATE_IDLE      = 0   # esperando /mission/start o /mission/map
uint8 STATE_ARMING    = 1   # mandando setpoints y pidiendo OFFBOARD + ARM
uint8 STATE_TAKEOFF   = 2   # subiendo a la altura de la misión
uint8 STATE_NAVIGATE  = 3   # yendo al objetivo actual (waypoint, frontera, carril)
uint8 STATE_AVOID     = 4   # yendo al objetivo, desviado por la evasión
uint8 STATE_HOLD      = 5   # posición mantenida (pausa, fallo de evasión, fin sin aterrizaje)
uint8 STATE_RETURN    = 6   # volviendo al punto de despegue
uint8 STATE_LAND      = 7   # aterrizando
uint8 STATE_DONE      = 8   # misión completada, desarmado
uint8 STATE_ABORTED   = 9   # /mission/abort: aterrizaje inmediato
uint8 STATE_PREEMPTED = 10  # el piloto sacó al dron de OFFBOARD
uint8 STATE_FAILED    = 11  # no se pudo armar / entrar en OFFBOARD
uint8 STATE_SCAN      = 12  # girando 360° en el lugar para ver alrededor

uint8 MISSION_NONE      = 0
uint8 MISSION_WAYPOINTS = 1
uint8 MISSION_MAPPING   = 2

uint8 state
string state_name
uint8 mission_type
string message

# --- Progreso ---
uint16 waypoint_index
uint16 waypoint_count
string goal_kind               # "waypoint 0", "frontera", "carril de barrido", "home"
float32 distance_to_goal       # horizontal
float32 distance_to_home
float32 distance_traveled
float32 tracking_error         # |setpoint - posición|: cuánto le cuesta seguir la referencia
float32 altitude               # sobre el despegue
float32 ground_speed
uint16 path_points             # vértices del camino A* actual (0 = sin camino)
string plan_status

# --- Evasión ---
bool avoiding
float32 obstacle_distance      # obstáculo más cercano en cualquier dirección
float32 obstacle_bearing       # rumbo NED (rad) de ese obstáculo
string avoid_reason

# --- Mapeo ---
float32 coverage               # fracción conocida del área de mapeo (0..1)
uint16 frontiers               # fronteras abiertas dentro del área
uint16 goals_reached
uint16 walls_flat
uint16 walls_irregular
uint16 obstacles
