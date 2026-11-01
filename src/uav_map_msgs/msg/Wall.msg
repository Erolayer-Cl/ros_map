# Una pared detectada por uav_wall_mapping/mapper_node.
#
# Coordenadas NED de PX4 (x = norte, y = este), relativas al origen local
# (punto de despegue). Las alturas son sobre el despegue, positivas hacia arriba.

uint8 TYPE_FLAT      = 0   # plana: un segmento recto (points = 2 extremos)
uint8 TYPE_IRREGULAR = 1   # irregular: polilínea (salientes, curva, dentada, escombros)
uint8 TYPE_OBSTACLE  = 2   # obstáculo compacto (columna, caja): polígono cerrado

uint32 id                  # estable entre publicaciones mientras la pared no cambie mucho
uint8 type
string type_name           # "plana" | "irregular" | "obstaculo"
geometry_msgs/Point[] points   # vértices NED (z = 0)
float32 length             # m (largo del segmento o de la polilínea)
float32 thickness          # m, espesor aparente en planta
float32 rms_error          # m, dispersión de las celdas respecto de la geometría
float32 max_deviation      # m
float32 height_min         # m sobre el despegue (NaN si no hay datos 3D)
float32 height_max
float32 tilt_deg           # inclinación respecto de la vertical (NaN si no se pudo medir)
uint32 cells               # celdas de la grilla que la forman
