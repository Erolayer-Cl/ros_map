# Histograma polar de obstáculos alrededor del dron (/obstacles/sectors).
#
# Lo publica mapper_node y lo consume la evasión de mission_node. Mismo formato
# que px4_msgs/ObstacleDistance (72 sectores de 5°, centrado en el Norte) pero
# en metros y float.

# header.stamp = instante de CAPTURA del dato en vivo más nuevo que se usó (no
# el de publicación): la misión calcula la edad con esto y la suma a la
# latencia del frenado. Los sectores en sí están calculados contra la posición
# del dron al publicar (`position`), a partir de puntos guardados en NED.
std_msgs/Header header
float32 angle_offset       # rumbo NED (rad) del CENTRO del sector 0 (0 = Norte)
float32 increment          # ancho de cada sector (rad), positivo hacia el Este
float32 range_max          # m: más allá de esto no se reporta nada
float32[] ranges           # distancia horizontal desde el centro del dron; inf = libre
geometry_msgs/Point position   # posición NED del dron al calcularlos
float32 band_below         # franja de alturas considerada, relativa al dron (m)
float32 band_above
bool fresh_sensor          # false: sólo hay datos del mapa, el sensor no publica
