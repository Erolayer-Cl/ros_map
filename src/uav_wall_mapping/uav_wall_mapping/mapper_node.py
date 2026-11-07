#!/usr/bin/env python3
"""Nodo de mapeo: nubes de puntos -> mapa 2D, mapa 3D, paredes y sectores de obstáculos.

Adaptador ROS de `mapper_core.WallMapper` (que es Python puro). Corre siempre,
haya misión o no: el mapa se va armando con cada vuelo. La misión de mapeo
(`/mission/map` en mission_node) es la que decide a dónde volar para completarlo
y cuándo guardarlo.

Entradas (todas opcionales; con una alcanza):
- `cloud.topics`: sensor_msgs/PointCloud2 (lidar 3D, cámara de profundidad).
  En Unreal: `/airsim_node/PX4/lidar/points/lidar`. Con un Livox Mid-360
  real: `/livox/lidar`. Con una RealSense: `/camera/depth/color/points`.
- `scan.topics`: sensor_msgs/LaserScan (lidar 2D, LightWare SF45).
- `range.topics`: sensor_msgs/Range (rangefinders 1D, o los Distance de AirSim
  del repo de referencia).
- La pose sale de PX4 por uXRCE-DDS (`/fmu/out/vehicle_local_position_v1` +
  `/fmu/out/vehicle_attitude`): no hace falta TF ni MAVROS. Se guarda un
  historial de 2 s (`pose_buffer.py`) y cada nube se transforma con la pose
  interpolada en su `header.stamp` (instante de captura), no con la última.
  Para eso los tiempos de PX4 tienen que estar en la base del reloj de ROS
  (`UXRCE_DDS_SYNCT=1`, ver docs/jetson-px4-1.17.md); si no lo están (o el
  stamp de la nube viene de otro reloj, p. ej. un Livox sin PTP) se avisa y se
  usa la hora de llegada.

Salidas:
- `/map` (nav_msgs/OccupancyGrid, frame `map`, ENU): el plano.
- `/map/cloud` (PointCloud2 ENU, color por altura): el mapa 3D.
- `/walls` (uav_map_msgs/WallArray, NED) y `/walls/markers` (MarkerArray ENU).
- `/obstacles/sectors` (uav_map_msgs/ObstacleSectors): para la evasión.
- `/fmu/in/obstacle_distance` (opcional): el mismo histograma para la
  Collision Prevention de PX4, como red de seguridad en modo Position.
- TF `map` -> `base_link` (opcional) para seguir al dron en Foxglove.

Servicios: `/map/save` (guarda PGM/YAML, PNG, SVG, JSON y PLY en `output_dir`)
y `/map/reset`.
"""

import array
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Range
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from px4_msgs.msg import ObstacleDistance, VehicleAttitude, VehicleLocalPosition
from uav_map_msgs.msg import ObstacleSectors, Wall as WallMsg, WallArray

from uav_wall_mapping.cloud import (laserscan_to_xyz, pointcloud2_to_xyz, range_to_xyz,
                                    xyz_to_pointcloud2_data)
from uav_wall_mapping.export import WALL_COLORS, save_map_bundle
from uav_wall_mapping.geometry import (SensorMount, ned_to_enu, px4_quat_to_enu_flu,
                                       yaw_from_quat)
from uav_wall_mapping.mapper_core import MapperConfig, WallMapper
from uav_wall_mapping.pose_buffer import PoseBuffer
from uav_wall_mapping.sectors import to_px4_obstacle_distance
from uav_wall_mapping.walls import FLAT, IRREGULAR, OBSTACLE

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
MAP_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
# Sectores: sólo sirve el último. RELIABLE con cola encolaría datos viejos
# detrás de una pérdida (mission_node usa el mismo perfil).
SECTORS_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

WALL_PARAMS = {
    'spine_resolution': 0.3, 'link_distance': 0.6, 'min_cells': 5, 'rdp_tolerance': 0.3,
    'flat_max_rms': 0.08, 'max_wall_thickness': 0.8, 'flat_max_bow': 0.06,
    'min_flat_length': 1.0, 'min_wall_length': 1.0, 'max_obstacle_extent': 3.0,
    'merge_angle_deg': 12.0, 'merge_gap': 0.6, 'join_distance': 1.0,
    'min_irregular_length': 0.6, 'corner_trim': 0.4,
}


def _triplets(flat, count, default):
    values = list(flat or [])
    out = []
    for i in range(count):
        chunk = values[3 * i:3 * i + 3]
        out.append(tuple(chunk) if len(chunk) == 3 else default)
    return out


class MapperNode(Node):

    def __init__(self):
        super().__init__('mapper_node')
        self._declare()
        p = self.get_parameter

        cfg = MapperConfig(
            resolution=p('map.resolution').value,
            initial_size=p('map.initial_size').value,
            max_size=p('map.max_size').value,
            min_height=p('map.min_height').value,
            max_height=p('map.max_height').value,
            max_height_above_drone=p('map.max_height_above_drone').value,
            floor_slope_deg=p('map.floor_slope_deg').value,
            no_hit_free_range=p('map.no_hit_free_range').value,
            free_ray_bins=p('map.free_ray_bins').value,
            sensor_fov_deg=p('map.sensor_fov_deg').value,
            min_range=p('map.min_range').value,
            self_filter_box=(tuple(p('self_filter.box').value)
                             if len(p('self_filter.box').value) == 6 else None),
            outlier_voxel=p('outliers.voxel').value,
            outlier_min_points=p('outliers.min_points').value,
            outlier_max_range=p('outliers.max_range').value,
            max_range=p('map.max_range').value,
            voxel_resolution=p('map.voxel_resolution').value,
            sector_bins=72,
            sector_range=p('sectors.range').value,
            sector_band_below=p('sectors.band_below').value,
            sector_band_above=p('sectors.band_above').value,
            sector_min_points=p('sectors.min_points').value,
            sector_timeout=p('sectors.sensor_timeout').value,
            min_cloud_points=p('sectors.min_cloud_points').value,
            sectors_from_map=p('sectors.from_map').value,
            forget_after=p('map.forget_after').value,
            forget_max_hits=p('map.forget_max_hits').value,
            weak_protect_sec=p('map.weak_protect_sec').value,
            traversed_radius=p('map.traversed_radius').value,
            walls={k: p(f'walls.{k}').value for k in WALL_PARAMS},
        )
        self.mapper = WallMapper(cfg)
        self.lock = threading.Lock()
        self.poses = PoseBuffer(horizon=p('time.pose_history').value,
                                max_extrapolation=p('time.max_extrapolation').value)

        self.position = None
        self.q = None
        self.pose_stamp = 0.0
        self._attitude_ever = False
        self._skew_warned = set()
        self.position_valid = False
        self._last_cloud = {}
        self._sensor_ever = False
        self._map_pub_count = 0

        heavy = MutuallyExclusiveCallbackGroup()   # extracción de paredes y guardado
        sensors = MutuallyExclusiveCallbackGroup()

        self.create_subscription(VehicleLocalPosition, p('px4.local_position_topic').value,
                                 self._on_position, PX4_QOS)
        self.create_subscription(VehicleAttitude, p('px4.attitude_topic').value,
                                 self._on_attitude, PX4_QOS)

        self._subscribe_sensors(sensors)

        self.pub_map = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.pub_cloud = self.create_publisher(PointCloud2, '/map/cloud', 2)
        self.pub_walls = self.create_publisher(WallArray, '/walls', MAP_QOS)
        self.pub_markers = self.create_publisher(MarkerArray, '/walls/markers', MAP_QOS)
        self.pub_sectors = self.create_publisher(ObstacleSectors, '/obstacles/sectors',
                                                 SECTORS_QOS)
        self.pub_scan = self.create_publisher(LaserScan, '/obstacles/scan', 5)
        self.pub_px4_obstacles = None
        if p('px4_obstacle_distance.enabled').value:
            self.pub_px4_obstacles = self.create_publisher(
                ObstacleDistance, p('px4_obstacle_distance.topic').value, 10)
            self.get_logger().warn(
                'publicando /fmu/in/obstacle_distance para Collision Prevention de PX4: si este '
                'nodo se cae, PX4 deja de permitir movimiento horizontal en Position (ver docs)')
        self.tf_broadcaster = None
        if p('publish_tf').value:
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)

        self.create_service(Trigger, '/map/save', self._srv_save, callback_group=heavy)
        self.create_service(Trigger, '/map/reset', self._srv_reset, callback_group=heavy)

        self.create_timer(1.0 / p('sectors.rate').value, self._publish_sectors)
        self.create_timer(1.0 / p('publish.map_rate').value, self._publish_map, callback_group=heavy)
        self.create_timer(p('publish.walls_period').value, self._publish_walls, callback_group=heavy)
        self.create_timer(p('publish.cloud_period').value, self._publish_cloud, callback_group=heavy)

        self.get_logger().info(
            f'mapper_node listo: grilla {cfg.resolution:.2f} m, franja {cfg.min_height:.1f} m del '
            f'piso a {cfg.max_height_above_drone:.1f} m sobre el dron (tope {cfg.max_height:.1f} m), '
            f'salida en {self._output_dir()}')

    # ------------------------------------------------------------------
    def _declare(self):
        d = self.declare_parameter
        d('px4.local_position_topic', '/fmu/out/vehicle_local_position_v1')
        d('px4.attitude_topic', '/fmu/out/vehicle_attitude')
        d('pose_timeout', 0.5)
        # Tiempos: cada nube con la pose de su captura (ver pose_buffer.py).
        d('time.use_px4_timestamps', True)   # timestamp_sample de PX4 (requiere UXRCE_DDS_SYNCT=1)
        d('time.max_clock_skew', 1.0)        # s: un stamp más lejos que esto del reloj = otro reloj
        d('time.pose_history', 2.0)          # s de poses guardadas
        d('time.max_extrapolation', 0.1)     # s: nube más nueva que la última pose: se usa ésa

        d('cloud.topics', ['/airsim_node/PX4/lidar/points/lidar'])
        d('cloud.axes', ['flu'])
        d('cloud.mount_xyz', [0.0, 0.0, -0.15])        # FRD, uno por tópico (planos de a 3)
        d('cloud.mount_rpy_deg', [0.0, 0.0, 0.0])
        d('cloud.max_rate', 10.0)                       # Hz por tópico
        d('cloud.downsample', 0.08)                     # m; 0 = sin filtro

        d('scan.topics', [''])
        d('scan.mount_xyz', [0.0, 0.0, -0.1])
        d('scan.mount_rpy_deg', [0.0, 0.0, 0.0])

        d('range.topics', [''])
        d('range.bearings_deg', [0.0])
        d('range.mount_xyz', [0.0, 0.0, 0.0])

        d('map.resolution', 0.15)
        d('map.initial_size', 40.0)
        d('map.max_size', 400.0)
        d('map.min_height', 0.3)
        d('map.max_height', 8.0)
        d('map.floor_slope_deg', 2.0)          # el piso sube d·tan(esto) (errores de actitud lejos)
        d('map.max_height_above_drone', 1.0)   # tope de la grilla 2D relativo al dron (techos)
        d('map.no_hit_free_range', 6.0)
        d('map.free_ray_bins', 360)
        d('map.sensor_fov_deg', 360.0)
        d('map.min_range', 0.5)
        d('map.weak_protect_sec', 2.0)   # s: rayos sin eco no vacían celdas vistas hace menos
        d('map.traversed_radius', 0.8)   # m alrededor del camino recorrido que se dan por conocidos
        # Caja del propio dron, cuerpo FRD [xmin, xmax, ymin, ymax, zmin, zmax]
        # (z+ abajo). Medir: hélices, patas, mástil. [] = sin filtro.
        d('self_filter.box', [-0.5, 0.5, -0.5, 0.5, -0.35, 0.4])
        # Ruido suelto: < min_points en la vecindad 3x3x3 de vóxeles de `voxel` m.
        d('outliers.voxel', 0.25)
        d('outliers.min_points', 2)          # 1 = sin filtro
        d('outliers.max_range', 8.0)         # m desde el sensor
        d('map.max_range', 40.0)
        d('map.voxel_resolution', 0.2)
        # Olvido de lo efímero: celdas/vóxeles vistos en pocas nubes y no
        # vueltos a ver en este tiempo (obstáculos que se movieron).
        d('map.forget_after', 0.0)           # s (10 = OA_DB_EXPIRE de ArduPilot); 0 = nunca
        d('map.forget_max_hits', 10)

        d('sectors.rate', 10.0)
        d('sectors.range', 15.0)
        d('sectors.band_below', 0.7)
        d('sectors.band_above', 0.7)
        d('sectors.min_points', 2)            # distancia del sector = la del k-ésimo punto
        d('sectors.min_cloud_points', 20)    # menos puntos útiles = sensor ciego (no es dato fresco)
        d('sectors.sensor_timeout', 0.4)     # s desde la captura (PX4 CP: 0.5 s)
        d('sectors.from_map', True)

        for name, value in WALL_PARAMS.items():
            d(f'walls.{name}', value)

        d('publish.map_rate', 2.0)
        d('publish.walls_period', 2.0)
        d('publish.cloud_period', 2.0)
        d('publish_tf', True)
        d('map_frame', 'map')
        d('output_dir', '~/ros_map_mapas')

        d('px4_obstacle_distance.enabled', False)
        d('px4_obstacle_distance.topic', '/fmu/in/obstacle_distance')

    def _output_dir(self):
        return os.path.expanduser(self.get_parameter('output_dir').value)

    def _subscribe_sensors(self, group):
        p = self.get_parameter
        topics = [t for t in p('cloud.topics').value if t]
        axes = list(p('cloud.axes').value) or ['flu']
        mounts = _triplets(p('cloud.mount_xyz').value, len(topics), (0.0, 0.0, 0.0))
        rpys = _triplets(p('cloud.mount_rpy_deg').value, len(topics), (0.0, 0.0, 0.0))
        for i, topic in enumerate(topics):
            mount = SensorMount(mounts[i], rpys[i], axes[min(i, len(axes) - 1)])
            self.create_subscription(
                PointCloud2, topic, lambda msg, t=topic, m=mount: self._on_cloud(t, m, msg),
                qos_profile_sensor_data, callback_group=group)
            self.get_logger().info(f'  nube: {topic} (ejes {mount.axes}, montaje {mounts[i]}, '
                                   f'rpy {rpys[i]})')

        scans = [t for t in p('scan.topics').value if t]
        scan_mounts = _triplets(p('scan.mount_xyz').value, len(scans), (0.0, 0.0, 0.0))
        scan_rpys = _triplets(p('scan.mount_rpy_deg').value, len(scans), (0.0, 0.0, 0.0))
        for i, topic in enumerate(scans):
            mount = SensorMount(scan_mounts[i], scan_rpys[i], 'flu')
            self.create_subscription(
                LaserScan, topic, lambda msg, t=topic, m=mount: self._on_scan(t, m, msg),
                qos_profile_sensor_data, callback_group=group)
            self.get_logger().info(f'  lidar 2D: {topic}')

        ranges = [t for t in p('range.topics').value if t]
        bearings = list(p('range.bearings_deg').value)
        offset = tuple(p('range.mount_xyz').value[:3]) if len(p('range.mount_xyz').value) >= 3 \
            else (0.0, 0.0, 0.0)
        if ranges and len(bearings) != len(ranges):
            self.get_logger().error('range.topics y range.bearings_deg tienen distinto largo: '
                                    'se ignoran los rangefinders')
            ranges = []
        for topic, bearing in zip(ranges, bearings):
            mount = SensorMount(offset, (0.0, 0.0, float(bearing)), 'frd')
            self.create_subscription(
                Range, topic, lambda msg, t=topic, m=mount: self._on_range(t, m, msg),
                qos_profile_sensor_data, callback_group=group)
            self.get_logger().info(f'  rangefinder: {topic} @ {bearing:+.0f}°')

        if not (topics or scans or ranges):
            self.get_logger().error('no hay ningún sensor configurado: el mapa va a quedar vacío')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _stamp_or_now(self, t, what):
        """`t` si está en la base de tiempo del reloj de ROS; si no, la hora de
        llegada (con un aviso por fuente: es un problema de configuración)."""
        now = self._now()
        if t > 0.0 and abs(t - now) <= self.get_parameter('time.max_clock_skew').value:
            return t
        if what not in self._skew_warned:
            self._skew_warned.add(what)
            self.get_logger().warn(
                f'{what}: stamp {t:.3f} s desfasado {t - now:+.3f} s del reloj del nodo; se usa '
                'la hora de llegada (¿UXRCE_DDS_SYNCT=1? ¿PTP/use_sim_time en el sensor?). '
                'Sin el instante de captura el mapa se corre al acelerar o girar.')
        return now

    def _px4_time(self, msg, what):
        if not self.get_parameter('time.use_px4_timestamps').value:
            return self._now()
        us = int(getattr(msg, 'timestamp_sample', 0) or getattr(msg, 'timestamp', 0) or 0)
        return self._stamp_or_now(us * 1e-6, what)

    def _on_position(self, msg):
        self.position_valid = bool(msg.xy_valid and msg.z_valid)
        if not self.position_valid:
            return
        t = self._px4_time(msg, 'vehicle_local_position')
        position = (float(msg.x), float(msg.y), float(msg.z))
        self.poses.add_position(t, position)
        if not self._attitude_ever:
            # Sin actitud todavía: alcanza con el rumbo para empezar.
            self.poses.add_attitude(t, [math.cos(msg.heading / 2), 0.0, 0.0,
                                        math.sin(msg.heading / 2)])
        with self.lock:
            self.position = position
            self.pose_stamp = self._now()
            if self.q is None:
                self.q = [math.cos(msg.heading / 2), 0.0, 0.0, math.sin(msg.heading / 2)]
            self.mapper.update_trajectory(self.position)

    def _on_attitude(self, msg):
        q = [float(v) for v in msg.q]
        self._attitude_ever = True
        self.poses.add_attitude(self._px4_time(msg, 'vehicle_attitude'), q)
        self.q = q

    def _pose(self):
        if self.position is None or self.q is None:
            return None
        if self._now() - self.pose_stamp > self.get_parameter('pose_timeout').value:
            return None
        return self.position, list(self.q)

    def _pose_at(self, stamp):
        """`(posición, q, stamp)` de la captura; si el stamp cae fuera del
        historial, la última pose con la hora de llegada (y un aviso)."""
        found = self.poses.lookup(stamp)
        if found is not None:
            return found[0], found[1], stamp
        pose = self._pose()
        if pose is None:
            return None
        span = self.poses.span()
        where = (f'historial {span[0]:.3f}..{span[1]:.3f}' if span else 'historial vacío')
        self.get_logger().warn(f'nube con stamp {stamp:.3f} fuera del historial de poses '
                               f'({where}): se usa la última pose y la hora de llegada',
                               throttle_duration_sec=5.0)
        return pose[0], pose[1], self._now()

    def _integrate(self, source, mount, points_sensor, stamp, min_points=None,
                   min_cloud_points=None, downsample=0.0, filter_outliers=True, fov_deg=None):
        found = self._pose_at(stamp)
        if found is None:
            return
        position, q, stamp = found
        with self.lock:
            before = self.mapper.starved_clouds
            self_hits = self.mapper.self_hits
            self.mapper.integrate_sensor(points_sensor, mount, position, q, stamp,
                                         source=source, min_points=min_points,
                                         min_cloud_points=min_cloud_points,
                                         downsample=downsample, filter_outliers=filter_outliers,
                                         fov_deg=fov_deg)
            starved = self.mapper.starved_clouds > before
            self_hits = self.mapper.self_hits - self_hits
        if self_hits > 0:
            # En vuelo estable no debería haber casi ninguno: si hay muchos, la
            # caja o el montaje (cloud.mount_*) están mal medidos.
            self.get_logger().info(f'{source}: {self_hits} puntos dentro de la caja del dron '
                                   '(self_filter.box) descartados', throttle_duration_sec=30.0)
        if starved:
            self.get_logger().warn(f'{source}: nube (casi) vacía: se trata como sensor ciego '
                                   '(no talla libre ni cuenta como dato fresco)',
                                   throttle_duration_sec=5.0)
        self._sensor_ever = True

    def _header_time(self, header, what):
        stamp = header.stamp
        return self._stamp_or_now(float(stamp.sec) + float(stamp.nanosec) * 1e-9, what)

    def _on_cloud(self, topic, mount, msg):
        now = self._now()
        if now - self._last_cloud.get(topic, 0.0) < 1.0 / self.get_parameter('cloud.max_rate').value:
            return
        self._last_cloud[topic] = now
        try:
            pts = pointcloud2_to_xyz(msg.fields, msg.data, msg.point_step, msg.width, msg.height,
                                     msg.is_bigendian)
        except ValueError as exc:
            self.get_logger().error(f'{topic}: {exc}', throttle_duration_sec=5.0)
            return
        self._integrate(topic, mount, pts, self._header_time(msg.header, topic),
                        downsample=self.get_parameter('cloud.downsample').value)

    def _on_scan(self, topic, mount, msg):
        pts = laserscan_to_xyz(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max)
        self._integrate(topic, mount, pts, self._header_time(msg.header, topic))

    def _on_range(self, topic, mount, msg):
        pts = range_to_xyz(float(msg.range), float(msg.min_range), float(msg.max_range))
        # Un rangefinder da un solo punto: con min_points > 1 nunca contaría (y
        # el filtro de puntos aislados lo tiraría).
        fov = math.degrees(float(msg.field_of_view)) if msg.field_of_view > 0.0 else 5.0
        self._integrate(topic, mount, pts, self._header_time(msg.header, topic), min_points=1,
                        min_cloud_points=0, filter_outliers=False, fov_deg=fov)

    # ------------------------------------------------------------------
    # Publicación
    # ------------------------------------------------------------------
    @staticmethod
    def _set_stamp(header, t):
        sec = math.floor(t)
        header.stamp.sec = int(sec)
        header.stamp.nanosec = min(int(round((t - sec) * 1e9)), 999999999)

    def _publish_sectors(self):
        pose = self._pose()
        if pose is None:
            return
        position, q = pose
        now = self._now()
        with self.lock:
            info = self.mapper.sector_info(now, position)
        cfg = self.mapper.cfg
        msg = ObstacleSectors()
        # Stamp = captura del dato en vivo más nuevo: la misión calcula la edad
        # con esto y la suma a la latencia del frenado.
        self._set_stamp(msg.header, info.stamp if info.fresh else now)
        msg.header.frame_id = 'ned'
        msg.angle_offset = 0.0
        msg.increment = 2.0 * math.pi / cfg.sector_bins
        msg.range_max = float(cfg.sector_range)
        msg.ranges = [float(r) for r in info.ranges]
        msg.live_ranges = [float(r) for r in info.live]
        msg.seen_age = [float(a) for a in info.seen_age]
        msg.position = Point(x=position[0], y=position[1], z=position[2])
        msg.band_below = float(cfg.sector_band_below)
        msg.band_above = float(cfg.sector_band_above)
        msg.fresh_sensor = bool(info.fresh)
        # Sin sensor fresco no se publica: la misión tiene que enterarse de que
        # se quedó ciega (y frenar), no recibir sólo la memoria del mapa.
        if info.fresh:
            self.pub_sectors.publish(msg)
            self._publish_scan(info.ranges, position)
            if self.pub_px4_obstacles is not None:
                self._publish_px4_obstacles(info)
        if self.tf_broadcaster is not None:
            self._publish_tf(position, q)

    def _publish_scan(self, ranges, position):
        """Los sectores como LaserScan en `map` ENU centrado en el dron: es la
        forma más fácil de verlos en Foxglove/RViz (no se usa para nada más)."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link_level'
        n = len(ranges)
        # LaserScan: antihorario desde +X (Este). Rumbo NED b -> ángulo ENU pi/2 - b.
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / n
        scan.angle_max = math.pi - scan.angle_increment
        scan.range_min = 0.1
        scan.range_max = float(self.mapper.cfg.sector_range)
        out = []
        for k in range(n):
            angle_enu = scan.angle_min + k * scan.angle_increment
            bearing = (math.pi / 2.0 - angle_enu) % (2.0 * math.pi)
            idx = int(round(bearing / (2.0 * math.pi / n))) % n
            r = ranges[idx]
            out.append(float(r) if math.isfinite(r) else float('inf'))
        scan.ranges = out
        self.pub_scan.publish(scan)

    def _publish_px4_obstacles(self, info):
        msg = ObstacleDistance()
        # Instante de CAPTURA (no de publicación): PX4 descuenta la edad del dato
        # en el frenado (CollisionPrevention: delay_distance += v·edad). Con
        # timestamp = 0 PX4 pondría "ahora" y ocultaría la latencia. Requiere
        # UXRCE_DDS_SYNCT=1 para que la base de tiempo coincida.
        msg.timestamp = int(info.stamp * 1e6) if info.fresh else \
            int(self.get_clock().now().nanoseconds / 1000)
        msg.frame = ObstacleDistance.MAV_FRAME_LOCAL_NED
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        # Sectores que ningún sensor miró hace poco = UINT16_MAX (desconocido):
        # con CP_GO_NO_DATA = 0 PX4 no deja moverse hacia ellos. Antes se
        # mandaban como libres (max_distance + 1).
        unknown = np.asarray(info.seen_age) > self.mapper.cfg.sector_timeout
        msg.distances = to_px4_obstacle_distance(info.ranges, self.mapper.cfg.sector_range,
                                                 self.mapper.cfg.min_range, unknown_mask=unknown)
        msg.increment = 5.0
        msg.min_distance = int(self.mapper.cfg.min_range * 100)
        msg.max_distance = int(self.mapper.cfg.sector_range * 100)
        msg.angle_offset = 0.0
        self.pub_px4_obstacles.publish(msg)

    def _publish_tf(self, position, q):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('map_frame').value
        t.child_frame_id = 'base_link'
        t.transform.translation.x = position[1]
        t.transform.translation.y = position[0]
        t.transform.translation.z = -position[2]
        x, y, z, w = px4_quat_to_enu_flu(q)
        t.transform.rotation.x, t.transform.rotation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = z, w
        level = TransformStamped()
        level.header = t.header
        level.child_frame_id = 'base_link_level'
        level.transform.translation = t.transform.translation
        level.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform([t, level])

    def _publish_map(self):
        with self.lock:
            snap = self.mapper.snapshot(self._now())
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').value
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(snap.resolution)
        msg.info.height, msg.info.width = snap.values.shape
        # ENU: x = este, y = norte. Las filas de la grilla son norte, las columnas
        # este, así que el arreglo aplanado ya está en el orden de OccupancyGrid.
        msg.info.origin.position.x = float(snap.origin_e)
        msg.info.origin.position.y = float(snap.origin_n)
        msg.info.origin.orientation.w = 1.0
        # array('b') es la vía rápida de rclpy para int8[] (una lista de Python
        # se valida elemento por elemento: segundos con mapas grandes).
        msg.data = array.array('b', snap.values.reshape(-1).astype(np.int8).tobytes())
        self.pub_map.publish(msg)

    def _publish_walls(self):
        # Copia bajo el lock, extracción afuera: los sectores (10 Hz) no esperan.
        with self.lock:
            state = self.mapper.capture_state()
        walls = self.mapper.extract_walls(state)
        took = self.mapper.last_extract_sec
        if took > 1.0:
            self.get_logger().warn(f'la extracción de paredes tardó {took:.1f} s: subir '
                                   'publish.walls_period o map.resolution', throttle_duration_sec=30)
        stamp = self.get_clock().now().to_msg()
        out = WallArray()
        out.header.stamp = stamp
        out.header.frame_id = 'ned'
        for wall in walls:
            m = WallMsg()
            m.id = int(wall.id)
            m.type = int(wall.kind)
            m.type_name = wall.kind_name
            m.points = [Point(x=float(v[0]), y=float(v[1]), z=0.0) for v in wall.vertices]
            m.length = float(wall.length)
            m.thickness = float(wall.thickness)
            m.rms_error = float(wall.rms)
            m.max_deviation = float(wall.max_deviation)
            m.height_min = float(wall.height_min)
            m.height_max = float(wall.height_max)
            m.tilt_deg = float(wall.tilt_deg)
            m.cells = int(wall.cells)
            out.walls.append(m)
        out.flat_count = sum(1 for w in walls if w.kind == FLAT)
        out.irregular_count = sum(1 for w in walls if w.kind == IRREGULAR)
        out.obstacle_count = sum(1 for w in walls if w.kind == OBSTACLE)
        self.pub_walls.publish(out)
        self.pub_markers.publish(self._markers(walls, stamp))

    def _markers(self, walls, stamp):
        """Paredes como "cercas" 3D: cada tramo es un rectángulo vertical entre la
        altura mínima y máxima medidas, coloreado por tipo."""
        frame = self.get_parameter('map_frame').value
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for wall in walls:
            low = wall.height_min if math.isfinite(wall.height_min) else 0.0
            high = wall.height_max if math.isfinite(wall.height_max) else 3.0
            verts = wall.vertices
            if wall.kind == OBSTACLE and len(verts) >= 3:
                verts = np.vstack((verts, verts[:1]))
            r, g, b = (c / 255.0 for c in WALL_COLORS[wall.kind])

            fence = Marker()
            fence.header.frame_id = frame
            fence.header.stamp = stamp
            fence.ns = 'paredes'
            fence.id = int(wall.id)
            fence.type = Marker.TRIANGLE_LIST
            fence.scale.x = fence.scale.y = fence.scale.z = 1.0
            fence.color.r, fence.color.g, fence.color.b, fence.color.a = r, g, b, 0.6
            fence.pose.orientation.w = 1.0
            for a, c in zip(verts[:-1], verts[1:]):
                p1 = Point(x=float(a[1]), y=float(a[0]), z=low)
                p2 = Point(x=float(c[1]), y=float(c[0]), z=low)
                p3 = Point(x=float(c[1]), y=float(c[0]), z=high)
                p4 = Point(x=float(a[1]), y=float(a[0]), z=high)
                fence.points.extend([p1, p2, p3, p1, p3, p4])
            array.markers.append(fence)

            label = Marker()
            label.header = fence.header
            label.ns = 'etiquetas'
            label.id = int(wall.id)
            label.type = Marker.TEXT_VIEW_FACING
            center = wall.vertices.mean(axis=0)
            label.pose.position.x = float(center[1])
            label.pose.position.y = float(center[0])
            label.pose.position.z = high + 0.4
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            text = f'P{wall.id} {wall.kind_name} {wall.length:.1f} m'
            if math.isfinite(wall.tilt_deg) and wall.tilt_deg >= 5.0:
                text += f' ({wall.tilt_deg:.0f}°)'
            label.text = text
            array.markers.append(label)
        return array

    def _publish_cloud(self):
        if self.pub_cloud.get_subscription_count() == 0:
            return
        with self.lock:
            pts = self.mapper.voxels.points(2)
            home = self.mapper.home_down or 0.0
        if len(pts) == 0:
            return
        enu = ned_to_enu(pts)
        heights = home - pts[:, 2]
        span = max(float(np.ptp(heights)), 0.5)
        level = np.clip((heights - heights.min()) / span, 0.0, 1.0)
        rgb = np.column_stack((255 * level, 80 + 100 * (1 - abs(level - 0.5) * 2), 255 * (1 - level)))
        fields, data, step = xyz_to_pointcloud2_data(enu, rgb.astype(np.uint8))
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').value
        msg.height = 1
        msg.width = len(enu)
        msg.fields = [PointField(name=n, offset=o, datatype=t, count=c) for n, o, t, c in fields]
        msg.is_bigendian = False
        msg.point_step = step
        msg.row_step = step * len(enu)
        msg.is_dense = True
        msg.data = array.array('B', data)
        self.pub_cloud.publish(msg)

    # ------------------------------------------------------------------
    # Servicios
    # ------------------------------------------------------------------
    def _srv_save(self, request, response):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        try:
            # Copia bajo el lock; paredes y archivos afuera (puede tardar segundos
            # justo cuando el dron vuelve, y la evasión necesita los sectores).
            with self.lock:
                state = self.mapper.capture_state()
            self.mapper.extract_walls(state)
            files = save_map_bundle(self._output_dir(), f'mapa_{stamp}', state,
                                    {'fecha': stamp})
        except OSError as exc:
            response.success = False
            response.message = f'no se pudo guardar el mapa: {exc}'
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = f'mapa guardado: {files[2]} (+{len(files) - 1} archivos)'
        self.get_logger().info(response.message)
        return response

    def _srv_reset(self, request, response):
        with self.lock:
            self.mapper.reset()
        self.get_logger().info('mapa borrado')
        response.success = True
        response.message = 'mapa borrado'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
