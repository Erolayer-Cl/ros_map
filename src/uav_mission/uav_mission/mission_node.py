#!/usr/bin/env python3
"""Nodo de misión: único dueño del flujo OFFBOARD de PX4 (v1.17, uXRCE-DDS).

Adaptador ROS de `mission_core.MissionCore`. Como en el repo de referencia,
**sólo este nodo** publica en `/fmu/in/trajectory_setpoint`,
`/fmu/in/offboard_control_mode` y `/fmu/in/vehicle_command`.

    ros2 service call /mission/start  std_srvs/srv/Trigger   # waypoints con esquive
    ros2 service call /mission/map    std_srvs/srv/Trigger   # misión de mapeo
    ros2 service call /mission/pause  std_srvs/srv/Trigger
    ros2 service call /mission/resume std_srvs/srv/Trigger
    ros2 service call /mission/cancel std_srvs/srv/Trigger   # vuelve y aterriza
    ros2 service call /mission/abort  std_srvs/srv/Trigger   # aterriza donde está

Entradas: posición y estado de PX4, `/obstacles/sectors` y `/map` de
mapper_node. Salidas: setpoints, comandos, `/mission/status`, y `/mission/path`
(el camino A* actual, para verlo en Foxglove).

Hilos: el lazo de control corre a 20 Hz en su propio grupo de callbacks; el A*
(lo único lento) corre en otro, así una búsqueda larga nunca corta el flujo de
setpoints (PX4 sale de OFFBOARD si pasa más de COM_OF_LOSS_T sin recibirlos).
"""

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from uav_map_msgs.msg import MissionStatus, ObstacleSectors, WallArray

from uav_mission.mission_core import DEFAULTS, FLYING_STATES, MissionCore, MissionType, State
from uav_mission.planner import PlanResult
from uav_wall_mapping.occupancy import GridSnapshot

NAN = float('nan')
NAN3 = [NAN, NAN, NAN]

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
MAP_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
# Igual que el publicador de mapper_node: sólo sirve el último dato.
SECTORS_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

assert {s.value for s in State} <= set(range(13)), 'State tiene que coincidir con MissionStatus'


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        # OJO con el sufijo _v1: en PX4 v1.17 VehicleStatus y VehicleLocalPosition
        # tienen MESSAGE_VERSION = 1, y el cliente uXRCE-DDS publica en
        # `<tópico>_v<versión>`. Con el nombre sin sufijo el tópico NO existe y no
        # llega nada (ni error). Si se cambia de firmware, revisar con
        # `ros2 topic list | grep fmu/out`.
        self.declare_parameter('px4.local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('px4.status_topic', '/fmu/out/vehicle_status_v1')
        self.declare_parameter('px4.target_system', 1)
        self.declare_parameter('auto_start', '')           # '' | 'waypoints' | 'mapeo'
        self.declare_parameter('map_frame', 'map')
        # s: un header.stamp de /obstacles/sectors más lejos que esto del reloj
        # del nodo viene de otro reloj; se usa la hora de llegada (con aviso).
        self.declare_parameter('time.max_clock_skew', 1.0)
        self._skew_warned = False

        self.lock = threading.RLock()
        self.core = MissionCore(param=lambda n: self.get_parameter(n).value, log=self._log)
        self.planner = self.core.make_planner()
        self._auto_done = False

        control = MutuallyExclusiveCallbackGroup()
        planning = MutuallyExclusiveCallbackGroup()
        inputs = MutuallyExclusiveCallbackGroup()

        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pub_status = self.create_publisher(MissionStatus, '/mission/status', 10)
        self.pub_path = self.create_publisher(Path, '/mission/path', 2)

        self.create_subscription(VehicleLocalPosition,
                                 self.get_parameter('px4.local_position_topic').value,
                                 self._on_position, PX4_QOS, callback_group=inputs)
        self.create_subscription(VehicleStatus, self.get_parameter('px4.status_topic').value,
                                 self._on_status, PX4_QOS, callback_group=inputs)
        self.create_subscription(ObstacleSectors, '/obstacles/sectors', self._on_sectors,
                                 SECTORS_QOS, callback_group=inputs)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS, callback_group=inputs)
        self.create_subscription(WallArray, '/walls', self._on_walls, MAP_QOS, callback_group=inputs)

        for name, handler in (('start', self._srv_start), ('map', self._srv_map),
                              ('pause', self._srv_pause), ('resume', self._srv_resume),
                              ('cancel', self._srv_cancel), ('abort', self._srv_abort)):
            self.create_service(Trigger, f'/mission/{name}', handler, callback_group=inputs)

        self.cli_map_reset = self.create_client(Trigger, '/map/reset')
        self.cli_map_save = self.create_client(Trigger, '/map/save')

        rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / rate, self._loop, callback_group=control)
        self.create_timer(0.2, self._plan, callback_group=planning)
        self.create_timer(0.5, self._publish_path, callback_group=inputs)
        self.create_timer(1.0, self._auto_start, callback_group=inputs)
        self._loop_error_logged = False

        self.get_logger().info(
            'mission_node listo. /mission/start = waypoints, /mission/map = mapeo del área '
            f'{list(self.get_parameter("map_mission.area").value)} (relativa al despegue)')

    # ------------------------------------------------------------------
    def _log(self, level, msg):
        logger = self.get_logger()
        {'info': logger.info, 'warn': logger.warn, 'error': logger.error}.get(level, logger.info)(msg)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # ------------------------------------------------------------------
    # Entradas
    # ------------------------------------------------------------------
    def _on_position(self, msg):
        with self.lock:
            self.core.update_vehicle((msg.x, msg.y, msg.z), (msg.vx, msg.vy, msg.vz),
                                     msg.heading, valid=bool(msg.xy_valid and msg.z_valid),
                                     stamp=self._now())

    def _on_status(self, msg):
        with self.lock:
            self.core.update_status(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                                    msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _sectors_stamp(self, msg):
        """Instante de captura del dato (header.stamp de mapper_node). La misión
        usa su edad para frenar antes; si viene de otro reloj, hora de llegada."""
        now = self._now()
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if abs(t - now) <= self.get_parameter('time.max_clock_skew').value:
            return min(t, now)
        if not self._skew_warned:
            self._skew_warned = True
            self.get_logger().warn(
                f'/obstacles/sectors con stamp desfasado {t - now:+.2f} s del reloj de este nodo: '
                'se usa la hora de llegada (la edad real del dato queda oculta)')
        return now

    def _on_sectors(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        shift = 0
        if abs(msg.angle_offset) > 1e-6 and len(ranges):
            shift = int(round(msg.angle_offset / msg.increment))
            ranges = np.roll(ranges, shift)
        live = np.asarray(getattr(msg, 'live_ranges', []) or [], dtype=float)
        seen_age = np.asarray(getattr(msg, 'seen_age', []) or [], dtype=float)
        if shift:
            live = np.roll(live, shift) if len(live) else live
            seen_age = np.roll(seen_age, shift) if len(seen_age) else seen_age
        stamp = self._sectors_stamp(msg)
        with self.lock:
            self.core.update_sectors(ranges, stamp, range_max=float(msg.range_max),
                                     live=live if len(live) == len(ranges) else None,
                                     seen_age=seen_age if len(seen_age) == len(ranges) else None)

    def _on_map(self, msg):
        values = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        snap = GridSnapshot(values=values, resolution=float(msg.info.resolution),
                            origin_n=float(msg.info.origin.position.y),
                            origin_e=float(msg.info.origin.position.x), stamp=self._now())
        with self.lock:
            self.core.update_map(snap)

    def _on_walls(self, msg):
        with self.lock:
            self.core.update_walls_summary(msg.flat_count, msg.irregular_count, msg.obstacle_count)

    # ------------------------------------------------------------------
    # Servicios
    # ------------------------------------------------------------------
    def _call(self, fn, response):
        with self.lock:
            ok, message = fn(self._now())
        response.success = bool(ok)
        response.message = message
        (self.get_logger().info if ok else self.get_logger().warn)(f'servicio: {message}')
        return response

    def _srv_start(self, request, response):
        return self._call(self.core.start_waypoints, response)

    def _srv_map(self, request, response):
        return self._call(self.core.start_mapping, response)

    def _srv_pause(self, request, response):
        return self._call(self.core.pause, response)

    def _srv_resume(self, request, response):
        return self._call(self.core.resume, response)

    def _srv_cancel(self, request, response):
        return self._call(self.core.cancel, response)

    def _srv_abort(self, request, response):
        return self._call(self.core.abort, response)

    def _auto_start(self):
        """`auto_start` arranca sola la misión apenas se puede (simulación)."""
        mode = self.get_parameter('auto_start').value
        if self._auto_done or mode not in ('waypoints', 'mapeo'):
            return
        with self.lock:
            fn = self.core.start_mapping if mode == 'mapeo' else self.core.start_waypoints
            ok, message = fn(self._now())
        if ok:
            self._auto_done = True
            self.get_logger().warn(f'auto_start={mode}: {message}')

    # ------------------------------------------------------------------
    # Lazo de control
    # ------------------------------------------------------------------
    def _loop(self):
        """Un ciclo. Nunca deja escapar una excepción: con el dron en el aire,
        sostener el flujo de setpoints es más importante que la maniobra."""
        try:
            with self.lock:
                self.core.step(self._now())
                stream = self.core.stream_offboard
                setpoint = self.core.setpoint
                velocity = self.core.velocity_setpoint
                commands = self.core.pop_commands()
                status = self.core.status()
            if stream and setpoint is not None:
                self._publish_offboard_mode()
                self._publish_setpoint(setpoint, velocity)
            for command in commands:
                self._execute(command)
            self._publish_status(status)
        except Exception as exc:  # noqa: BLE001 - a propósito
            if not self._loop_error_logged:
                self.get_logger().error(f'error en el lazo de control, se pasa a HOLD: {exc!r}')
                self._loop_error_logged = True
            with self.lock:
                if self.core.state in FLYING_STATES:
                    self.core._enter(State.HOLD, f'error interno: {exc!r}')
                    if self.core.setpoint is not None:
                        self._publish_offboard_mode()
                        self._publish_setpoint(self.core.setpoint, None)

    def _plan(self):
        """A* fuera del lazo de control (ver docstring del módulo)."""
        with self.lock:
            request = self.core.planning_request(self._now())
        if request is None:
            return
        try:
            result = self.planner.plan(request['snapshot'], request['start'], request['goal'])
        except Exception as exc:  # noqa: BLE001
            result = PlanResult(reason=f'excepción en A*: {exc!r}')
        with self.lock:
            self.core.planning_result(request['id'], result, self._now())

    def _execute(self, command):
        if command == 'mode_offboard':
            # param1 = 1 (custom mode), param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif command == 'arm':
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif command == 'land':
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif command == 'map_reset':
            self._call_mapper(self.cli_map_reset, 'borrar el mapa')
        elif command == 'map_save':
            self._call_mapper(self.cli_map_save, 'guardar el mapa')

    def _call_mapper(self, client, what):
        if not client.service_is_ready():
            self.get_logger().error(f'no se pudo {what}: el servicio {client.srv_name} no está '
                                    '(¿corre mapper_node?)')
            return
        future = client.call_async(Trigger.Request())

        def done(fut):
            try:
                res = fut.result()
                (self.get_logger().info if res.success else self.get_logger().error)(
                    f'{what}: {res.message}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'no se pudo {what}: {exc!r}')
        future.add_done_callback(done)

    # ------------------------------------------------------------------
    # Publicadores
    # ------------------------------------------------------------------
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.pub_offboard.publish(msg)

    def _publish_setpoint(self, setpoint, velocity):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [float(setpoint[0]), float(setpoint[1]), float(setpoint[2])]
        # Velocidad horizontal como feedforward (la zanahoria se mueve a esa
        # velocidad); la vertical y la aceleración las resuelve PX4 (NaN).
        msg.velocity = ([float(velocity[0]), float(velocity[1]), NAN] if velocity is not None
                        else NAN3)
        msg.acceleration = NAN3
        msg.jerk = NAN3
        msg.yaw = float(setpoint[3])
        msg.yawspeed = NAN
        self.pub_setpoint.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1, msg.param2 = float(param1), float(param2)
        msg.target_system = int(self.get_parameter('px4.target_system').value)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_command.publish(msg)

    def _publish_status(self, s):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ned'
        msg.state = s['state']
        msg.state_name = s['state_name']
        msg.mission_type = s['mission_type']
        msg.message = s['message']
        msg.waypoint_index = s['waypoint_index']
        msg.waypoint_count = s['waypoint_count']
        msg.goal_kind = s['goal_kind']
        for field in ('distance_to_goal', 'distance_to_home', 'distance_traveled',
                      'tracking_error', 'altitude', 'ground_speed', 'obstacle_distance',
                      'obstacle_bearing', 'coverage'):
            setattr(msg, field, float(s[field]))
        msg.path_points = s['path_points']
        msg.plan_status = s['plan_status']
        msg.avoiding = bool(s['avoiding'])
        msg.avoid_reason = s['avoid_reason']
        msg.frontiers = s['frontiers']
        msg.goals_reached = s['goals_reached']
        msg.walls_flat = s['walls_flat']
        msg.walls_irregular = s['walls_irregular']
        msg.obstacles = s['obstacles']
        self.pub_status.publish(msg)

    def _publish_path(self):
        with self.lock:
            path = list(self.core._path or [])
            goal = self.core._goal
            home = self.core.home
            active = self.core.state in FLYING_STATES and self.core.mission != MissionType.NONE
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').value
        if active and goal is not None:
            altitude = -goal[2] + (home[2] if home else 0.0)
            points = path if path else [goal[:2]]
            for n, e in points:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = float(e)
                pose.pose.position.y = float(n)
                pose.pose.position.z = float(altitude)
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
        self.pub_path.publish(msg)

    def land_on_shutdown(self):
        """Ctrl-C con el dron armado: pedir LAND antes de morir (si no, el
        failsafe de PX4 por pérdida de offboard decide, y puede ser un RTL)."""
        with self.lock:
            armed = self.core.armed
        if not armed:
            return
        self.get_logger().warn('cerrando con el dron armado: mandando LAND')
        for _ in range(5):
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except BaseException as exc:
        # Se atrapa todo a propósito: si el nodo muere por lo que sea, el dron
        # sigue armado en el aire y hay que bajarlo (lección del repo de referencia).
        node.land_on_shutdown()
        if not isinstance(exc, (KeyboardInterrupt, rclpy.executors.ExternalShutdownException)):
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
