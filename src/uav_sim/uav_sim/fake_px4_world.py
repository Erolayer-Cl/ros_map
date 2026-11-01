#!/usr/bin/env python3
"""PX4 + Unreal de mentira, con ROS: para probar el stack completo sin GPU.

Igual que `fake_px4` del repo de referencia, pero con un lidar 3D que ve el
mismo escenario YAML que después se crea en Unreal. Publica exactamente los
tópicos que publicarían PX4 v1.17 (por uXRCE-DDS) y `airsim_node` de
Cosys-AirSim, así que mapper_node y mission_node corren sin cambiar nada:

- `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1`,
  `/fmu/out/vehicle_attitude`, `/fmu/out/vehicle_odometry`
- `/airsim_node/PX4/lidar/points/lidar` (PointCloud2, marco del sensor, FLU)
- `/fake_world/markers`: la geometría real del escenario, para comparar en
  Foxglove contra lo que dibuja el mapper.

    ros2 launch uav_sim fake_sim.launch.py escenario:=06_habitacion
    ros2 service call /mission/map std_srvs/srv/Trigger

No simula física real, latencia de uXRCE-DDS ni preflight checks: que una
misión ande acá no garantiza que ande en Unreal, pero si se rompe acá se va a
romper allá.
"""

import array
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleAttitude,
                          VehicleCommand, VehicleLocalPosition, VehicleOdometry, VehicleStatus)

from uav_sim.scenario import load_scenario
from uav_sim.world import LidarModel, SimVehicle, World
from uav_wall_mapping.cloud import xyz_to_pointcloud2_data
from uav_wall_mapping.geometry import rot_to_quat_xyzw

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


class FakePX4World(Node):

    def __init__(self):
        super().__init__('fake_px4_world')
        d = self.declare_parameter
        d('escenario', '01_pared_plana')
        d('lidar.topic', '/airsim_node/PX4/lidar/points/lidar')
        d('lidar.frame_id', 'PX4/lidar')
        d('lidar.rate', 10.0)
        d('lidar.channels', 16)
        d('lidar.vertical_min_deg', -7.0)
        d('lidar.vertical_max_deg', 52.0)
        d('lidar.horizontal_samples', 360)
        d('lidar.range', 40.0)
        d('lidar.mount_xyz', [0.0, 0.0, -0.15])
        d('lidar.mount_rpy_deg', [0.0, 0.0, 0.0])
        d('lidar.noise', 0.01)
        d('vehicle.max_speed', 4.0)
        d('vehicle.max_accel', 3.0)
        d('vehicle.start_yaw_deg', 0.0)
        d('offboard_timeout', 1.0)     # como COM_OF_LOSS_T de PX4

        p = self.get_parameter
        self.scenario = load_scenario(p('escenario').value)
        self.world = World(self.scenario)
        self.lidar = LidarModel(
            channels=p('lidar.channels').value,
            vertical_min_deg=p('lidar.vertical_min_deg').value,
            vertical_max_deg=p('lidar.vertical_max_deg').value,
            horizontal_samples=p('lidar.horizontal_samples').value,
            max_range=p('lidar.range').value,
            mount_frd=tuple(p('lidar.mount_xyz').value),
            mount_rpy_deg=tuple(p('lidar.mount_rpy_deg').value),
            noise_std=p('lidar.noise').value)
        self.vehicle = SimVehicle(yaw=math.radians(p('vehicle.start_yaw_deg').value),
                                  max_speed=p('vehicle.max_speed').value,
                                  max_accel=p('vehicle.max_accel').value)
        self.rng = np.random.default_rng(0)
        self.setpoint = None
        self.velocity_ff = None
        self.last_offboard = -math.inf
        self.min_clearance = math.inf
        self.collision_warned = False

        self.pub_pos = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', PX4_QOS)
        self.pub_status = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status_v1', PX4_QOS)
        self.pub_att = self.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', PX4_QOS)
        self.pub_odom = self.create_publisher(VehicleOdometry, '/fmu/out/vehicle_odometry', PX4_QOS)
        self.pub_cloud = self.create_publisher(PointCloud2, p('lidar.topic').value, 5)
        self.pub_world = self.create_publisher(MarkerArray, '/fake_world/markers', LATCHED)

        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self._on_setpoint, PX4_QOS)
        self.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', self._on_command, PX4_QOS)
        self.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode',
                                 self._on_offboard, PX4_QOS)

        self.dt = 0.02
        self.create_timer(self.dt, self._step)
        self.create_timer(1.0 / p('lidar.rate').value, self._scan)
        self._publish_world()
        self.get_logger().info(
            f'fake_px4_world: escenario {self.scenario.name} ({len(self.scenario.boxes)} cajas, '
            f'{len(self.scenario.cylinders)} cilindros). {self.scenario.description}')

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _stamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_setpoint(self, msg):
        if all(math.isfinite(v) for v in msg.position):
            self.setpoint = (float(msg.position[0]), float(msg.position[1]),
                             float(msg.position[2]), float(msg.yaw))
        vel = [float(v) for v in msg.velocity[:2]]
        self.velocity_ff = vel if all(math.isfinite(v) for v in vel) else None

    def _on_offboard(self, msg):
        self.last_offboard = self._now()

    def _on_command(self, msg):
        v = self.vehicle
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE and int(msg.param2) == 6:
            if self._now() - self.last_offboard < 0.5:
                v.offboard = True
            else:
                self.get_logger().warn('OFFBOARD rechazado: no llegan offboard_control_mode')
        elif msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            v.armed = msg.param1 > 0.5
        elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            if not v.landing:
                self.get_logger().info(f'LAND en N={v.position[0]:.1f} E={v.position[1]:.1f}')
            v.landing = True
            v.offboard = False

    def _step(self):
        v = self.vehicle
        if v.offboard and self._now() - self.last_offboard > self.get_parameter('offboard_timeout').value:
            # Failsafe de PX4 por pérdida de offboard: acá, quedarse quieto.
            self.get_logger().warn('failsafe: se cortó offboard_control_mode, saliendo de OFFBOARD')
            v.offboard = False
            self.setpoint = (v.position[0], v.position[1], v.position[2], v.yaw)
        v.step(self.dt, self.setpoint if v.armed else None,
               self.velocity_ff if v.offboard else None)
        if v.armed and -v.position[2] > 0.3:
            clear = self.world.clearance(v.position)
            self.min_clearance = min(self.min_clearance, clear)
            if clear < 0.3 and not self.collision_warned:
                self.get_logger().error(f'¡CHOQUE! a {clear:.2f} m de un obstáculo en '
                                        f'N={v.position[0]:.1f} E={v.position[1]:.1f}')
                self.collision_warned = True
        stamp = self._stamp_us()

        pos = VehicleLocalPosition()
        pos.timestamp = stamp
        pos.xy_valid = pos.z_valid = pos.v_xy_valid = pos.v_z_valid = True
        pos.x, pos.y, pos.z = (float(c) for c in v.position)
        pos.vx, pos.vy, pos.vz = (float(c) for c in v.velocity)
        pos.heading = float(v.yaw)
        self.pub_pos.publish(pos)

        att = VehicleAttitude()
        att.timestamp = stamp
        att.q = [float(c) for c in v.q]
        self.pub_att.publish(att)

        odom = VehicleOdometry()
        odom.timestamp = stamp
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.position = [float(c) for c in v.position]
        odom.q = [float(c) for c in v.q]
        odom.velocity = [float(c) for c in v.velocity]
        self.pub_odom.publish(odom)

        status = VehicleStatus()
        status.timestamp = stamp
        status.arming_state = (VehicleStatus.ARMING_STATE_ARMED if v.armed
                               else VehicleStatus.ARMING_STATE_DISARMED)
        status.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if v.offboard
                            else VehicleStatus.NAVIGATION_STATE_AUTO_LAND if v.landing
                            else VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)
        self.pub_status.publish(status)

    def _scan(self):
        v = self.vehicle
        pts = self.lidar.scan(self.world, v.position, v.q, self.rng)
        fields, data, step = xyz_to_pointcloud2_data(pts)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('lidar.frame_id').value
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [PointField(name=n, offset=o, datatype=t, count=c) for n, o, t, c in fields]
        msg.is_bigendian = False
        msg.point_step = step
        msg.row_step = step * len(pts)
        msg.is_dense = True
        msg.data = array.array('B', data)
        self.pub_cloud.publish(msg)

    def _publish_world(self):
        """La geometría real, en `map` ENU, semitransparente."""
        array = MarkerArray()
        ned_to_enu = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        for i, (center, rot, half) in enumerate(self.world.boxes):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'mundo_real'
            m.id = i
            m.type = Marker.CUBE
            m.pose.position = Point(x=float(center[1]), y=float(center[0]), z=float(-center[2]))
            # Rotación de la caja expresada en ENU (ned_to_enu es su propia inversa).
            x, y, z, w = rot_to_quat_xyzw(ned_to_enu @ rot @ ned_to_enu)
            m.pose.orientation.x, m.pose.orientation.y = x, y
            m.pose.orientation.z, m.pose.orientation.w = z, w
            # Ejes locales NED (largo, ancho, alto) -> ENU (ancho, largo, alto)
            m.scale.x, m.scale.y, m.scale.z = float(2 * half[1]), float(2 * half[0]), float(2 * half[2])
            m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.6, 0.6, 0.35
            array.markers.append(m)
        for j, cyl in enumerate(self.world.cylinders):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'mundo_real'
            m.id = 1000 + j
            m.type = Marker.CYLINDER
            height = cyl.bottom_d - cyl.top_d
            m.pose.position = Point(x=float(cyl.center_ne[1]), y=float(cyl.center_ne[0]),
                                    z=float(-(cyl.bottom_d + cyl.top_d) / 2.0))
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = float(2 * cyl.radius)
            m.scale.z = float(height)
            m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.6, 0.6, 0.35
            array.markers.append(m)
        self.pub_world.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FakePX4World()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if math.isfinite(node.min_clearance):
            node.get_logger().info(f'distancia mínima a obstáculos en todo el vuelo: '
                                   f'{node.min_clearance:.2f} m')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
