"""ROS 2 de mentira en memoria, sólo para verificar los nodos sin ROS instalado.

Registra en sys.modules: rclpy (+qos, node, executors, callback_groups,
exceptions), tf2_ros, builtin_interfaces, std_msgs, geometry_msgs, sensor_msgs,
nav_msgs, visualization_msgs, std_srvs, px4_msgs, uav_map_msgs.

Los mensajes usan __slots__: asignar un campo que no existe levanta
AttributeError, igual que los mensajes reales de rclpy.
"""
import sys
import types


class Clock:
    def __init__(self):
        self.t = 1000.0

    def now(self):
        clock = self

        class _T:
            nanoseconds = int(clock.t * 1e9)

            def to_msg(self_inner):
                m = Time()
                m.sec = int(clock.t)
                m.nanosec = int((clock.t % 1) * 1e9)
                return m
        return _T()


CLOCK = Clock()
BUS = {}          # topic -> list of callbacks
SERVICES = {}     # name -> callback
TIMERS = []       # (period, callback, [next])
NODES = []


def msg_class(name, fields, consts=None):
    """fields: dict nombre -> fábrica del valor por defecto."""
    def __init__(self, **kw):
        for k, f in fields.items():
            setattr(self, k, f())
        for k, v in kw.items():
            setattr(self, k, v)
    attrs = {'__slots__': tuple(fields), '__init__': __init__,
             '__repr__': lambda self: f'{name}(' + ', '.join(f'{k}={getattr(self, k)!r}' for k in fields) + ')'}
    attrs.update(consts or {})
    return type(name, (), attrs)


def f0():
    return 0.0


def i0():
    return 0


def s0():
    return ''


def b0():
    return False


def lst():
    return []


Time = msg_class('Time', {'sec': i0, 'nanosec': i0})
Header = msg_class('Header', {'stamp': Time, 'frame_id': s0})
Point = msg_class('Point', {'x': f0, 'y': f0, 'z': f0})
Vector3 = msg_class('Vector3', {'x': f0, 'y': f0, 'z': f0})
Quaternion = msg_class('Quaternion', {'x': f0, 'y': f0, 'z': f0, 'w': lambda: 1.0})
Pose = msg_class('Pose', {'position': Point, 'orientation': Quaternion})
PoseStamped = msg_class('PoseStamped', {'header': Header, 'pose': Pose})
Transform = msg_class('Transform', {'translation': Vector3, 'rotation': Quaternion})
TransformStamped = msg_class('TransformStamped', {'header': Header, 'child_frame_id': s0, 'transform': Transform})
PointField = msg_class('PointField', {'name': s0, 'offset': i0, 'datatype': i0, 'count': i0},
                       {'FLOAT32': 7})
PointCloud2 = msg_class('PointCloud2', {'header': Header, 'height': i0, 'width': i0, 'fields': lst,
                                        'is_bigendian': b0, 'point_step': i0, 'row_step': i0,
                                        'data': lambda: b'', 'is_dense': b0})
LaserScan = msg_class('LaserScan', {'header': Header, 'angle_min': f0, 'angle_max': f0,
                                    'angle_increment': f0, 'time_increment': f0, 'scan_time': f0,
                                    'range_min': f0, 'range_max': f0, 'ranges': lst, 'intensities': lst})
Range = msg_class('Range', {'header': Header, 'radiation_type': i0, 'field_of_view': f0,
                            'min_range': f0, 'max_range': f0, 'range': f0})
MapMetaData = msg_class('MapMetaData', {'map_load_time': Time, 'resolution': f0, 'width': i0,
                                        'height': i0, 'origin': Pose})
OccupancyGrid = msg_class('OccupancyGrid', {'header': Header, 'info': MapMetaData, 'data': lst})
Path = msg_class('Path', {'header': Header, 'poses': lst})
ColorRGBA = msg_class('ColorRGBA', {'r': f0, 'g': f0, 'b': f0, 'a': f0})
Marker = msg_class('Marker', {'header': Header, 'ns': s0, 'id': i0, 'type': i0, 'action': i0,
                              'pose': Pose, 'scale': Vector3, 'color': ColorRGBA, 'points': lst,
                              'text': s0, 'lifetime': Time, 'frame_locked': b0},
                   {'CUBE': 1, 'CYLINDER': 3, 'TEXT_VIEW_FACING': 9, 'TRIANGLE_LIST': 11,
                    'LINE_STRIP': 4, 'DELETEALL': 3, 'ADD': 0})
MarkerArray = msg_class('MarkerArray', {'markers': lst})

PX4 = {}
PX4['OffboardControlMode'] = msg_class('OffboardControlMode', {
    'timestamp': i0, 'position': b0, 'velocity': b0, 'acceleration': b0, 'attitude': b0,
    'body_rate': b0, 'thrust_and_torque': b0, 'direct_actuator': b0})
PX4['TrajectorySetpoint'] = msg_class('TrajectorySetpoint', {
    'timestamp': i0, 'position': lambda: [0.0] * 3, 'velocity': lambda: [0.0] * 3,
    'acceleration': lambda: [0.0] * 3, 'jerk': lambda: [0.0] * 3, 'yaw': f0, 'yawspeed': f0})
PX4['VehicleCommand'] = msg_class('VehicleCommand', {
    'timestamp': i0, 'param1': f0, 'param2': f0, 'param3': f0, 'param4': f0, 'param5': f0,
    'param6': f0, 'param7': f0, 'command': i0, 'target_system': i0, 'target_component': i0,
    'source_system': i0, 'source_component': i0, 'confirmation': i0, 'from_external': b0},
    {'VEHICLE_CMD_NAV_LAND': 21, 'VEHICLE_CMD_DO_SET_MODE': 176,
     'VEHICLE_CMD_COMPONENT_ARM_DISARM': 400})
PX4['VehicleLocalPosition'] = msg_class('VehicleLocalPosition', {
    'timestamp': i0, 'xy_valid': b0, 'z_valid': b0, 'v_xy_valid': b0, 'v_z_valid': b0,
    'x': f0, 'y': f0, 'z': f0, 'vx': f0, 'vy': f0, 'vz': f0, 'heading': f0})
PX4['VehicleStatus'] = msg_class('VehicleStatus', {'timestamp': i0, 'arming_state': i0, 'nav_state': i0},
                                 {'ARMING_STATE_DISARMED': 1, 'ARMING_STATE_ARMED': 2,
                                  'NAVIGATION_STATE_AUTO_LOITER': 4, 'NAVIGATION_STATE_OFFBOARD': 14,
                                  'NAVIGATION_STATE_AUTO_LAND': 18})
PX4['VehicleAttitude'] = msg_class('VehicleAttitude', {'timestamp': i0, 'q': lambda: [1.0, 0.0, 0.0, 0.0]})
PX4['VehicleOdometry'] = msg_class('VehicleOdometry', {
    'timestamp': i0, 'pose_frame': i0, 'position': lambda: [0.0] * 3, 'q': lambda: [1.0, 0, 0, 0],
    'velocity': lambda: [0.0] * 3}, {'POSE_FRAME_NED': 1})
PX4['ObstacleDistance'] = msg_class('ObstacleDistance', {
    'timestamp': i0, 'frame': i0, 'sensor_type': i0, 'distances': lambda: [0] * 72, 'increment': f0,
    'min_distance': i0, 'max_distance': i0, 'angle_offset': f0},
    {'MAV_FRAME_GLOBAL': 0, 'MAV_FRAME_LOCAL_NED': 1, 'MAV_FRAME_BODY_FRD': 12,
     'MAV_DISTANCE_SENSOR_LASER': 0})

import os as _o
_REPO_SRC = _o.path.join(_o.path.dirname(_o.path.dirname(_o.path.dirname(_o.path.abspath(__file__)))), 'src')
MAP_MSG_DIR = _o.path.join(_o.environ.get('UAV_SRC', _REPO_SRC), 'uav_map_msgs/msg')
_NESTED = {'std_msgs/Header': Header, 'Header': Header, 'geometry_msgs/Point': Point,
           'builtin_interfaces/Time': Time, 'geometry_msgs/Vector3': Vector3}


def _factory(typ, known):
    if typ.endswith(']'):
        return lst
    if typ in _NESTED:
        return _NESTED[typ]
    if typ in known:
        return known[typ]
    if typ.startswith(('float', 'double')):
        return f0
    if typ.startswith(('int', 'uint', 'byte', 'char')):
        return i0
    if typ == 'bool':
        return b0
    if typ == 'string':
        return s0
    raise ValueError(f'tipo desconocido en .msg: {typ}')


def load_map_msgs(directory=MAP_MSG_DIR):
    """Clases de uav_map_msgs leídas de los .msg del repo: así el ROS falso no
    queda atrás cuando se agrega un campo."""
    import os as _os
    import re as _re
    out = {}
    pending = sorted(f for f in _os.listdir(directory) if f.endswith('.msg'))
    order = sorted(pending, key=lambda f: 0 if f == 'Wall.msg' else 1)
    for fname in order:
        fields, consts = {}, {}
        for line in open(_os.path.join(directory, fname)):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            m = _re.match(r'(\S+)\s+(\w+)\s*=\s*(\S+)', line)
            if m:
                consts[m.group(2)] = int(m.group(3)) if m.group(3).lstrip('-').isdigit() else m.group(3)
                continue
            typ, name = line.split()[:2]
            fields[name] = _factory(typ, out)
        cls = fname[:-4]
        out[cls] = msg_class(cls, fields, consts)
    return out


MAP = load_map_msgs()

TriggerReq = msg_class('TriggerRequest', {})
TriggerRes = msg_class('TriggerResponse', {'success': b0, 'message': s0})
Trigger = type('Trigger', (), {'Request': TriggerReq, 'Response': TriggerRes})


class Param:
    def __init__(self, value):
        self.value = value


class Logger:
    def __init__(self, name):
        self.name = name
        self.lines = []

    def _log(self, level, msg, **kw):
        line = f'[{level}] [{self.name}] {msg}'
        self.lines.append(line)
        if level != 'INFO' or VERBOSE:
            print(line)

    def info(self, msg, **kw):
        self._log('INFO', msg)

    def warn(self, msg, **kw):
        self._log('WARN', msg)

    warning = warn

    def error(self, msg, **kw):
        self._log('ERROR', msg)

    def debug(self, msg, **kw):
        pass


VERBOSE = False


class Publisher:
    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
        self.count = 0
        self.last = None

    def publish(self, msg):
        if not isinstance(msg, self.msg_type):
            raise TypeError(f'{self.topic}: se esperaba {self.msg_type.__name__}, llegó {type(msg).__name__}')
        self.count += 1
        self.last = msg
        for cb in BUS.get(self.topic, []):
            cb(msg)

    def get_subscription_count(self):
        return len(BUS.get(self.topic, []))


class Client:
    def __init__(self, name):
        self.srv_name = name

    def service_is_ready(self):
        return self.srv_name in SERVICES

    def call_async(self, request):
        res = SERVICES[self.srv_name](request, TriggerRes())

        class Fut:
            def result(self_inner):
                return res

            def add_done_callback(self_inner, cb):
                cb(self_inner)
        return Fut()


class Node:
    def __init__(self, name):
        self._name = name
        self._params = {}
        self._overrides = OVERRIDES.get(name, {})
        self._logger = Logger(name)
        NODES.append(self)

    def declare_parameter(self, name, value=None, descriptor=None):
        self._params[name] = self._overrides.get(name, value)
        return Param(self._params[name])

    def get_parameter(self, name):
        return Param(self._params[name])

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return CLOCK

    def get_name(self):
        return self._name

    def create_publisher(self, msg_type, topic, qos, **kw):
        return Publisher(topic, msg_type)

    def create_subscription(self, msg_type, topic, cb, qos, **kw):
        def checked(msg, cb=cb, msg_type=msg_type, topic=topic):
            if not isinstance(msg, msg_type):
                raise TypeError(f'{topic}: tipo {type(msg).__name__} != {msg_type.__name__}')
            cb(msg)
        BUS.setdefault(topic, []).append(checked)

    def create_timer(self, period, cb, **kw):
        TIMERS.append([period, cb, CLOCK.t + period])

    def create_service(self, srv_type, name, cb, **kw):
        SERVICES[name] = cb

    def create_client(self, srv_type, name, **kw):
        return Client(name)

    def destroy_node(self):
        pass


OVERRIDES = {}


def spin_for(seconds, dt=0.01):
    end = CLOCK.t + seconds
    while CLOCK.t < end:
        CLOCK.t += dt
        for timer in TIMERS:
            if CLOCK.t >= timer[2]:
                timer[2] += timer[0]
                timer[1]()


def call(service):
    return SERVICES[service](TriggerReq(), TriggerRes())


def install():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class _QoS:
        def __init__(self, **kw):
            pass
    enum = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, VOLATILE=1, TRANSIENT_LOCAL=2,
                                 KEEP_LAST=1, KEEP_ALL=2)
    rclpy = mod('rclpy', init=lambda args=None: None, ok=lambda: True, shutdown=lambda: None,
                spin=lambda n: None, spin_once=lambda n, timeout_sec=0: None)
    rclpy.node = mod('rclpy.node', Node=Node)
    rclpy.qos = mod('rclpy.qos', QoSProfile=_QoS, ReliabilityPolicy=enum, DurabilityPolicy=enum,
                    HistoryPolicy=enum, qos_profile_sensor_data=_QoS())

    class _Ex:
        def __init__(self, **kw):
            pass

        def add_node(self, n):
            pass

        def spin(self):
            pass

        def shutdown(self):
            pass
    rclpy.executors = mod('rclpy.executors', MultiThreadedExecutor=_Ex,
                          ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}))
    rclpy.callback_groups = mod('rclpy.callback_groups',
                                MutuallyExclusiveCallbackGroup=lambda: None,
                                ReentrantCallbackGroup=lambda: None)
    rclpy.exceptions = mod('rclpy.exceptions',
                           ParameterUninitializedException=type('PUE', (Exception,), {}))

    class TB:
        def __init__(self, node):
            self.sent = 0

        def sendTransform(self, t):
            items = t if isinstance(t, list) else [t]
            for item in items:
                assert isinstance(item, TransformStamped)
            self.sent += 1
    mod('tf2_ros', TransformBroadcaster=TB)
    mod('builtin_interfaces'); mod('builtin_interfaces.msg', Time=Time)
    mod('std_msgs'); mod('std_msgs.msg', Header=Header)
    mod('geometry_msgs'); mod('geometry_msgs.msg', Point=Point, PoseStamped=PoseStamped,
                              TransformStamped=TransformStamped, Pose=Pose, Quaternion=Quaternion)
    mod('sensor_msgs'); mod('sensor_msgs.msg', PointCloud2=PointCloud2, PointField=PointField,
                            LaserScan=LaserScan, Range=Range)
    mod('nav_msgs'); mod('nav_msgs.msg', OccupancyGrid=OccupancyGrid, Path=Path)
    mod('visualization_msgs'); mod('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
    mod('std_srvs'); mod('std_srvs.srv', Trigger=Trigger)
    mod('px4_msgs'); mod('px4_msgs.msg', **PX4)
    mod('uav_map_msgs'); mod('uav_map_msgs.msg', **MAP)
