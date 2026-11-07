#!/usr/bin/env python3
"""Slow every velocity command so the robot can always stop before a hit.

Sits between twist_mux and the controller: /cmd_vel_mux ->
/mobile_base_controller/reference. The points it reacts to are the ToF rows
of all four faces, kept for a few seconds in the odom frame, because near the
hull corners no face sees anything (each face covers only +-30 deg).

Only motion that closes on a point is limited, and only by what lies along
that motion: the hull, grown by ``min_gap`` along the motion and by
``side_min_gap`` across it, is swept along the commanded direction and the
speed is the highest that still stops before the first point it meets. So
the clearance holds whatever the angle it meets a wall at, and it can still
slide along a wall it stopped ``min_gap`` from. Moving away from anything is never limited, so the robot can always
leave a place it got into. Points already inside the hull are taken as noise.
Close to stopping it still creeps, and zero speed only ever blocks the one
direction that would touch. Numbers and sources: config/safety.yaml.
"""

from collections import deque
import math
from types import SimpleNamespace

from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points

# Outward normal of each face in the base frame.
NORMALS = {'front': (1.0, 0.0), 'rear': (-1.0, 0.0),
           'left': (0.0, 1.0), 'right': (0.0, -1.0)}


def stopping_speed(distance, p):
    """Return the highest speed that still stops within ``distance``.

    Travel during ``latency`` at full speed, then braking at ``brake_decel``:
    distance = v * latency + v^2 / (2 * brake_decel). Never below
    ``creep_speed`` until the distance is used up; zero from there.
    """
    if distance <= 0.0:
        return 0.0
    a, t = p.brake_decel, p.latency
    return max(p.creep_speed, a * (math.sqrt(t * t + 2.0 * distance / a) - t))


def path_distance(points, direction, half):
    """Return how far the hull moves along ``direction`` before a point.

    ``points`` is N x 2 in the base frame, ``direction`` a unit vector and
    ``half`` the hull's half length and width. A point is hit once
    |p - s * direction| <= half on both axes; s is found per axis and the
    ranges intersected. Points inside the hull now, or only behind it, are
    ignored. inf when nothing is in the way.
    """
    u = np.asarray(direction, dtype=float)
    h = np.asarray(half, dtype=float)
    with np.errstate(divide='ignore', invalid='ignore'):
        first, second = (points - h) / u, (points + h) / u
    lo, hi = np.minimum(first, second), np.maximum(first, second)
    # Along an axis it does not move on, the hull covers a point always or
    # never.
    inside = np.abs(points) <= h
    still = u == 0.0
    lo = np.where(still, np.where(inside, -np.inf, np.inf), lo)
    hi = np.where(still, np.where(inside, np.inf, -np.inf), hi)
    enter, leave = lo.max(axis=1), hi.min(axis=1)
    hits = (enter > 0.0) & (enter <= leave)
    return float(enter[hits].min()) if hits.any() else math.inf


def beside(points, direction, half, gap, ahead):
    """Return True if a point is within ``gap`` of the hull's swept sides.

    The sides run from the back of the hull to ``ahead`` past its front.
    Points in the swept path itself are left to ``path_distance``.
    """
    u = np.asarray(direction, dtype=float)
    n = np.array([-u[1], u[0]])
    along, across = points @ u, np.abs(points @ n)
    reach, width = np.abs(u) @ half, np.abs(n) @ half
    return bool(np.any((across > width) & (across <= width + gap)
                       & (along >= -reach) & (along <= reach + ahead)))


def turn_angle(points, half, sign, step=math.radians(1.0)):
    """Return how far the hull turns (direction ``sign``) before a point.

    Only points between the hull and its corner circle can be hit. Turning
    the hull by +angle is turning the points by -angle. Checked in ``step``
    increments and rounded down; inf when nothing can be hit.
    """
    h = np.asarray(half, dtype=float)
    r = np.hypot(points[:, 0], points[:, 1])
    near = points[(r <= np.hypot(*h)) & ~np.all(np.abs(points) <= h, axis=1)]
    if not len(near):
        return math.inf
    angles = np.arange(step, math.pi + step, step)[:, None]
    c, s = np.cos(sign * angles), np.sin(sign * angles)
    x = c * near[:, 0] + s * near[:, 1]
    y = -s * near[:, 0] + c * near[:, 1]
    hit = np.any((np.abs(x) <= h[0]) & (np.abs(y) <= h[1]), axis=1)
    return float(angles[hit.argmax(), 0] - step) if hit.any() else math.inf


def govern(vx, vy, wz, points, ages, p):
    """Return (vx, vy, wz) scaled down to what is safe; direction is kept.

    ``points`` N x 2 in the base frame, ``ages`` seconds since each face's
    last cloud. A face silent past ``slow_after`` lets motion toward it only
    creep; past ``stale_after`` that motion and turning are blocked and
    everything else creeps.
    """
    half = (p.half_length, p.half_width)
    speed = math.hypot(vx, vy)
    v_cap, w_cap = p.max_speed, p.max_angular
    for face, age in ages.items():
        toward = vx * NORMALS[face][0] + vy * NORMALS[face][1] > 0.0
        if age > p.stale_after:
            v_cap = min(v_cap, 0.0 if toward else p.creep_speed)
            w_cap = 0.0
        elif age > p.slow_after and toward:
            v_cap = min(v_cap, p.creep_speed)
    if speed > 0.0:
        u = (vx / speed, vy / speed)
        grown = tuple(h + p.side_min_gap + (p.min_gap - p.side_min_gap)
                      * abs(ui) for h, ui in zip(half, u))
        distance = path_distance(points, u, grown)
        # A point already within min_gap stops only motion that closes on it.
        close = points[np.all(np.abs(points) <= grown, axis=1)]
        if path_distance(close, u, half) < math.inf:
            distance = 0.0
        v_cap = min(v_cap, stopping_speed(distance, p))
        # The side zone runs as far ahead as the stop at full speed.
        ahead = p.max_speed * p.latency \
            + p.max_speed ** 2 / (2.0 * p.brake_decel)
        if beside(points, u, half, p.side_gap, ahead):
            v_cap = min(v_cap, p.side_factor * p.max_speed)
    if wz:
        # The corners move fastest; limit their arc like a straight path.
        corner = math.hypot(*half)
        angle = turn_angle(points, half, math.copysign(1.0, wz))
        w_cap = min(w_cap,
                    stopping_speed(angle * corner - p.min_gap, p) / corner)
    scale = min(1.0, v_cap / speed if speed else 1.0,
                w_cap / abs(wz) if wz else 1.0)
    return vx * scale, vy * scale, wz * scale


def spin_clear(points, half, margin):
    """Return True if no point outside the hull is within its corner circle.

    That circle plus ``margin`` is what a standing spin sweeps.
    """
    h = np.asarray(half, dtype=float)
    outside = ~np.all(np.abs(points) <= h, axis=1)
    r = np.hypot(points[:, 0], points[:, 1])
    return not np.any(outside & (r <= np.hypot(*h) + margin))


def axis_only(vx, vy, wz, points, p):
    """Snap a command to what ToF-only mode allows.

    Translation only along the sensing axes (the larger of vx, vy), with at
    most ``trim_angular`` of heading trim; a standing spin only when
    ``spin_clear``.
    """
    if vx or vy:
        if abs(vx) >= abs(vy):
            vy = 0.0
        else:
            vx = 0.0
        wz = max(-p.trim_angular, min(p.trim_angular, wz))
    elif wz and not spin_clear(points, (p.half_length, p.half_width),
                               p.spin_margin):
        wz = 0.0
    return vx, vy, wz


def to_base(odom, pose):
    """Return N x 2 odom points in the base frame at ``pose`` (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (odom - (x, y)) @ np.array([[c, -s], [s, c]])


class ToFMemory:
    """The ToF rows of all faces, kept ``memory`` seconds in odom.

    Up close no face sees the hull corners (each covers only +-30 deg), so
    what the faces saw a moment ago stands in for them.
    """

    def __init__(self, node, p):
        self.node, self.p = node, p
        self.tf = Buffer()
        TransformListener(self.tf, node)
        self.memory = deque()  # (seconds, N x 2 points in odom)
        self.seen = {face: -math.inf for face in p.faces}
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for face in p.faces:
            node.create_subscription(
                PointCloud2, '/tof/{}/rows'.format(face),
                lambda message, face=face: self.on_cloud(face, message), qos)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def warn(self, error):
        self.node.get_logger().warn(str(error), throttle_duration_sec=5.0)

    def on_cloud(self, face, message):
        self.seen[face] = self.now()
        try:
            transform = self.tf.lookup_transform(
                self.p.odom_frame, message.header.frame_id,
                message.header.stamp)
        except TransformException:
            try:
                transform = self.tf.lookup_transform(
                    self.p.odom_frame, message.header.frame_id, Time())
            except TransformException as error:
                self.warn(error)
                return
        # Points only: do_transform_cloud rebuilds the cloud and fails on
        # Gazebo's padded point layout.
        xyz = transform_points(point_cloud2.read_points_numpy(
            message, field_names=('x', 'y', 'z'), skip_nans=True),
            transform.transform)
        # odom z is height above the floor (the EKF runs in 2D).
        keep = (xyz[:, 2] >= self.p.min_height) \
            & (xyz[:, 2] <= self.p.max_height)
        self.memory.append((self.now(), xyz[keep, :2]))

    def pose(self):
        """Return (x, y, yaw) of the base in odom, or None without TF."""
        try:
            t = self.tf.lookup_transform(
                self.p.odom_frame, self.p.base_frame, Time()).transform
        except TransformException as error:
            self.warn(error)
            return None
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def points(self, pose):
        """Return the kept points as N x 2 in the base frame at ``pose``."""
        now = self.now()
        while self.memory and now - self.memory[0][0] > self.p.memory:
            self.memory.popleft()
        if not self.memory:
            return np.empty((0, 2))
        return to_base(np.concatenate([xy for _, xy in self.memory]), pose)

    def ages(self):
        now = self.now()
        return {face: now - seen for face, seen in self.seen.items()}


class SafetyGovernor(Node):
    """Limit /cmd_vel_mux by the ToF ring and pass it to the controller."""

    def __init__(self):
        super().__init__('safety_governor')
        defaults = {
            'faces': list(NORMALS), 'base_frame': 'base_footprint',
            'odom_frame': 'odom', 'half_length': 0.15, 'half_width': 0.11,
            'max_speed': 0.45, 'max_angular': 2.5, 'brake_decel': 1.5,
            'latency': 0.18, 'min_gap': 0.03, 'side_min_gap': 0.01,
            'creep_speed': 0.03,
            'side_gap': 0.05, 'side_factor': 0.6, 'min_height': 0.02,
            'max_height': 0.20, 'memory': 3.0, 'slow_after': 0.2,
            'stale_after': 0.75, 'tof_only': False, 'trim_angular': 0.2,
            'spin_margin': 0.02}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = SimpleNamespace(**{name: self.get_parameter(name).value
                                    for name in defaults})
        self.tof = ToFMemory(self, self.p)
        self.output = self.create_publisher(
            TwistStamped, '/mobile_base_controller/reference', 10)
        self.create_subscription(
            TwistStamped, '/cmd_vel_mux', self.on_command, 10)

    def on_command(self, message):
        out = TwistStamped()
        out.header.frame_id = message.header.frame_id
        # Our own clock, not the input's: the controller times a command out
        # by its stamp, and a sender on another clock (wall time against sim
        # time) would make every command look fresh forever.
        out.header.stamp = self.get_clock().now().to_msg()
        pose = self.tof.pose()
        if pose is None:
            # No pose, no memory: publish zero rather than drive blind.
            self.output.publish(out)
            return
        points = self.tof.points(pose)
        twist = message.twist
        command = (twist.linear.x, twist.linear.y, twist.angular.z)
        if self.p.tof_only:
            command = axis_only(*command, points, self.p)
        (out.twist.linear.x, out.twist.linear.y,
         out.twist.angular.z) = govern(*command, points, self.tof.ages(),
                                       self.p)
        self.output.publish(out)


def main(args=None):
    """Run the safety governor."""
    rclpy.init(args=args)
    node = SafetyGovernor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
