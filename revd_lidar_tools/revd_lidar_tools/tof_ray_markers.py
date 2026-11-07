#!/usr/bin/env python3
"""Draw ToF zone rays as RViz line markers, hits and misses alike.

The point clouds the ToF sensors publish carry only the zones that returned
something, which is exactly the wrong picture for judging coverage: the zones
that saw nothing are invisible, and a fan that covers half of what you assumed
looks identical to one that covers all of it.

So this node draws the grid rather than the cloud. Every zone gets a segment
whether or not it returned, running from the sensor's minimum range out to the
point of contact, or out to maximum range where there was no contact. The near
dead zone is deliberately left blank: nothing is measured inside it, so drawing
a line there would claim coverage the part does not have.

Hits and misses are drawn in different colours because the distinction is the
one that matters on a real robot. A drop-off does not read as a long range - it
reads as no return at all, the same as an open doorway.

Every sensor goes into one MarkerArray published on a timer, rather than one
array per sensor published on arrival, so a dropped message costs a frame of
staleness everywhere at once instead of leaving one sensor behind the rest.

The markers are stamped zero on purpose, which asks RViz for the latest
available transform. The clouds can arrive stamped slightly ahead of the newest
odom to base_footprint transform; RViz drops a marker it cannot transform at its
stamp, and the ring blinks. The ray geometry is exact in the sensor's own frame
regardless, so the stamp only decides where the robot is drawn.
"""

import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Rays are context, not content: just enough to show the fan, never enough to
# hide what is behind it.
RAY_HIT = ColorRGBA(r=0.784, g=0.824, b=0.863, a=0.11)
RAY_MISS = ColorRGBA(r=0.353, g=0.392, b=0.439, a=0.055)


def zone_directions(zones, half_span):
    """Return unit ray directions for a zones x zones grid, row major.

    The angles are zone centres, not field edges: with `zones` samples across
    the field of view the outermost ray sits half a zone inside the edge, which
    is what `half_span` already accounts for.

    Row and column both run from -half_span upward, which is the order Gazebo
    fills the cloud in: row 0 is the bottom of the grid, not the top, and the
    ring field on each point carries that same row index. Getting this
    backwards mirrors the miss rays about the horizon, which is worse than it
    sounds - the misses are what a drop-off looks like, so an inverted grid
    draws the cliff into the ceiling.
    """
    if zones < 2:
        steps = [0.0]
    else:
        step = 2.0 * half_span / (zones - 1)
        steps = [-half_span + index * step for index in range(zones)]
    directions = []
    for pitch in steps:
        for yaw in steps:
            directions.append((
                math.cos(pitch) * math.cos(yaw),
                math.cos(pitch) * math.sin(yaw),
                math.sin(pitch),
            ))
    return directions


def scaled(direction, distance):
    """Return a Point at `distance` along a unit `direction`."""
    point = Point()
    point.x = direction[0] * distance
    point.y = direction[1] * distance
    point.z = direction[2] * distance
    return point


class ToFRayMarkers(Node):
    """Publish one LINE_LIST marker per ToF sensor, one segment per zone."""

    def __init__(self):
        super().__init__('tof_ray_markers')
        self.declare_parameter(
            'cloud_topics',
            [
                '/tof/front/rows',
                '/tof/rear/rows',
                '/tof/left/rows',
                '/tof/right/rows',
            ],
        )
        self.declare_parameter('marker_topic', '/tof/rays')
        self.declare_parameter('zones', 8)
        self.declare_parameter('field_of_view', math.radians(60.0))
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('line_width', 0.001)
        # How far a ray that saw nothing is drawn. The sensor reaches further,
        # but nothing past the marking range is acted on, and 2 m spokes on
        # every miss bury the returns that matter.
        self.declare_parameter('miss_length', 1.0)
        self.declare_parameter('publish_rate', 15.0)

        self.zones = self.get_parameter('zones').value
        field_of_view = self.get_parameter('field_of_view').value
        zone_angle = field_of_view / self.zones
        grid = zone_directions(
            self.zones, field_of_view / 2.0 - zone_angle / 2.0)
        # Each cloud carries only its face's kept rows (see tof_rows), so each
        # face draws only those rows of the grid.
        self.directions = {}
        self.rows = {}
        for topic in self.get_parameter('cloud_topics').value:
            face = face_of(topic)
            self.declare_parameter('rows.' + face, list(range(self.zones)))
            self.rows[face] = self.get_parameter('rows.' + face).value
            self.directions[face] = [
                grid[row * self.zones + column]
                for row in self.rows[face]
                for column in range(self.zones)]
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.line_width = self.get_parameter('line_width').value
        self.miss_length = self.get_parameter('miss_length').value

        # Sensor data is best effort, and the bridge republishes it as such.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.latest = {}
        self.subscriptions_by_topic = {}
        for index, topic in enumerate(
                self.get_parameter('cloud_topics').value):
            self.subscriptions_by_topic[topic] = self.create_subscription(
                PointCloud2,
                topic,
                lambda message, marker_id=index, name=topic:
                    self.latest.__setitem__(marker_id, (name, message)),
                qos,
            )
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self.publish_rays)

    def zone_endpoints(self, message, expected):
        """Return one contact point per zone, None where the zone missed.

        Gazebo delivers an organised cloud, one point per zone in grid order,
        with misses left non-finite. A cloud that arrives filtered to hits only
        cannot be indexed back onto the grid, so it is reported as all misses
        rather than silently mapping the wrong ranges onto the wrong zones.

        Contact points are passed through as measured rather than rebuilt from
        the zone index. Only the miss rays are drawn along a reconstructed
        direction, so if the grid ordering here ever disagreed with Gazebo's,
        the hits would still land where the sensor actually saw them.
        """
        points = list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=False))
        if len(points) != expected:
            return [None] * expected
        endpoints = []
        for x, y, z in points:
            distance = math.sqrt(x * x + y * y + z * z)
            if not math.isfinite(distance) or distance <= self.min_range:
                endpoints.append(None)
            else:
                endpoints.append(Point(x=float(x), y=float(y), z=float(z)))
        return endpoints

    def sensor_markers(self, topic, message, marker_id):
        """Turn one sensor's cloud into its ray marker."""
        face = face_of(topic)
        frame = message.header.frame_id or 'tof_{}_link'.format(face)
        rays = self.marker(frame, 'tof_rays', marker_id, Marker.LINE_LIST)
        rays.scale.x = self.line_width
        directions = self.directions[face]
        for direction, endpoint in zip(
                directions, self.zone_endpoints(message, len(directions))):
            rays.points.append(scaled(direction, self.min_range))
            if endpoint is None:
                rays.points.append(scaled(direction, self.miss_length))
                colour = RAY_MISS
            else:
                rays.points.append(endpoint)
                colour = RAY_HIT
            rays.colors.append(colour)
            rays.colors.append(colour)
        return rays

    @staticmethod
    def marker(frame, namespace, marker_id, kind):
        """A marker in ``frame``, stamped zero so RViz uses the latest TF."""
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_rays(self):
        """Publish the whole ring as one array, on the timer."""
        if not self.latest:
            return
        markers = []
        for marker_id, (topic, message) in sorted(self.latest.items()):
            markers.append(self.sensor_markers(topic, message, marker_id))
        self.publisher.publish(MarkerArray(markers=markers))


def face_of(topic):
    """'/tof/front/rows' -> 'front'."""
    return topic.strip('/').split('/')[-2]


def main(args=None):
    """Run the ToF ray marker node."""
    rclpy.init(args=args)
    node = ToFRayMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
