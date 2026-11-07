#!/usr/bin/env python3
"""Keep only the chosen zone rows of each ToF sensor.

The VL53L7CX always reports its whole 8 x 8 grid; there is no per-zone
readout. Most rows on a face that leans down just see floor, so this node
slices the organised cloud on /tof/<face>/points down to the rows listed in
``rows.<face>`` and republishes it on /tof/<face>/rows. Row 0 is the bottom
of the grid, the order Gazebo fills it in.

The output stays organised (height = kept rows, width = zones) and keeps every
field, so a consumer can still map a point back onto its zone.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

FACES = ('front', 'rear', 'left', 'right')


def keep_rows(message, zones, rows):
    """Return a copy of an organised zones x zones cloud with only ``rows``."""
    row_bytes = message.point_step * zones
    if len(message.data) != row_bytes * zones:
        return None
    data = bytes(message.data)
    out = PointCloud2()
    out.header = message.header
    out.fields = message.fields
    out.is_bigendian = message.is_bigendian
    out.point_step = message.point_step
    out.is_dense = message.is_dense
    out.height = len(rows)
    out.width = zones
    out.row_step = row_bytes
    out.data = b''.join(data[r * row_bytes:(r + 1) * row_bytes] for r in rows)
    return out


class ToFRows(Node):
    """Republish each face's cloud cut down to its configured rows."""

    def __init__(self):
        super().__init__('tof_rows')
        self.declare_parameter('faces', list(FACES))
        self.declare_parameter('zones', 8)
        self.zones = self.get_parameter('zones').value
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for face in self.get_parameter('faces').value:
            self.declare_parameter('rows.' + face, list(range(self.zones)))
            rows = list(self.get_parameter('rows.' + face).value)
            # Reliable out, so both best-effort and reliable readers match.
            publisher = self.create_publisher(
                PointCloud2, '/tof/{}/rows'.format(face), 5)
            self.create_subscription(
                PointCloud2, '/tof/{}/points'.format(face),
                lambda message, rows=rows, publisher=publisher:
                    self.on_cloud(message, rows, publisher),
                qos)

    def on_cloud(self, message, rows, publisher):
        out = keep_rows(message, self.zones, rows)
        if out is None:
            self.get_logger().warn(
                'cloud on {} is not a {} x {} grid, dropped'.format(
                    message.header.frame_id, self.zones, self.zones),
                throttle_duration_sec=5.0)
            return
        publisher.publish(out)


def main(args=None):
    """Run the ToF row selector."""
    rclpy.init(args=args)
    node = ToFRows()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
