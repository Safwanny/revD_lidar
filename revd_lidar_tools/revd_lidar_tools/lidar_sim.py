#!/usr/bin/env python3
"""Make the simulated D500 scan read like the real STL-19P.

Gazebo's gpu_lidar gives exact ranges. This node sits where the real driver
will, /scan_raw -> /scan, and adds the datasheet's range noise, banded by
range, and drops returns past the target's maximum range. A dropped beam is
+inf, the LaserScan convention for no return (REP 117). The beam count is
left alone: SLAM needs a fixed one. Sim only. Numbers and sources:
config/lidar_sim.yaml.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def degrade(ranges, rng, bands, sigmas, max_range):
    """Return a copy of ``ranges`` with banded noise and far returns dropped.

    ``bands`` are the upper range edges of all but the last band;
    ``sigmas`` has one more entry than ``bands``. Misses (non-finite) and
    noisy ranges past ``max_range`` come out as +inf.
    """
    ranges = np.array(ranges, dtype=np.float64)
    hit = np.isfinite(ranges)
    sigma = np.asarray(sigmas)[np.digitize(np.where(hit, ranges, 0.0), bands)]
    noisy = ranges + rng.normal(0.0, sigma)
    noisy[~hit | (noisy > max_range)] = math.inf
    return noisy


class LidarSim(Node):
    """Republish /scan_raw with STL-19P range noise and range limit."""

    def __init__(self):
        super().__init__('lidar_sim')
        self.declare_parameter('seed', 1)
        self.declare_parameter('bands', [0.5, 2.0])
        self.declare_parameter('sigmas', [0.002, 0.004, 0.015])
        self.declare_parameter('max_range', 12.0)
        value = self.get_parameter
        self.bands = list(value('bands').value)
        self.sigmas = list(value('sigmas').value)
        if len(self.sigmas) != len(self.bands) + 1:
            raise ValueError('sigmas needs one more entry than bands')
        self.max_range = value('max_range').value
        self.rng = np.random.default_rng(value('seed').value)
        # Reliable out, so both best-effort and reliable readers match.
        self.publisher = self.create_publisher(LaserScan, '/scan', 5)
        self.create_subscription(
            LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, message):
        message.ranges = degrade(message.ranges, self.rng, self.bands,
                                 self.sigmas, self.max_range).tolist()
        message.range_max = float(self.max_range)
        self.publisher.publish(message)


def main(args=None):
    """Run the LiDAR sensor model."""
    rclpy.init(args=args)
    node = LidarSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
