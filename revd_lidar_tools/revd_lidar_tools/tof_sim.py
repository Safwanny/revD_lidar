#!/usr/bin/env python3
"""Make the simulated VL53L7CX clouds read like the real part.

Gazebo's gpu_lidar gives exact ranges and only constant Gaussian noise. The
real part, at 8 x 8 / 15 Hz, has noise that grows with range and misses
zones: its driver drops every zone whose target status is not valid, and the
datasheet's ranges are where 90 % of frames still detect the target. This
node sits where the real driver will, /tof/<face>/points_raw ->
/tof/<face>/points, and adds both. A zone with no valid target becomes NaN,
which is what the real driver publishes for it, and the grid stays organised.
Sim only. Numbers and sources: config/tof_sim.yaml.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

FACES = ('front', 'rear', 'left', 'right')


def degrade(xyz, rng, sigma_near, sigma_relative, dropout_near,
            dropout_far, max_range):
    """Return an N x 3 copy of ``xyz`` with range noise and dropped zones.

    Noise is along each ray, sigma = max(sigma_near, sigma_relative * range).
    A zone drops with a probability rising linearly from ``dropout_near`` at
    the sensor to ``dropout_far`` at ``max_range``. Misses (non-finite) and
    drops come out as NaN.
    """
    xyz = np.array(xyz, dtype=np.float64)
    with np.errstate(invalid='ignore'):
        ranges = np.linalg.norm(xyz, axis=1)
        hit = np.isfinite(ranges) & (ranges > 0.0)
    ranges = np.where(hit, ranges, max_range)
    sigma = np.maximum(sigma_near, sigma_relative * ranges)
    noisy = ranges + rng.normal(0.0, sigma)
    xyz *= (noisy / ranges)[:, None]
    p_drop = dropout_near + (dropout_far - dropout_near) * np.clip(
        ranges / max_range, 0.0, 1.0)
    xyz[~hit | (rng.random(len(ranges)) < p_drop)] = np.nan
    return xyz


class ToFSim(Node):
    """Republish each face's cloud with range noise and zone dropout."""

    def __init__(self):
        super().__init__('tof_sim')
        self.declare_parameter('faces', list(FACES))
        self.declare_parameter('seed', 1)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('sigma_near', 0.0045)
        self.declare_parameter('sigma_relative', 0.021)
        self.declare_parameter('dropout_near', 0.01)
        self.declare_parameter('dropout_far', 0.10)
        self.args = [self.get_parameter(name).value for name in (
            'sigma_near', 'sigma_relative', 'dropout_near', 'dropout_far',
            'max_range')]
        self.rng = np.random.default_rng(self.get_parameter('seed').value)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for face in self.get_parameter('faces').value:
            # Reliable out, so both best-effort and reliable readers match.
            publisher = self.create_publisher(
                PointCloud2, '/tof/{}/points'.format(face), 5)
            self.create_subscription(
                PointCloud2, '/tof/{}/points_raw'.format(face),
                lambda message, publisher=publisher:
                    publisher.publish(self.on_cloud(message)),
                qos)

    def on_cloud(self, message):
        # Edit x, y, z in place: Gazebo pads its points, so the layout is
        # kept byte for byte rather than rebuilt.
        points = np.frombuffer(bytearray(message.data), dtype=point_cloud2.
                               dtype_from_fields(message.fields,
                                                 message.point_step))
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1)
        points['x'], points['y'], points['z'] = degrade(
            xyz, self.rng, *self.args).T
        message.data = points.tobytes()
        message.is_dense = False
        return message


def main(args=None):
    """Run the ToF sensor model."""
    rclpy.init(args=args)
    node = ToFSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
