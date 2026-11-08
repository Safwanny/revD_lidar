#!/usr/bin/env python3
"""Drive a mapping route in sim, record trajectories, save the SLAM map.

Stands in for a human on teleop: follows the world's waypoints in
slam_routes.yaml on /cmd_vel_teleop, so every command still passes the
safety chain. It steers by /ground_truth/odom so the route is the same on
every run whatever the estimate does; evaluation only, like odom_eval.

Writes to output_dir, at 10 Hz as TUM files (for evo_ape / evo_rpe):
truth.tum (world frame), <estimate>.tum (map -> base_footprint, from
slam_toolbox or a localizer) and ekf.tum (/odometry/filtered, the no-SLAM
baseline). At the end, with save_map, it calls /slam_toolbox/save_map and
/slam_toolbox/serialize_map to write map.pgm + map.yaml + map.posegraph +
map.data; then route.json, and exits 0. A waypoint not reached within its
timeout exits 1.
"""

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

SPEED = 0.3
TURN_GAIN = 1.5
MAX_TURN = 0.6
REACHED = 0.15
# Generous: the governor may slow the robot near walls and in doorways.
TIMEOUT_PER_METRE = 15.0
TUM_PERIOD = 0.1


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def command(pose, target):
    """Body-frame (vx, vy, wz) towards target, facing the way of travel."""
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    distance = math.hypot(dx, dy)
    speed = min(SPEED, distance) / max(distance, 1e-9)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    vx, vy = speed * (c * dx + s * dy), speed * (-s * dx + c * dy)
    wz = max(-MAX_TURN, min(MAX_TURN,
                            TURN_GAIN * wrap(math.atan2(dy, dx) - pose[2])))
    return vx, vy, wz


class SlamEval(Node):

    def __init__(self):
        super().__init__('slam_eval')
        self.declare_parameter('world', 'navigation_basic')
        self.declare_parameter('output_dir', 'slam_eval')
        self.declare_parameter('estimate', 'slam')
        self.declare_parameter('save_map', True)
        self.declare_parameter('routes', str(
            Path(get_package_share_directory('revd_lidar_tools'))
            / 'config' / 'slam_routes.yaml'))
        routes = yaml.safe_load(Path(
            self.get_parameter('routes').value).read_text(encoding='utf-8'))
        self.waypoints = list(
            routes[self.get_parameter('world').value]['waypoints'])
        self.output = Path(self.get_parameter('output_dir').value).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.estimate = self.get_parameter('estimate').value
        self.save_map = self.get_parameter('save_map').value
        self.files = {name: open(self.output / f'{name}.tum', 'w')
                      for name in ('truth', self.estimate, 'ekf')}
        self.last_written = {}
        self.truth = None
        self.path = 0.0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 self.on_truth, 20)
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda m: self.write('ekf', m.header.stamp, m.pose.pose), 20)
        self.publisher = self.create_publisher(
            TwistStamped, '/cmd_vel_teleop', 10)
        self.save_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.serialize_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.result = None
        self.start = None
        self.create_timer(0.05, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def write(self, name, stamp, pose):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.result is not None or t - self.last_written.get(name, -1.0) < TUM_PERIOD:
            return
        self.last_written[name] = t
        p, q = pose.position, pose.orientation
        self.files[name].write(
            f'{t:.6f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n')

    def on_truth(self, message):
        p = message.pose.pose.position
        pose = (p.x, p.y, yaw_of(message.pose.pose.orientation))
        if self.truth is not None:
            self.path += math.hypot(pose[0] - self.truth[0],
                                    pose[1] - self.truth[1])
        self.truth = pose
        self.write('truth', message.header.stamp, message.pose.pose)
        try:
            transform = self.tf.lookup_transform(
                'map', 'base_footprint', Time())
        except TransformException:
            return
        t, r = transform.transform.translation, transform.transform.rotation
        pose = type(message.pose.pose)()
        pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
        pose.orientation = r
        self.write(self.estimate, transform.header.stamp, pose)

    def publish(self, vx, vy, wz):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self.publisher.publish(message)

    def step(self):
        if self.truth is None or self.result is not None:
            return
        if self.start is None:
            if not self.tf.can_transform('map', 'base_footprint', Time()):
                return
            self.get_logger().info(f'{len(self.waypoints)} waypoints')
            self.start = self.now()
            self.begin_leg()
        target = self.waypoints[0]
        if math.hypot(target[0] - self.truth[0],
                      target[1] - self.truth[1]) < REACHED:
            self.waypoints.pop(0)
            if not self.waypoints:
                self.finish(True)
                return
            self.begin_leg()
            return
        if self.now() > self.deadline:
            self.get_logger().error(
                f'waypoint {target} not reached from {self.truth[:2]}')
            self.finish(False)
            return
        self.publish(*command(self.truth, target))

    def begin_leg(self):
        target = self.waypoints[0]
        length = math.hypot(target[0] - self.truth[0],
                            target[1] - self.truth[1])
        self.deadline = self.now() + 10.0 + TIMEOUT_PER_METRE * length

    def finish(self, reached):
        self.publish(0.0, 0.0, 0.0)
        self.result = {'reached': reached, 'path_m': round(self.path, 2),
                       'sim_time_s': round(self.now() - self.start, 1)}
        for handle in self.files.values():
            handle.close()
        self.get_logger().info(f'route done: {self.result}')
        if not self.save_map:
            self.done()
            return
        self.save_attempts = 0
        self.save_client.wait_for_service(timeout_sec=10.0)
        self.request_save()

    def request_save(self):
        self.save_attempts += 1
        request = SaveMap.Request()
        request.name.data = str(self.output / 'map')
        self.save_client.call_async(request).add_done_callback(self.saved)

    def saved(self, future):
        self.result['map_saved'] = future.result().result == 0
        # slam_toolbox makes a new map saver per call, and it gives up if
        # /map has not reached it within 2 s, which the first call often
        # misses; a second call works.
        if not self.result['map_saved'] and self.save_attempts < 3:
            self.get_logger().warn('save_map failed; retrying')
            self.request_save()
            return
        # The pose graph, for slam_toolbox localization mode.
        request = SerializePoseGraph.Request()
        request.filename = str(self.output / 'map')
        self.serialize_client.wait_for_service(timeout_sec=10.0)
        self.serialize_client.call_async(request).add_done_callback(
            self.serialized)

    def serialized(self, future):
        self.result['map_saved'] &= future.result().result == 0
        self.done()

    def done(self):
        (self.output / 'route.json').write_text(
            json.dumps(self.result, indent=2) + '\n', encoding='utf-8')
        self.get_logger().info(json.dumps(self.result))
        raise SystemExit(0 if self.result['reached']
                         and self.result.get('map_saved', True) else 1)


def main():
    rclpy.init()
    node = SlamEval()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as stop:
        code = stop.code
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
