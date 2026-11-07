#!/usr/bin/env python3
"""Drive fixed paths in sim and score odometry against Gazebo ground truth.

Runs a standstill, three laps of a 2 m square (drive, stop, turn on the
spot), three laps of the same square strafed while rotating and a turn on
the spot, then prints end position and heading errors for the EKF
(/odometry/filtered) and the wheels alone (/mobile_base_controller/odometry)
against 2 % of the path and 3 deg. Each run is aligned by its starting pose only, so heading drift
is not fitted away. Poses are also written at 10 Hz as TUM files under
output_dir, for evo_ape / evo_traj and odom_eval_plot.

"EKF beats wheel-only" is judged on strafe_rotate only. The square is made of
on-the-spot turns, the very motion the wheel constant was calibrated on in
this sim, so there it only checks the calibration against itself.

The turn calibrates that constant and is not scored: it reports the wheel
turn over the true turn. Multiply
kinematics.sum_of_robot_center_projection_on_X_Y_axis in controllers.yaml by
that ratio, rebuild and run again until it reads 1.000 +- 0.005.

Evaluation only: this is the one node allowed to read /ground_truth/odom.
Spawn the robot with room for the square, e.g. x:=-4.0 y:=-3.2 in
navigation_basic.
"""

import math
from pathlib import Path

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

SOURCES = {
    'truth': '/ground_truth/odom',
    'ekf': '/odometry/filtered',
    'wheel': '/mobile_base_controller/odometry',
}
SIDE = 2.0
SPEED = 0.2
TURN_RATE = 0.5
LAPS = 3
POSITION_LIMIT = 0.02
HEADING_LIMIT = math.radians(3.0)
STANDSTILL_LIMIT = math.radians(1.0)
STANDSTILL_TIME = 60.0
SETTLE = 1.0
TUM_PERIOD = 0.1
# Runs off the wheel calibration path, where wheel-only is a fair reference.
BEATS_RUNS = ('strafe_rotate',)
# Under half a turn, so the wrapped heading is the whole turn.
CALIBRATION_TURN = 2.5


def wrap(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def relative(origin, pose):
    """Pose (x, y, yaw) expressed in the frame of origin (x, y, yaw)."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2]))


def end_error(truth_origin, truth_end, estimate_origin, estimate_end):
    """End position error (m) and heading error (rad), origin-aligned."""
    t = relative(truth_origin, truth_end)
    e = relative(estimate_origin, estimate_end)
    return math.hypot(e[0] - t[0], e[1] - t[1]), wrap(e[2] - t[2])


def square():
    """(seconds, vx, vy, wz, frame) segments: drive, stop, turn left."""
    leg = [(SIDE / SPEED, SPEED, 0.0, 0.0, 'body'), (SETTLE, 0, 0, 0, 'body'),
           (math.pi / 2 / TURN_RATE, 0.0, 0.0, TURN_RATE, 'body'),
           (SETTLE, 0, 0, 0, 'body')]
    return leg * 4 * LAPS


def strafe_rotate():
    """The square in the odom frame, turning 90 deg along each side."""
    time = SIDE / SPEED
    legs = []
    for side in range(4):
        heading = side * math.pi / 2
        legs += [(time, SPEED * math.cos(heading), SPEED * math.sin(heading),
                  math.pi / 2 / time, 'odom'), (SETTLE, 0, 0, 0, 'body')]
    return legs * LAPS


RUNS = [
    ('standstill', [(STANDSTILL_TIME, 0.0, 0.0, 0.0, 'body')]),
    ('square', square()),
    ('strafe_rotate', strafe_rotate()),
    # Last: the square and strafe must start at the spawn heading, or the
    # square's first side runs into the wall.
    ('turn', [(CALIBRATION_TURN / TURN_RATE, 0.0, 0.0, TURN_RATE, 'body'),
              (SETTLE, 0, 0, 0, 'body')]),
]


def pose_of(message):
    """(x, y, yaw) of an Odometry message."""
    p, q = message.pose.pose.position, message.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (p.x, p.y, yaw)


class OdomEval(Node):
    """Command the runs, record the three sources, report the scores."""

    def __init__(self):
        super().__init__('odom_eval')
        self.declare_parameter('output_dir', 'odom_eval')
        self.output = Path(self.get_parameter('output_dir').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.latest = dict.fromkeys(SOURCES)
        self.files = {}
        for name, topic in SOURCES.items():
            self.create_subscription(
                Odometry, topic,
                lambda message, name=name: self.on_odometry(name, message), 20)
        self.command = self.create_publisher(
            TwistStamped, '/mobile_base_controller/reference', 10)
        self.runs = list(RUNS)
        self.segments = []
        self.results = []
        self.passed = None
        self.turn_ratio = None
        self.create_timer(0.02, self.step)

    def on_odometry(self, name, message):
        self.latest[name] = message
        if name in self.files:
            if name == 'truth':
                pose = pose_of(message)
                self.path += math.hypot(pose[0] - self.last_truth[0],
                                        pose[1] - self.last_truth[1])
                self.last_truth = pose
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if stamp - self.last_written.get(name, -1.0) < TUM_PERIOD:
                return
            self.last_written[name] = stamp
            p, q = message.pose.pose.position, message.pose.pose.orientation
            self.files[name].write(
                f'{stamp:.6f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def step(self):
        if None in self.latest.values() or self.passed is not None:
            return
        if not self.segments:
            if self.results or self.files:
                self.finish_run()
            if not self.runs:
                self.report()
                return
            self.start_run(*self.runs.pop(0))
        duration, vx, vy, wz, frame = self.segments[0]
        if self.now() - self.segment_start >= duration:
            self.segments.pop(0)
            self.segment_start = self.now()
            return
        if frame == 'odom':
            # World-frame motion turned into the body frame with the robot's
            # own estimate, as a real holonomic driver would.
            yaw = pose_of(self.latest['ekf'])[2]
            vx, vy = (math.cos(yaw) * vx + math.sin(yaw) * vy,
                      -math.sin(yaw) * vx + math.cos(yaw) * vy)
        self.publish(vx, vy, wz)

    def publish(self, vx, vy, wz):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self.command.publish(message)

    def start_run(self, name, segments):
        self.get_logger().info(f'run {name}')
        self.run_name = name
        self.segments = list(segments) + [(2.0, 0, 0, 0, 'body')]
        self.segment_start = self.run_start = self.now()
        self.origins = {k: pose_of(m) for k, m in self.latest.items()}
        self.path = 0.0
        self.last_truth = self.origins['truth']
        self.files = {k: open(self.output / f'{name}_{k}.tum', 'w')
                      for k in SOURCES}
        self.last_written = {}

    def finish_run(self):
        for handle in self.files.values():
            handle.close()
        self.files = {}
        minutes = (self.now() - self.run_start) / 60.0
        ends = {k: pose_of(m) for k, m in self.latest.items()}
        if self.run_name == 'turn':
            turn = {k: relative(self.origins[k], ends[k])[2]
                    for k in ('truth', 'wheel')}
            self.turn_ratio = turn['wheel'] / turn['truth']
            return
        for source in ('ekf', 'wheel'):
            position, heading = end_error(
                self.origins['truth'], ends['truth'],
                self.origins[source], ends[source])
            self.results.append((self.run_name, source, self.path, position,
                                 heading, minutes))

    def report(self):
        lines = ['run            source  path m  pos err  % path  head deg'
                 '  verdict']
        passed = True
        by_run = {}
        for run, source, path, position, heading, minutes in self.results:
            by_run.setdefault(run, {})[source] = (position, heading)
            if run == 'standstill':
                rate = abs(heading) / minutes
                ok = source == 'wheel' or rate < STANDSTILL_LIMIT
                verdict = f'{math.degrees(rate):.2f} deg/min ' + (
                    'ok' if ok else 'FAIL')
            else:
                ok = (position <= POSITION_LIMIT * path
                      and abs(heading) <= HEADING_LIMIT) or source == 'wheel'
                verdict = 'ok' if ok else 'FAIL'
            if source == 'wheel':
                verdict = 'reference'
            passed &= ok
            lines.append(
                f'{run:<14} {source:<6} {path:7.2f} {position:8.3f} '
                f'{100 * position / max(path, 1e-9):7.2f} '
                f'{math.degrees(heading):9.2f}  {verdict}')
        for run, scores in by_run.items():
            if run not in BEATS_RUNS:
                continue
            beats = (scores['ekf'][0] < scores['wheel'][0]
                     and abs(scores['ekf'][1]) < abs(scores['wheel'][1]))
            passed &= beats
            lines.append(f'{run}: EKF beats wheel-only: '
                         + ('yes' if beats else 'NO'))
        if self.turn_ratio is not None:
            lines.append(f'turn: wheel / true = {self.turn_ratio:.4f} '
                         '(multiply sum_of_robot_center_projection_on_X_Y_'
                         'axis by it)')
        lines.append('PASS' if passed else 'FAIL')
        self.get_logger().info('\n' + '\n'.join(lines))
        (self.output / 'report.txt').write_text('\n'.join(lines) + '\n')
        self.passed = passed


def main(args=None):
    """Run the evaluation and exit 0 only if every threshold holds."""
    rclpy.init(args=args)
    node = OdomEval()
    try:
        while rclpy.ok() and node.passed is None:
            rclpy.spin_once(node)
        node.publish(0.0, 0.0, 0.0)
    except KeyboardInterrupt:
        pass
    passed = node.passed
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
