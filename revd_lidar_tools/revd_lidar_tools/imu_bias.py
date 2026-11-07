#!/usr/bin/env python3
"""Remove gyro z bias, learnt whenever the robot stands still.

robot_localization has no gyro-bias state, so any bias left in /imu/data
integrates straight into heading. While the wheels are stopped the true yaw
rate is zero, so what the gyro reads then is bias plus noise. This node
averages it over every stop and publishes /imu/data_unbiased with the bias
subtracted; the EKF fuses that. The BNO085 does the same on-chip, but CEVA
warns that slow ground-robot turns can fool it, so this backs it up on the
real robot too.

Standstill is taken from the wheel odometry twist, with a settle time after
each stop. Samples whose rate is too high to be bias (someone lifting or
turning the robot by hand) are ignored.
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class BiasEstimator:
    """Running mean of gyro z over standstill samples."""

    def __init__(self, settle_time, min_samples, window, max_rate):
        self.settle_time = settle_time
        self.min_samples = min_samples
        # ponytail: a running mean that turns into an exponential average
        # after `window` samples, so a slowly drifting bias is followed. A
        # temperature model would replace it if hardware drift demands it.
        self.window = window
        self.max_rate = max_rate
        self.bias = 0.0
        self.samples = 0
        self.still_since = None

    def wheels(self, time, still):
        """Record whether the wheels are stopped at `time` (seconds)."""
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = time

    def gyro(self, time, rate):
        """Learn from one gyro z sample if standing still; return it unbiased."""
        if (self.still_since is not None
                and time - self.still_since >= self.settle_time
                and abs(rate) <= self.max_rate):
            self.samples += 1
            self.bias += (rate - self.bias) / min(self.samples, self.window)
        return rate - self.correction()

    def correction(self):
        """The bias in use; zero until enough standstill has been seen."""
        return self.bias if self.samples >= self.min_samples else 0.0


class ImuBias(Node):
    """Relay /imu/data to /imu/data_unbiased with gyro z bias removed."""

    def __init__(self):
        super().__init__('imu_bias')
        self.declare_parameter('enabled', True)
        # Wheel twist below this is standstill (m/s and rad/s).
        self.declare_parameter('still_speed', 1e-3)
        self.declare_parameter('settle_time', 0.5)
        # 5 s at 100 Hz: the mean's noise is then half the BNO08x bias spec.
        self.declare_parameter('min_samples', 500)
        # 5 min of standstill at 100 Hz.
        self.declare_parameter('window', 30000)
        # 0.02 rad/s is about 1 deg/s, well above any calibrated BNO085 bias.
        self.declare_parameter('max_rate', 0.02)
        value = self.get_parameter
        self.enabled = value('enabled').value
        self.still_speed = value('still_speed').value
        self.estimator = BiasEstimator(
            value('settle_time').value, value('min_samples').value,
            value('window').value, value('max_rate').value)
        self.publisher = self.create_publisher(
            Imu, '/imu/data_unbiased', qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/mobile_base_controller/odometry', self.on_wheels, 10)

    def on_wheels(self, message):
        twist = message.twist.twist
        still = max(abs(twist.linear.x), abs(twist.linear.y),
                    abs(twist.angular.z)) < self.still_speed
        self.estimator.wheels(self.seconds(message), still)

    def on_imu(self, message):
        if self.enabled:
            message.angular_velocity.z = self.estimator.gyro(
                self.seconds(message), message.angular_velocity.z)
        self.publisher.publish(message)

    @staticmethod
    def seconds(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def main(args=None):
    """Run the gyro bias relay."""
    rclpy.init(args=args)
    node = ImuBias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
