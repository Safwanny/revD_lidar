"""Origin alignment and heading wrap in the odometry scorer."""

import math

from revd_lidar_tools.odom_eval import end_error, relative, wrap


def test_wrap():
    assert math.isclose(wrap(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(wrap(-3 * math.pi / 2), math.pi / 2)


def test_relative_rotates_into_origin_frame():
    x, y, yaw = relative((1.0, 1.0, math.pi / 2), (1.0, 3.0, math.pi))
    assert math.isclose(x, 2.0) and abs(y) < 1e-12
    assert math.isclose(yaw, math.pi / 2)


def test_estimate_in_another_frame_scores_zero():
    # Same motion seen from frames that differ by an offset and a rotation.
    truth = ((5.0, -2.0, 1.0), (5.0 + 2 * math.cos(1.0),
                                -2.0 + 2 * math.sin(1.0), 2.0))
    estimate = ((0.0, 0.0, 0.0), (2.0, 0.0, 1.0))
    position, heading = end_error(*truth, *estimate)
    assert position < 1e-9 and abs(heading) < 1e-9


def test_heading_drift_shows_as_error():
    position, heading = end_error((0, 0, 0), (2, 0, 0),
                                  (0, 0, 0), (2, 0, math.radians(4)))
    assert position < 1e-9
    assert math.isclose(math.degrees(heading), 4.0)
