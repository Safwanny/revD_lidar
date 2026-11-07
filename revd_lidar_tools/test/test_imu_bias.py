"""Standstill gyro bias estimation."""

import random

from revd_lidar_tools.imu_bias import BiasEstimator

BIAS = 1.5e-4


def estimator():
    return BiasEstimator(settle_time=0.5, min_samples=500, window=30000,
                         max_rate=0.02)


def run(e, start, seconds, still, rate=lambda: BIAS):
    e.wheels(start, still)
    out = []
    for i in range(int(seconds * 100)):
        out.append(e.gyro(start + i / 100, rate()))
    return out


def test_learns_bias_at_standstill():
    random.seed(1)
    e = estimator()
    run(e, 0.0, 30.0, True, lambda: BIAS + random.gauss(0, 1.7e-3))
    assert abs(e.correction() - BIAS) < 5e-5


def test_no_correction_before_enough_standstill():
    e = estimator()
    out = run(e, 0.0, 5.0, True)  # 0.5 s settle leaves 450 samples
    assert e.correction() == 0.0 and out[-1] == BIAS


def test_motion_does_not_teach_the_bias():
    e = estimator()
    run(e, 0.0, 10.0, True)
    run(e, 10.0, 10.0, False, lambda: 0.5)  # turning at 0.5 rad/s
    assert abs(e.correction() - BIAS) < 1e-9


def test_settle_time_after_each_stop():
    e = estimator()
    run(e, 0.0, 10.0, True)
    e.wheels(10.0, False)
    out = run(e, 10.01, 0.4, True, lambda: 0.01)  # still settling
    assert abs(e.correction() - BIAS) < 1e-9
    assert abs(out[-1] - (0.01 - BIAS)) < 1e-9


def test_rejects_rates_too_high_for_bias():
    e = estimator()
    run(e, 0.0, 10.0, True)
    run(e, 10.0, 5.0, True, lambda: 0.1)  # turned by hand, wheels free
    assert abs(e.correction() - BIAS) < 1e-9
