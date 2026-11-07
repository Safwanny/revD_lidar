"""Banded range noise and the range limit of the D500 model."""

import math

import numpy as np

from revd_lidar_tools.lidar_sim import degrade

BANDS = [0.5, 2.0]
SIGMAS = [0.002, 0.004, 0.015]


def test_noise_follows_the_bands():
    rng = np.random.default_rng(1)
    for distance, sigma in ((0.3, 0.002), (1.0, 0.004), (5.0, 0.015)):
        out = degrade(np.full(20000, distance), rng, BANDS, SIGMAS, 12.0)
        assert abs(out.mean() - distance) < 3 * sigma / math.sqrt(len(out))
        assert abs(out.std() - sigma) < 0.05 * sigma


def test_misses_and_far_returns_are_inf():
    rng = np.random.default_rng(1)
    out = degrade([math.inf, math.nan, 9.0, 3.0], rng, BANDS, SIGMAS, 8.0)
    assert out[0] == math.inf and out[1] == math.inf and out[2] == math.inf
    assert math.isfinite(out[3])


def test_seeded():
    a = degrade([1.0, 4.0], np.random.default_rng(7), BANDS, SIGMAS, 12.0)
    b = degrade([1.0, 4.0], np.random.default_rng(7), BANDS, SIGMAS, 12.0)
    assert np.array_equal(a, b)
