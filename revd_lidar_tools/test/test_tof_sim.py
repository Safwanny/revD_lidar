#!/usr/bin/env python3

# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The ToF sensor model: noise along the ray, dropout rising with range."""

import numpy as np
from revd_lidar_tools.tof_sim import degrade

ARGS = dict(sigma_near=0.0045, sigma_relative=0.021, dropout_near=0.01,
            dropout_far=0.10, max_range=2.0)


def rays(distance, count=20000):
    """``count`` returns straight ahead at ``distance``."""
    return np.tile([distance, 0.0, 0.0], (count, 1))


def test_noise_is_along_the_ray_and_grows_with_range():
    rng = np.random.default_rng(0)
    for distance, sigma in ((0.1, 0.0045), (1.5, 0.0315)):
        out = degrade(rays(distance), rng, **ARGS)
        kept = out[np.isfinite(out[:, 0])]
        assert np.all(kept[:, 1:] == 0.0)
        assert abs(kept[:, 0].mean() - distance) < 0.1 * sigma
        assert abs(kept[:, 0].std() - sigma) < 0.05 * sigma


def test_dropout_rises_from_near_to_far():
    rng = np.random.default_rng(0)
    near = np.isnan(degrade(rays(0.0), rng, **ARGS)[:, 0]).mean()
    far = np.isnan(degrade(rays(2.0), rng, **ARGS)[:, 0]).mean()
    assert near == 1.0  # zero range is no target, not a return
    assert abs(np.isnan(degrade(rays(0.02), rng, **ARGS)[:, 0]).mean()
               - 0.0109) < 0.005
    assert abs(far - 0.10) < 0.01


def test_misses_stay_misses():
    out = degrade([[np.inf, 0.0, 0.0], [np.nan] * 3],
                  np.random.default_rng(0), **ARGS)
    assert np.isnan(out).all()
