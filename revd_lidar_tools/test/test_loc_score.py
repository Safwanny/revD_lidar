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

import math

import numpy as np
from revd_lidar_tools.loc_score import score


def test_converges_after_the_offset_and_measures_from_there():
    t = np.arange(0.0, 10.0, 0.1)
    truth = np.column_stack((t, 0.1 * t, np.zeros_like(t), np.zeros_like(t)))
    # Spawned at (1, 2, 90 deg): truth in the world, estimate in the map.
    spawn = (1.0, 2.0, math.pi / 2)
    world = truth.copy()
    world[:, 1], world[:, 2] = 1.0, 2.0 + 0.1 * t
    world[:, 3] = math.pi / 2
    estimate = truth.copy()
    estimate[:, 2] = np.where(t < 3.0, 0.5, 0.02)
    result = score(world, estimate, spawn)
    assert result['converged']
    assert math.isclose(result['converge_s'], 3.0, abs_tol=1e-6)
    assert math.isclose(result['rmse_m'], 0.02, abs_tol=1e-6)
    assert result['lost_share'] == 0.0


def test_never_converged():
    t = np.arange(0.0, 2.0, 0.1)
    truth = np.column_stack((t, t, t, t))
    estimate = truth.copy()
    estimate[:, 1] += 1.0
    assert score(truth, estimate) == {'samples': 20, 'converged': False}
