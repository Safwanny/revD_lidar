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
from revd_lidar_tools.nav_eval import CLEARANCE, hull_gap, MIN_HOP, sample_goals
from scipy import ndimage

# A 6 x 6 m walled room at 0.05 m, origin (-3, -3), plus a closed-off 1 m
# cell outside it that goals must never land in.
META = {'resolution': 0.05, 'origin': [-3.0, -3.0, 0.0]}
GRID = np.zeros((140, 140), bool)
GRID[0, :120] = GRID[119, :120] = GRID[:120, 0] = GRID[:120, 119] = True


def test_goals_are_clear_reachable_spaced_and_seeded():
    goals = sample_goals(GRID, META, (0.0, 0.0), 20, seed=3)
    assert goals == sample_goals(GRID, META, (0.0, 0.0), 20, seed=3)
    assert goals != sample_goals(GRID, META, (0.0, 0.0), 20, seed=4)
    last = (0.0, 0.0)
    for x, y, yaw in goals:
        assert -3 + CLEARANCE <= x <= 3 - CLEARANCE
        assert -3 + CLEARANCE <= y <= 3 - CLEARANCE
        assert math.dist((x, y), last) >= MIN_HOP
        assert -math.pi <= yaw <= math.pi
        last = (x, y)


def test_hull_gap_goes_negative_inside_a_wall():
    distance = ndimage.distance_transform_edt(~GRID) * 0.05
    assert math.isclose(hull_gap((0.0, 0.0, 0.0), distance, META),
                        2.975 - 0.15 - 0.025, abs_tol=0.051)
    # Nose 0.15 m into the east wall's cells.
    assert hull_gap((2.9, 0.0, 0.0), distance, META) <= 0.0
