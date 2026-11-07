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

"""Row selection keeps whole grid rows, in the order asked for."""

from revd_lidar_tools.tof_rows import keep_rows
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

ZONES = 8


def grid_cloud():
    """An 8 x 8 cloud whose point (row, column) sits at x=row, y=column."""
    points = [(float(row), float(column), 0.0)
              for row in range(ZONES) for column in range(ZONES)]
    fields = [PointField(name=name, offset=4 * index,
                         datatype=PointField.FLOAT32, count=1)
              for index, name in enumerate('xyz')]
    cloud = point_cloud2.create_cloud(Header(frame_id='tof_front_link'),
                                      fields, points)
    cloud.height, cloud.width = ZONES, ZONES
    cloud.row_step = cloud.point_step * ZONES
    return cloud


def test_keeps_only_the_listed_rows_whole():
    out = keep_rows(grid_cloud(), ZONES, [4, 5, 6])
    assert (out.height, out.width) == (3, ZONES)
    assert out.header.frame_id == 'tof_front_link'
    points = list(point_cloud2.read_points(out, field_names=('x', 'y')))
    assert [(p[0], p[1]) for p in points] == [
        (float(row), float(column))
        for row in (4, 5, 6) for column in range(ZONES)]


def test_rejects_a_cloud_that_is_not_the_full_grid():
    cloud = grid_cloud()
    cloud.data = bytes(cloud.data)[:cloud.point_step * 10]
    assert keep_rows(cloud, ZONES, [0]) is None
