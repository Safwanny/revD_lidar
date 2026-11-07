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

"""Check the URDF and meshes against the W11.25 CAD numbers."""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / 'urdf' / 'robot.urdf.xacro'

# CAD numbers, mm: build_mecanum_01_fusion.py WX, AZ, SEAT_Z + PLATE_T;
# clearance_check.txt hull, roller envelope 59.92, track 186.5; STL-19P
# datasheet 35.0 tall, window 7.8 below the top.
WHEEL_X, WHEEL_Y, WHEEL_R, WHEEL_W = 105.0, 93.25, 29.96, 31.5
HULL = (296.9, 168.1, 49.0)
HULL_BOTTOM, HULL_RIM = 11.56, 60.56
DECK_TOP = 58.06
SCAN_ABOVE_DECK = 35.0 - 7.8
WHEELS = {'front_left': (1, 1), 'front_right': (1, -1),
          'rear_left': (-1, 1), 'rear_right': (-1, -1)}
TOL = 0.3  # mm


def xacro(*args):
    return subprocess.run(['xacro', str(XACRO), *args], check=True,
                          capture_output=True, text=True).stdout


@pytest.fixture(scope='module')
def robot():
    return ET.fromstring(xacro())


def vec(text):
    return np.array([float(v) for v in text.split()]) * 1e3


def origin(robot, joint):
    return vec(robot.find(f"joint[@name='{joint}']/origin").attrib['xyz'])


def load_stl(name):
    data = (PKG / 'meshes' / name).read_bytes()
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(data[84:84 + 50 * n], rec)['v'].reshape(-1, 3) * 1e3


def test_check_urdf(tmp_path):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(xacro())
    subprocess.run(['check_urdf', str(urdf)], check=True, capture_output=True)


def test_wheels(robot):
    base_z = origin(robot, 'base_footprint_to_base_link_joint')[2]
    assert base_z == pytest.approx(WHEEL_R)
    for name, (sx, sy) in WHEELS.items():
        joint = robot.find(f"joint[@name='{name}_wheel_joint']")
        assert joint.attrib['type'] == 'continuous'
        assert origin(robot, f'{name}_wheel_joint') == pytest.approx(
            [sx * WHEEL_X, sy * WHEEL_Y, 0.0])
        cyl = robot.find(f"link[@name='{name}_wheel_link']/collision/"
                         'geometry/cylinder').attrib
        assert float(cyl['radius']) * 1e3 == pytest.approx(WHEEL_R)
        assert float(cyl['length']) * 1e3 == pytest.approx(WHEEL_W)
    assert [j.attrib['name'] for j in robot.findall('ros2_control/joint')] \
        == [f'{w}_wheel_joint' for w in
            ('front_left', 'front_right', 'rear_right', 'rear_left')]


def test_wheel_meshes_centred_on_rollers():
    for mesh in ('wheel_fl.stl', 'wheel_fr.stl', 'wheel_rl.stl',
                 'wheel_rr.stl'):
        pts = load_stl(mesh)
        r = np.hypot(pts[:, 0], pts[:, 2])
        assert abs(r.max() - WHEEL_R) < TOL
        plates = pts[r > 20.0, 1]
        assert abs(plates.max() - plates.min() - WHEEL_W) < TOL
        assert abs((plates.max() + plates.min()) / 2) < TOL


def test_body_mesh_is_the_hull():
    pts = load_stl('body.stl')
    size = pts.max(0) - pts.min(0)
    assert np.all(np.abs(size - HULL) < TOL)
    assert abs(pts[:, 2].min() - HULL_BOTTOM) < TOL
    assert abs(pts[:, 2].max() - HULL_RIM) < TOL


def test_lidar_scan_plane(robot):
    # Ground frame height of the scan plane: deck top + window height.
    z = origin(robot, 'base_footprint_to_base_link_joint')[2] + \
        origin(robot, 'base_link_to_laser_link_joint')[2]
    assert z == pytest.approx(DECK_TOP + SCAN_ABOVE_DECK)
    assert z - HULL_RIM > 20.0  # beam clears the hull rim
    joint = robot.find("joint[@name='base_link_to_laser_link_joint']")
    assert joint.find('parent').attrib['link'] == 'base_link'


def test_sensor_frames_and_no_turret(robot):
    links = {link.attrib['name'] for link in robot.findall('link')}
    for face in ('front', 'rear', 'left', 'right'):
        assert {f'tof_{face}_link', f'tof_{face}_rows_frame'} <= links
    assert 'imu_link' in links
    assert not [n for n in links if 'turret' in n or 'camera' in n]
    tof = origin(robot, 'base_link_to_tof_front_link_joint')
    assert tof[0] < HULL[0] / 2 and math.isclose(tof[1], 0.0)


def test_prefix():
    robot = ET.fromstring(xacro('prefix:=robot_'))
    assert robot.find("link[@name='robot_laser_link']") is not None
    assert robot.find("joint[@name='robot_front_left_wheel_joint']") is not None
