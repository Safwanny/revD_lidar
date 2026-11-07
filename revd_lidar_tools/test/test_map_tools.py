"""world_grid renders what the scan plane cuts; map_score scores it."""

import math

import numpy as np
import pytest

from revd_lidar_tools import map_score, world_grid

WORLD = """<sdf version="1.9"><world name="w">
  <model name="ground"><link name="l"><collision name="c"><geometry>
    <plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  <model name="room_e"><pose>2 0 0.5 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>0.1 4 1</size></box></geometry>
    </collision></link></model>
  <model name="room_w"><pose>-2 0 0.5 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>0.1 4 1</size></box></geometry>
    </collision></link></model>
  <model name="room_n"><pose>0 2 0.5 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>4 0.1 1</size></box></geometry>
    </collision></link></model>
  <model name="room_s"><pose>0 -2 0.5 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>4 0.1 1</size></box></geometry>
    </collision></link></model>
  <model name="leg"><pose>1 1 0.2 0 0 0</pose><link name="l">
    <collision name="c"><geometry><cylinder><radius>0.012</radius>
    <length>0.4</length></cylinder></geometry></collision></link></model>
  <model name="table_top"><pose>1 1 0.7 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>1 1 0.03</size></box></geometry>
    </collision></link></model>
  <model name="turned"><pose>-1 -1 0.2 0 0 0.785398</pose><link name="l">
    <pose>0.5 0 0 0 0 0</pose>
    <collision name="c"><geometry><box><size>0.4 0.1 0.4</size></box>
    </geometry></collision></link></model>
</world></sdf>
"""


def occupied_at(grid, origin, x, y):
    r = world_grid.RESOLUTION
    return grid[int((y - origin[1]) / r), int((x - origin[0]) / r)]


@pytest.fixture
def reference(tmp_path):
    sdf = tmp_path / 'w.sdf'
    sdf.write_text(WORLD)
    grid, origin = world_grid.render(world_grid.shapes(sdf, 0.085))
    world_grid.save_map(grid, origin, tmp_path / 'ref')
    return grid, origin, tmp_path / 'ref.yaml'


def test_scan_plane_cut(reference):
    grid, origin, _ = reference
    assert occupied_at(grid, origin, 2.0, 0.0)          # wall
    assert occupied_at(grid, origin, 1.0, 1.0)          # thin leg kept
    assert not occupied_at(grid, origin, 1.3, 1.3)      # table top above
    assert not occupied_at(grid, origin, 0.0, 0.0)
    # Link pose composed with the model yaw: centre at (-1, -1) + 0.5 at 45 deg.
    c = 0.5 * math.cos(math.pi / 4)
    assert occupied_at(grid, origin, -1 + c, -1 + c)
    assert not occupied_at(grid, origin, -1.0, -1.0)


def test_scan_height_walks_the_chain():
    urdf = """<robot name="r">
      <joint name="a"><parent link="base_footprint"/><child link="base_link"/>
        <origin xyz="0 0 0.03" rpy="0 0 0"/></joint>
      <joint name="b"><parent link="base_link"/><child link="laser_link"/>
        <origin xyz="0.1 0 0.055" rpy="0 0 1.0"/></joint></robot>"""
    assert world_grid.scan_height(urdf) == pytest.approx(0.085)


def test_perfect_map_scores_perfectly(reference):
    _, _, ref_yaml = reference
    result, _, _ = map_score.score(ref_yaml, ref_yaml, (0.0, 0.0, 0.0))
    assert result['accuracy_mean_m'] == 0.0
    assert result['coverage_within_tol'] == 1.0


def test_map_placed_at_spawn(reference, tmp_path):
    """A map built from a robot spawned at (0.5, 0.2, 90 deg) is its world
    grid seen from that pose; placed back at the spawn it scores perfectly,
    placed at the origin it does not."""
    grid, origin, ref_yaml = reference
    spawn = (0.5, 0.2, math.pi / 2)
    meta = {'resolution': world_grid.RESOLUTION, 'origin': origin}
    world = map_score.cell_centres(grid, meta)
    c, s = math.cos(spawn[2]), math.sin(spawn[2])
    dx, dy = world[:, 0] - spawn[0], world[:, 1] - spawn[1]
    local = np.column_stack((c * dx + s * dy, -s * dx + c * dy))
    r = world_grid.RESOLUTION
    lo = local.min(axis=0) - 0.5
    shape = np.ceil((local.max(axis=0) + 0.5 - lo) / r).astype(int)
    slam = np.zeros((shape[1], shape[0]), dtype=bool)
    idx = np.floor((local - lo) / r).astype(int)
    slam[idx[:, 1], idx[:, 0]] = True
    world_grid.save_map(slam, lo, tmp_path / 'slam')

    placed, _, _ = map_score.score(tmp_path / 'slam.yaml', ref_yaml, spawn)
    assert placed['accuracy_p95_m'] <= r
    assert placed['coverage_within_tol'] == 1.0
    misplaced, _, _ = map_score.score(
        tmp_path / 'slam.yaml', ref_yaml, (0.0, 0.0, 0.0))
    assert misplaced['accuracy_mean_m'] > 0.3


# Half the hull diagonal is 0.17 m; a little more so the governor never has
# to stop the scripted driver.
ROUTE_CLEARANCE = 0.2


def test_routes_clear_every_wall():
    """Every slam_routes.yaml leg keeps ROUTE_CLEARANCE from the world."""
    from pathlib import Path

    from ament_index_python.packages import get_package_share_directory
    from scipy.spatial import cKDTree
    import yaml

    routes = yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'slam_routes.yaml').read_text())
    worlds = Path(get_package_share_directory('revd_lidar_gazebo')) / 'worlds'
    for world, route in routes.items():
        grid, origin = world_grid.render(
            world_grid.shapes(worlds / f'{world}.sdf', 0.085))
        meta = {'resolution': world_grid.RESOLUTION, 'origin': origin}
        tree = cKDTree(map_score.cell_centres(grid, meta))
        points = [route['spawn'][:2]] + route['waypoints']
        for a, b in zip(points, points[1:]):
            line = np.linspace(a, b, 50)
            # Cell centres sit up to half a cell inside the true surface.
            gap = tree.query(line)[0].min() - world_grid.RESOLUTION / 2
            assert gap >= ROUTE_CLEARANCE, f'{world}: {a} -> {b} ({gap:.2f} m)'
