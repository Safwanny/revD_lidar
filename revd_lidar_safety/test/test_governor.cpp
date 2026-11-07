// Copyright 2026 Safwan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Every case of the Python governor's tests, plus the LiDAR source.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "revd_lidar_safety/governor.hpp"

using revd_lidar_safety::Ages;
using revd_lidar_safety::Command;
using revd_lidar_safety::Params;
using revd_lidar_safety::Point;

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
const Params P;
const Point HALF{0.15, 0.11};
const Ages FRESH;  // every face 0 s old, LiDAR not a source
const std::vector<Point> EMPTY;

std::vector<Point> line(double x0, double y0, double x1, double y1, int n)
{
  std::vector<Point> points;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / (n - 1);
    points.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
  }
  return points;
}

const std::vector<Point> WALL_AHEAD = line(0.40, -0.3, 0.40, 0.3, 13);

Ages with_face(std::size_t face, double age)
{
  Ages ages;
  ages.faces[face] = age;
  return ages;
}

Command govern(
  double vx, double vy, double wz, const std::vector<Point> & points,
  const Ages & ages = FRESH)
{
  return revd_lidar_safety::govern({vx, vy, wz}, points, ages, P);
}

}  // namespace

TEST(Governor, StoppingSpeedStopsInsideTheDistance)
{
  for (double distance : {0.05, 0.1, 0.2, 0.5}) {
    const double v = revd_lidar_safety::stopping_speed(distance, P);
    const double travel = v * P.latency + v * v / (2 * P.brake_decel);
    EXPECT_TRUE(travel <= distance + 1e-9 || v == P.creep_speed);
  }
  EXPECT_EQ(revd_lidar_safety::stopping_speed(0.0, P), 0.0);
  EXPECT_EQ(revd_lidar_safety::stopping_speed(1e-4, P), P.creep_speed);
  EXPECT_EQ(revd_lidar_safety::stopping_speed(kInf, P), kInf);
}

TEST(Governor, PathDistanceByDirection)
{
  EXPECT_NEAR(revd_lidar_safety::path_distance(WALL_AHEAD, {1, 0}, HALF), 0.25, 1e-9);
  // Diagonal: the corner meets the wall after 0.25 / cos 45.
  const double d = std::sqrt(0.5);
  EXPECT_NEAR(
    revd_lidar_safety::path_distance(WALL_AHEAD, {d, d}, HALF), 0.25 * std::sqrt(2.0), 1e-9);
  // Moving away or along it never meets the wall.
  EXPECT_EQ(revd_lidar_safety::path_distance(WALL_AHEAD, {-1, 0}, HALF), kInf);
  EXPECT_EQ(revd_lidar_safety::path_distance(WALL_AHEAD, {0, 1}, HALF), kInf);
}

TEST(Governor, PointsInsideTheHullAreIgnored)
{
  EXPECT_EQ(revd_lidar_safety::path_distance({{0.1, 0.0}}, {1, 0}, HALF), kInf);
}

TEST(Governor, BesideOnlyCountsTheSides)
{
  const auto side_wall = line(-0.3, 0.14, 0.3, 0.14, 13);
  EXPECT_TRUE(revd_lidar_safety::beside(side_wall, {1, 0}, HALF, 0.05, 0.15));
  EXPECT_FALSE(
    revd_lidar_safety::beside(line(-0.3, 0.24, 0.3, 0.24, 13), {1, 0}, HALF, 0.05, 0.15));
  EXPECT_FALSE(revd_lidar_safety::beside(WALL_AHEAD, {1, 0}, HALF, 0.05, 0.15));
}

TEST(Governor, TurnAngle)
{
  EXPECT_EQ(revd_lidar_safety::turn_angle({{1.0, 0.0}}, HALF, 1.0), kInf);
  // Just ahead of the front face's left end: turning right swings the face
  // into it at once, turning left swings it away.
  const std::vector<Point> point{{0.16, 0.08}};
  const double left = revd_lidar_safety::turn_angle(point, HALF, 1.0);
  const double right = revd_lidar_safety::turn_angle(point, HALF, -1.0);
  const double ten = 10.0 * M_PI / 180.0;
  EXPECT_LT(right, ten);
  EXPECT_LT(ten, left);
}

TEST(Governor, MovingAwayIsNeverLimited)
{
  const auto pressed = line(0.16, -0.1, 0.16, 0.1, 9);
  EXPECT_EQ(govern(0.3, 0.0, 0.0, pressed).vx, 0.0);
  const Command away = govern(-0.3, 0.0, 0.0, pressed);
  EXPECT_EQ(away.vx, -0.3);
  EXPECT_EQ(away.vy, 0.0);
  EXPECT_EQ(away.wz, 0.0);
}

TEST(Governor, ClearanceAtAnyAngle)
{
  // Sliding +y past a wall 2 cm off the side that closes in at 3 deg:
  // stopped where the side clearance would fall under side_min_gap.
  const double t = std::tan(3.0 * M_PI / 180.0);
  std::vector<Point> slanted, parallel, facing;
  for (int i = 0; i <= 4000; ++i) {
    const double y = 2.0 * i / 4000;
    slanted.push_back({0.17 - y * t, y});
    parallel.push_back({0.17, y - 1.0});
    facing.push_back({0.175, y - 1.0});
  }
  const double expected = revd_lidar_safety::stopping_speed(0.01 / t - 0.14, P);
  EXPECT_NEAR(govern(0.0, 0.6, 0.0, slanted).vy, expected, 0.05 * expected);
  // A parallel wall 2 cm off the side: it slides on at the side cap.
  EXPECT_NEAR(govern(0.0, 0.6, 0.0, parallel).vy, P.side_factor * P.max_speed, 1e-9);
  // Facing a wall it stopped min_gap short of: no further.
  EXPECT_EQ(govern(0.3, 0.0, 0.0, facing).vx, 0.0);
}

TEST(Governor, GovernedSpeeds)
{
  EXPECT_NEAR(govern(1.0, 0.0, 0.0, EMPTY).vx, P.max_speed, 1e-9);
  auto nearer = WALL_AHEAD;
  for (auto & q : nearer) {
    q.x -= 0.1;
  }
  const double vx = govern(1.0, 0.0, 0.0, nearer).vx;
  EXPECT_LT(P.creep_speed, vx);
  EXPECT_LT(vx, P.max_speed);
  EXPECT_NEAR(
    govern(1.0, 0.0, 0.0, line(-0.3, 0.14, 0.3, 0.14, 13)).vx,
    P.side_factor * P.max_speed, 1e-9);
}

TEST(Governor, StaleFaceBlocksOnlyItsSide)
{
  const Ages stale = with_face(0, 1.0);  // front
  const Command toward = govern(0.3, 0.0, 0.0, EMPTY, stale);
  EXPECT_EQ(toward.vx, 0.0);
  EXPECT_EQ(toward.vy, 0.0);
  EXPECT_EQ(govern(-0.3, 0.0, 0.0, EMPTY, stale).vx, -P.creep_speed);
  EXPECT_EQ(govern(0.0, 0.0, 1.0, EMPTY, stale).wz, 0.0);
  EXPECT_NEAR(govern(0.3, 0.0, 0.0, EMPTY, with_face(0, 0.3)).vx, P.creep_speed, 1e-12);
}

TEST(Governor, AxisOnly)
{
  auto check = [](Command in, const std::vector<Point> & points, Command expected) {
      const Command out = revd_lidar_safety::axis_only(in, points, P);
      EXPECT_EQ(out.vx, expected.vx);
      EXPECT_EQ(out.vy, expected.vy);
      EXPECT_EQ(out.wz, expected.wz);
    };
  check({0.3, 0.1, 1.0}, EMPTY, {0.3, 0.0, 0.2});
  check({0.1, -0.3, 0.0}, EMPTY, {0.0, -0.3, 0.0});
  check({0.0, 0.0, 1.0}, EMPTY, {0.0, 0.0, 1.0});
  // Something inside the corner circle: no standing spin.
  check({0.0, 0.0, 1.0}, {{0.19, 0.0}}, {0.0, 0.0, 0.0});
  EXPECT_TRUE(revd_lidar_safety::spin_clear({{0.22, 0.0}}, HALF, 0.02));
}

TEST(Governor, SilentLidarBlocksEveryDirection)
{
  Ages ages;
  ages.lidar = 0.0;
  EXPECT_NEAR(govern(0.3, 0.0, 0.0, EMPTY, ages).vx, 0.3, 1e-12);
  ages.lidar = 0.3;
  EXPECT_NEAR(govern(0.0, -0.3, 0.0, EMPTY, ages).vy, -P.creep_speed, 1e-12);
  EXPECT_EQ(govern(0.0, 0.0, 1.0, EMPTY, ages).wz, 1.0);
  ages.lidar = 1.0;
  for (const Command c : {Command{0.3, 0.0, 0.0}, Command{-0.2, 0.2, 0.0},
      Command{0.0, 0.0, 1.0}})
  {
    const Command out = govern(c.vx, c.vy, c.wz, EMPTY, ages);
    EXPECT_EQ(out.vx, 0.0);
    EXPECT_EQ(out.vy, 0.0);
    EXPECT_EQ(out.wz, 0.0);
  }
}

TEST(Governor, LidarPointsLimitLikeAnyOther)
{
  // A far, high wall only the scan sees: the same stop as a ToF wall.
  Ages ages;
  ages.lidar = 0.0;
  EXPECT_EQ(govern(1.0, 0.0, 0.0, WALL_AHEAD, ages).vx, govern(1.0, 0.0, 0.0, WALL_AHEAD).vx);
}
