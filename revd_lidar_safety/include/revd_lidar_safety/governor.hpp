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

// Geometry of the safety governor, free of ROS so it tests on its own.
//
// Every velocity command is slowed so the robot can always stop before a
// hit. Only motion that closes on a point is limited, and only by what lies
// along that motion: the hull, grown by min_gap along the motion and by
// side_min_gap across it, is swept along the commanded direction and the
// speed is the highest that still stops before the first point it meets.
// Moving away from anything is never limited, so the robot can always leave
// a place it got into. Points already inside the hull are taken as noise.
// Close to stopping it still creeps, and zero speed only ever blocks the one
// direction that would touch. Numbers and sources:
// revd_lidar_bringup/config/safety.yaml.

#ifndef REVD_LIDAR_SAFETY__GOVERNOR_HPP_
#define REVD_LIDAR_SAFETY__GOVERNOR_HPP_

#include <array>
#include <vector>

namespace revd_lidar_safety
{

struct Point
{
  double x;
  double y;
};

struct Command
{
  double vx;
  double vy;
  double wz;
};

struct Params
{
  double half_length = 0.15;
  double half_width = 0.11;
  double max_speed = 0.45;
  double max_angular = 2.5;
  double brake_decel = 1.5;
  double latency = 0.18;
  double min_gap = 0.03;
  double side_min_gap = 0.01;
  double creep_speed = 0.03;
  double side_gap = 0.05;
  double side_factor = 0.6;
  double slow_after = 0.2;
  double stale_after = 0.75;
  double trim_angular = 0.2;
  double spin_margin = 0.02;
};

// Front, rear, left, right: outward normal of each ToF face in the base
// frame.
constexpr std::array<Point, 4> kNormals{{{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0},
  {0.0, -1.0}}};

// Seconds since each source last reported. The LiDAR sees every direction;
// lidar < 0 means it is not a source (tof_only).
struct Ages
{
  std::array<double, 4> faces{};
  double lidar = -1.0;
};

// Highest speed that still stops within distance: travel during latency at
// full speed, then braking at brake_decel. Never below creep_speed until the
// distance is used up; zero from there.
double stopping_speed(double distance, const Params & p);

// How far the hull (half extents) moves along unit direction u before it
// meets point q; infinity if it never does or q is inside it now or only
// behind it.
double entry_distance(const Point & q, const Point & u, const Point & half);

// Smallest entry_distance over points.
double path_distance(
  const std::vector<Point> & points, const Point & u,
  const Point & half);

// True if a point is within gap of the hull's swept sides, from the back of
// the hull to ahead past its front. Points in the swept path are left to
// path_distance.
bool beside(
  const std::vector<Point> & points, const Point & u, const Point & half,
  double gap, double ahead);

// How far the hull turns (direction sign) before it meets a point, in
// one-degree steps rounded down; infinity when nothing can be hit.
double turn_angle(
  const std::vector<Point> & points, const Point & half,
  double sign);

// The command scaled down to what is safe; its direction is kept. A face
// silent past slow_after lets motion toward it only creep; past stale_after
// that motion and turning are blocked and everything else creeps. A silent
// LiDAR counts as a face toward every direction.
Command govern(
  const Command & command, const std::vector<Point> & points,
  const Ages & ages, const Params & p);

// True if no point outside the hull is within its corner circle plus
// margin: what a standing spin sweeps.
bool spin_clear(
  const std::vector<Point> & points, const Point & half,
  double margin);

// Snap a command to what tof_only mode allows: translation only along the
// sensing axes (the larger of vx, vy), with at most trim_angular of heading
// trim; a standing spin only when spin_clear.
Command axis_only(
  const Command & command, const std::vector<Point> & points,
  const Params & p);

}  // namespace revd_lidar_safety

#endif  // REVD_LIDAR_SAFETY__GOVERNOR_HPP_
