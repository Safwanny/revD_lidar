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

#include "revd_lidar_safety/governor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace revd_lidar_safety
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

bool inside(const Point & q, const Point & half)
{
  return std::abs(q.x) <= half.x && std::abs(q.y) <= half.y;
}

// Along one axis, the range of s for which |q - s * u| <= h.
void axis_range(double q, double u, double h, double & lo, double & hi)
{
  if (u == 0.0) {
    // Along an axis it does not move on, the hull covers a point always or
    // never.
    const bool covered = std::abs(q) <= h;
    lo = covered ? -kInf : kInf;
    hi = covered ? kInf : -kInf;
    return;
  }
  const double first = (q - h) / u;
  const double second = (q + h) / u;
  lo = std::min(first, second);
  hi = std::max(first, second);
}

}  // namespace

double stopping_speed(double distance, const Params & p)
{
  if (distance <= 0.0) {
    return 0.0;
  }
  const double a = p.brake_decel;
  const double t = p.latency;
  return std::max(p.creep_speed, a * (std::sqrt(t * t + 2.0 * distance / a) - t));
}

double entry_distance(const Point & q, const Point & u, const Point & half)
{
  double lo_x, hi_x, lo_y, hi_y;
  axis_range(q.x, u.x, half.x, lo_x, hi_x);
  axis_range(q.y, u.y, half.y, lo_y, hi_y);
  const double enter = std::max(lo_x, lo_y);
  const double leave = std::min(hi_x, hi_y);
  return enter > 0.0 && enter <= leave ? enter : kInf;
}

double path_distance(
  const std::vector<Point> & points, const Point & u,
  const Point & half)
{
  double nearest = kInf;
  for (const auto & q : points) {
    nearest = std::min(nearest, entry_distance(q, u, half));
  }
  return nearest;
}

bool beside(
  const std::vector<Point> & points, const Point & u, const Point & half,
  double gap, double ahead)
{
  const Point n{-u.y, u.x};
  const double reach = std::abs(u.x) * half.x + std::abs(u.y) * half.y;
  const double width = std::abs(n.x) * half.x + std::abs(n.y) * half.y;
  for (const auto & q : points) {
    const double along = q.x * u.x + q.y * u.y;
    const double across = std::abs(q.x * n.x + q.y * n.y);
    if (across > width && across <= width + gap && along >= -reach &&
      along <= reach + ahead)
    {
      return true;
    }
  }
  return false;
}

double turn_angle(
  const std::vector<Point> & points, const Point & half,
  double sign)
{
  // Only points between the hull and its corner circle can be hit. Turning
  // the hull by +angle is turning the points by -angle.
  const double step = M_PI / 180.0;
  const double corner = std::hypot(half.x, half.y);
  // As many steps as numpy.arange(step, pi + step, step) holds.
  const int steps = static_cast<int>(std::ceil((M_PI + step - step) / step));
  for (int i = 0; i < steps; ++i) {
    const double angle = step + i * step;
    const double c = std::cos(sign * angle);
    const double s = std::sin(sign * angle);
    for (const auto & q : points) {
      if (std::hypot(q.x, q.y) > corner || inside(q, half)) {
        continue;
      }
      if (inside({c * q.x + s * q.y, -s * q.x + c * q.y}, half)) {
        return angle - step;
      }
    }
  }
  return kInf;
}

Command govern(
  const Command & command, const std::vector<Point> & points,
  const Ages & ages, const Params & p)
{
  const Point half{p.half_length, p.half_width};
  const double speed = std::hypot(command.vx, command.vy);
  double v_cap = p.max_speed;
  double w_cap = p.max_angular;
  auto limit = [&](double age, bool toward) {
      if (age > p.stale_after) {
        v_cap = std::min(v_cap, toward ? 0.0 : p.creep_speed);
        w_cap = 0.0;
      } else if (age > p.slow_after && toward) {
        v_cap = std::min(v_cap, p.creep_speed);
      }
    };
  for (std::size_t face = 0; face < kNormals.size(); ++face) {
    limit(
      ages.faces[face],
      command.vx * kNormals[face].x + command.vy * kNormals[face].y > 0.0);
  }
  if (ages.lidar >= 0.0) {
    limit(ages.lidar, speed > 0.0);
  }
  if (speed > 0.0) {
    const Point u{command.vx / speed, command.vy / speed};
    const Point grown{
      half.x + p.side_min_gap + (p.min_gap - p.side_min_gap) * std::abs(u.x),
      half.y + p.side_min_gap + (p.min_gap - p.side_min_gap) * std::abs(u.y)};
    double distance = path_distance(points, u, grown);
    // A point already within min_gap stops only motion that closes on it.
    for (const auto & q : points) {
      if (inside(q, grown) && entry_distance(q, u, half) < kInf) {
        distance = 0.0;
        break;
      }
    }
    v_cap = std::min(v_cap, stopping_speed(distance, p));
    // The side zone runs as far ahead as the stop at full speed.
    const double ahead = p.max_speed * p.latency +
      p.max_speed * p.max_speed / (2.0 * p.brake_decel);
    if (beside(points, u, half, p.side_gap, ahead)) {
      v_cap = std::min(v_cap, p.side_factor * p.max_speed);
    }
  }
  if (command.wz != 0.0) {
    // The corners move fastest; limit their arc like a straight path.
    const double corner = std::hypot(half.x, half.y);
    const double angle = turn_angle(points, half, std::copysign(1.0, command.wz));
    w_cap = std::min(w_cap, stopping_speed(angle * corner - p.min_gap, p) / corner);
  }
  double scale = 1.0;
  if (speed > 0.0) {
    scale = std::min(scale, v_cap / speed);
  }
  if (command.wz != 0.0) {
    scale = std::min(scale, w_cap / std::abs(command.wz));
  }
  return {command.vx * scale, command.vy * scale, command.wz * scale};
}

bool spin_clear(
  const std::vector<Point> & points, const Point & half,
  double margin)
{
  const double reach = std::hypot(half.x, half.y) + margin;
  for (const auto & q : points) {
    if (!inside(q, half) && std::hypot(q.x, q.y) <= reach) {
      return false;
    }
  }
  return true;
}

Command axis_only(
  const Command & command, const std::vector<Point> & points,
  const Params & p)
{
  Command out = command;
  if (out.vx != 0.0 || out.vy != 0.0) {
    if (std::abs(out.vx) >= std::abs(out.vy)) {
      out.vy = 0.0;
    } else {
      out.vx = 0.0;
    }
    out.wz = std::clamp(out.wz, -p.trim_angular, p.trim_angular);
    return out;
  }
  if (out.wz != 0.0 && !spin_clear(points, {p.half_length, p.half_width}, p.spin_margin)) {
    out.wz = 0.0;
  }
  return out;
}

}  // namespace revd_lidar_safety
