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

// Parity with the Python governor this node replaced. parity_cases.txt holds
// governor inputs recorded in sim (test_safety_chain, navigation_basic:
// every tenth command, the 50 points nearest the hull plus 10 others) and
// the Python governor's output for each, in normal and tof_only mode, as
// recorded, with one face made stale, and with a random turn added.

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "revd_lidar_safety/governor.hpp"

using revd_lidar_safety::Ages;
using revd_lidar_safety::Command;
using revd_lidar_safety::Params;
using revd_lidar_safety::Point;

TEST(Parity, MatchesThePythonGovernor)
{
  std::ifstream file(PARITY_CASES);
  ASSERT_TRUE(file.good()) << PARITY_CASES;
  const Params p;
  std::string line;
  int compared = 0;
  int limited = 0;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream fields(line);
    std::size_t n;
    fields >> n;
    std::vector<Point> points(n);
    for (auto & q : points) {
      fields >> q.x >> q.y;
    }
    int tof_only;
    while (fields >> tof_only) {
      Command command, expected;
      Ages ages;
      fields >> command.vx >> command.vy >> command.wz;
      for (auto & age : ages.faces) {
        fields >> age;
      }
      fields >> expected.vx >> expected.vy >> expected.wz;
      if (tof_only) {
        command = revd_lidar_safety::axis_only(command, points, p);
      }
      const Command out = revd_lidar_safety::govern(command, points, ages, p);
      EXPECT_NEAR(out.vx, expected.vx, 1e-6) << line.substr(0, 80);
      EXPECT_NEAR(out.vy, expected.vy, 1e-6) << line.substr(0, 80);
      EXPECT_NEAR(out.wz, expected.wz, 1e-6) << line.substr(0, 80);
      ++compared;
      limited += out.vx != command.vx || out.vy != command.vy || out.wz != command.wz;
    }
  }
  EXPECT_GT(compared, 500);
  // The recording must exercise the limits, not only free driving.
  EXPECT_GT(limited, compared / 2);
}
