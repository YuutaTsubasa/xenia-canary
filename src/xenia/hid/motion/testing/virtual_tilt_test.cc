/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <cmath>
#include <numbers>

#include "xenia/hid/motion/virtual_tilt.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

// How the title's NUI library turns the accelerometer into a whole-degree
// angle (docs/kinect-pscam.md section 13): atan2(z, y), rounded.
int32_t AngleTheTitleReads(const std::array<int32_t, 3>& gravity) {
  double radians = std::atan2(double(gravity[2]), double(gravity[1]));
  return int32_t(std::floor(radians * 180.0 / std::numbers::pi + 0.5));
}

}  // namespace

TEST_CASE("Virtual tilt starts level", "[motion][virtual_tilt]") {
  VirtualTilt tilt;
  REQUIRE(tilt.angle() == 0);
  REQUIRE(AngleTheTitleReads(tilt.gravity()) == 0);
  REQUIRE(tilt.floor_plane()[3] == 0.0f);  // nothing remembered
}

TEST_CASE("The accelerometer agrees with every angle the motor takes",
          "[motion][virtual_tilt]") {
  VirtualTilt tilt;
  for (int32_t degrees = kV1TiltMinDegrees; degrees <= kV1TiltMaxDegrees;
       ++degrees) {
    REQUIRE(tilt.SetAngle(degrees) == degrees);
    REQUIRE(tilt.angle() == degrees);
    REQUIRE(AngleTheTitleReads(tilt.gravity()) == degrees);
    REQUIRE(tilt.gravity()[0] == 0);
  }
}

TEST_CASE("The motor stops at the end of its range", "[motion][virtual_tilt]") {
  VirtualTilt tilt;
  REQUIRE(tilt.SetAngle(40) == kV1TiltMaxDegrees);
  REQUIRE(tilt.SetAngle(-90) == kV1TiltMinDegrees);
  REQUIRE(AngleTheTitleReads(tilt.gravity()) == kV1TiltMinDegrees);
}

TEST_CASE("A remembered floor keeps the gravity of its moment",
          "[motion][virtual_tilt]") {
  VirtualTilt tilt;
  tilt.SetAngle(-25);
  tilt.RememberFloor({0.0f, 0.9f, 0.4f, 1.1f});
  tilt.SetAngle(3);
  REQUIRE(tilt.floor_plane()[1] == 0.9f);
  REQUIRE(tilt.floor_plane()[3] == 1.1f);
  REQUIRE(AngleTheTitleReads(tilt.floor_gravity()) == -25);
  REQUIRE(AngleTheTitleReads(tilt.gravity()) == 3);

  tilt.Reset();
  REQUIRE(tilt.angle() == 0);
  REQUIRE(tilt.floor_plane()[3] == 0.0f);
}

}  // namespace xe::hid::motion::test
