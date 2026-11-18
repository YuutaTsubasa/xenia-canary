/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <cmath>

#include "xenia/hid/motion/body_placement.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

constexpr double kInterval = 1.0 / 30.0;
constexpr float kSensorHeight = 0.9f;

// A body as PoseToBody gives it: hips at (0, 0, 2.5), feet 0.95 m below.
Body Standing(float feet = -0.95f) {
  Body body;
  for (auto& joint : body.joints) {
    joint = {0.0f, 0.0f, 2.5f};
  }
  body.joints[size_t(Joint::kHead)] = {0.0f, 0.75f, 2.5f};
  body.joints[size_t(Joint::kFootLeft)] = {-0.1f, feet, 2.45f};
  body.joints[size_t(Joint::kFootRight)] = {0.1f, feet, 2.45f};
  return body;
}

float Lowest(const Body& body) {
  return std::min(body.joints[size_t(Joint::kFootLeft)][1],
                  body.joints[size_t(Joint::kFootRight)][1]);
}

bool Near(float a, float b, float slack = 0.005f) {
  return std::abs(a - b) <= slack;
}

}  // namespace

TEST_CASE("A player in the middle stands on the floor", "[motion][placement]") {
  for (float feet : {-0.95f, -0.8f}) {  // tall or short legs
    BodyPlacement placement(90.0f, kSensorHeight);
    Body body = Standing(feet);
    placement.Place(body, 320, 300, 640, 480, kInterval);
    REQUIRE(Near(Lowest(body), -kSensorHeight + BodyPlacement::kFootRadius));
    REQUIRE(Near(body.joints[size_t(Joint::kHipCenter)][0], 0.0f));
    // Depth is left alone.
    REQUIRE(body.joints[size_t(Joint::kHead)][2] == 2.5f);
  }
}

TEST_CASE("Stepping to the side moves the body", "[motion][placement]") {
  // A 90 degree lens: the picture's edge is as far out as the body is far
  // away, so a quarter of the picture from the centre is 1.25 m at 2.5 m.
  BodyPlacement placement(90.0f, kSensorHeight);
  Body body = Standing();
  placement.Place(body, 160, 300, 640, 480, kInterval);
  // The picture's left is the sensor's left, skeleton +x.
  REQUIRE(Near(body.joints[size_t(Joint::kHipCenter)][0], 1.25f));
  REQUIRE(Near(body.joints[size_t(Joint::kFootLeft)][0], 1.15f));
}

TEST_CASE("A jump lifts the body, then it settles back",
          "[motion][placement]") {
  BodyPlacement placement(90.0f, kSensorHeight);
  for (int i = 0; i < 60; ++i) {
    Body body = Standing();
    placement.Place(body, 320, 300, 640, 480, kInterval);
  }
  // Hips 64 pixels higher in the picture: 0.5 m at 2.5 m through this lens.
  Body jumping = Standing();
  placement.Place(jumping, 320, 236, 640, 480, kInterval);
  float floor = -kSensorHeight + BodyPlacement::kFootRadius;
  REQUIRE(Lowest(jumping) > floor + 0.45f);
  REQUIRE(Lowest(jumping) < floor + 0.5f);

  // Back on the ground, the feet are on the floor again (the usual height
  // barely moved during one picture).
  Body landed = Standing();
  placement.Place(landed, 320, 300, 640, 480, kInterval);
  REQUIRE(Near(Lowest(landed), floor, 0.02f));
}

TEST_CASE("A crouch keeps the feet on the floor", "[motion][placement]") {
  BodyPlacement placement(90.0f, kSensorHeight);
  for (int i = 0; i < 60; ++i) {
    Body body = Standing();
    placement.Place(body, 320, 300, 640, 480, kInterval);
  }
  // Hips 0.3 m lower (38.4 pixels), feet 0.3 m nearer the hips.
  Body crouching = Standing(-0.65f);
  placement.Place(crouching, 320, 338.4f, 640, 480, kInterval);
  float floor = -kSensorHeight + BodyPlacement::kFootRadius;
  REQUIRE(Near(Lowest(crouching), floor, 0.02f));
  REQUIRE(crouching.joints[size_t(Joint::kHipCenter)][1] <
          floor + 0.95f - 0.25f);
}

}  // namespace xe::hid::motion::test
