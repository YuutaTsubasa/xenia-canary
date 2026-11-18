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

#include "xenia/hid/motion/synthetic_depth.h"
#include "xenia/hid/motion/virtual_kinect_v1.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

uint16_t At(const DepthImage& depth, uint32_t u, uint32_t v) {
  return depth.millimetres[size_t(v) * depth.width + u];
}

// The pixel a room point falls on through a level v1 lens. Room +x is the
// sensor's left, so it lands left of centre.
std::array<int, 2> PixelOf(float x, float y, float z) {
  return {int(std::floor(320.0f - kV1DepthFocalLength * x / z)),
          int(std::floor(240.0f - kV1DepthFocalLength * y / z))};
}

// A player standing 2.5 m away, hips at the sensor's height, the right arm
// held straight out to the side.
Body Standing() {
  Body body;
  auto put = [&](Joint joint, float x, float y, float z = 2.5f) {
    body.joints[size_t(joint)] = {x, y, z};
    body.joint_states[size_t(joint)] = JointState::kTracked;
  };
  put(Joint::kHipCenter, 0.0f, 0.0f);
  put(Joint::kSpine, 0.0f, 0.25f);
  put(Joint::kShoulderCenter, 0.0f, 0.5f);
  put(Joint::kHead, 0.0f, 0.75f);
  put(Joint::kShoulderLeft, -0.18f, 0.5f);
  put(Joint::kElbowLeft, -0.2f, 0.22f);
  put(Joint::kWristLeft, -0.21f, -0.03f);
  put(Joint::kHandLeft, -0.21f, -0.06f);
  put(Joint::kShoulderRight, 0.18f, 0.5f);
  put(Joint::kElbowRight, 0.46f, 0.5f);
  put(Joint::kWristRight, 0.7f, 0.5f);
  put(Joint::kHandRight, 0.73f, 0.5f);
  put(Joint::kHipLeft, -0.1f, 0.0f);
  put(Joint::kKneeLeft, -0.1f, -0.45f);
  put(Joint::kAnkleLeft, -0.1f, -0.85f);
  put(Joint::kFootLeft, -0.1f, -0.88f, 2.42f);
  put(Joint::kHipRight, 0.1f, 0.0f);
  put(Joint::kKneeRight, 0.1f, -0.45f);
  put(Joint::kAnkleRight, 0.1f, -0.85f);
  put(Joint::kFootRight, 0.1f, -0.88f, 2.42f);
  return body;
}

}  // namespace

TEST_CASE("An empty room: the wall ahead and the floor below",
          "[motion][synthetic_depth]") {
  SyntheticRoom room;
  DepthImage depth = DrawSyntheticDepth(nullptr, room);
  REQUIRE(depth.width == kV1DepthWidth);
  REQUIRE(depth.height == kV1DepthHeight);
  REQUIRE(depth.lens.focal_x == kV1DepthFocalLength);
  // Straight ahead (just above the centre, so above the floor's horizon).
  REQUIRE(At(depth, 320, 200) == 3800);
  // The wall is flat, so its depth is the same off to the side.
  REQUIRE(At(depth, 20, 200) == 3800);
  // The bottom row sees the floor, closer than the wall: a ray dropping
  // 0.9 m by z reaches the floor at z = 0.9 / (y/z).
  float drop = (479.5f - 240.0f) / kV1DepthFocalLength;
  REQUIRE(std::abs(At(depth, 320, 479) - 900.0f / drop) <= 1.0f);
  // Every pixel reads something.
  for (uint16_t value : depth.millimetres) {
    REQUIRE(value != 0);
  }
}

TEST_CASE("The body stands in front of the wall", "[motion][synthetic_depth]") {
  Body body = Standing();
  DepthImage depth = DrawSyntheticDepth(&body, SyntheticRoom());

  // The chest: the torso's front, a little nearer than its centre line.
  auto chest = PixelOf(0.0f, 0.3f, 2.5f);
  uint16_t chest_mm = At(depth, chest[0], chest[1]);
  REQUIRE(chest_mm > 2300);
  REQUIRE(chest_mm < 2450);

  // The player's right hand is out to the sensor's left, so on the left of
  // the picture; the same place on the right is the bare wall.
  auto hand = PixelOf(0.6f, 0.5f, 2.5f);
  REQUIRE(hand[0] < 320);
  REQUIRE(At(depth, hand[0], hand[1]) < 2500);
  REQUIRE(At(depth, 640 - hand[0], hand[1]) == 3800);

  // Between the legs, below the hips, the wall shows through.
  auto gap = PixelOf(0.0f, -0.6f, 2.5f);
  REQUIRE(At(depth, gap[0], gap[1]) > 3000);
}

TEST_CASE("Joints that are not tracked are not drawn",
          "[motion][synthetic_depth]") {
  Body body = Standing();
  body.joint_states[size_t(Joint::kHandRight)] = JointState::kNotTracked;
  body.joint_states[size_t(Joint::kElbowRight)] = JointState::kNotTracked;
  DepthImage depth = DrawSyntheticDepth(&body, SyntheticRoom());
  // The forearm and hand are gone (the upper arm went with the elbow).
  auto hand = PixelOf(0.65f, 0.5f, 2.5f);
  REQUIRE(At(depth, hand[0], hand[1]) == 3800);
}

TEST_CASE("A tilted sensor sees the same room from another angle",
          "[motion][synthetic_depth]") {
  SyntheticRoom room;
  room.tilt_degrees = 20.0f;
  DepthImage depth = DrawSyntheticDepth(nullptr, room);
  // The centre ray now rises 20 degrees and meets the wall farther along
  // the lens axis: z = 3.8 / cos(20).
  float expected = 3800.0f / std::cos(20.0f * std::numbers::pi_v<float> / 180);
  float centre = 0.5f * (At(depth, 319, 239) + At(depth, 320, 240));
  REQUIRE(std::abs(centre - expected) < 15.0f);

  // Tilted down, more of the picture is floor: the centre sees it.
  room.tilt_degrees = -27.0f;
  depth = DrawSyntheticDepth(nullptr, room);
  // The floor 0.9 m down along a ray 27 degrees down: 0.9 / sin(27) metres
  // along it, all of it along the lens axis.
  float floor = 900.0f / std::sin(27.0f * std::numbers::pi_v<float> / 180.0f);
  REQUIRE(std::abs(At(depth, 320, 240) - floor) < 15.0f);
}

}  // namespace xe::hid::motion::test
