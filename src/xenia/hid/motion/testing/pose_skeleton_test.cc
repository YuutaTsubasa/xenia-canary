/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// Ported from Free Riders Recompiled (tests/pose_skeleton_test.cpp), written
// by its author YuutaTsubasa.

#include <cmath>
#include <limits>
#include <utility>

#include "xenia/hid/motion/pose_skeleton.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

using namespace pose_point;

bool Near(float a, float b, float slack = 0.02f) {
  return std::fabs(a - b) <= slack;
}

const std::array<float, 3>& At(const Body& body, Joint joint) {
  return body.joints[size_t(joint)];
}

// A body standing in the middle of a 640x480 picture, arms down. A camera
// faces the player, so the player's right is the picture's left.
PoseLandmarks Standing() {
  PoseLandmarks landmarks{};
  auto put = [&](uint32_t point, float x, float y) {
    landmarks[point].x = x;
    landmarks[point].y = y;
    landmarks[point].score = 0.9f;
  };
  put(kNose, 320, 120);
  put(kShoulderLeft, 360, 170);
  put(kShoulderRight, 280, 170);
  put(kElbowLeft, 370, 230);
  put(kElbowRight, 270, 230);
  put(kWristLeft, 375, 290);
  put(kWristRight, 265, 290);
  put(kHipLeft, 345, 300);
  put(kHipRight, 295, 300);
  put(kKneeLeft, 348, 380);
  put(kKneeRight, 292, 380);
  put(kAnkleLeft, 350, 450);
  put(kAnkleRight, 290, 450);
  return landmarks;
}

// The same body with 3D points: 5 mm per pixel, all at one depth.
PoseLandmarks StandingWorld() {
  auto landmarks = Standing();
  for (auto& point : landmarks) {
    point.world = {(point.x - 320.0f) * 0.005f, (point.y - 300.0f) * 0.005f,
                   0.0f};
    point.has_world = true;
  }
  return landmarks;
}

void Mirror(PoseLandmarks& landmarks) {
  for (auto& point : landmarks) {
    point.x = 640.0f - point.x;
    point.world[0] = -point.world[0];
  }
  for (const auto& [left, right] : {std::pair{kShoulderLeft, kShoulderRight},
                                    {kElbowLeft, kElbowRight},
                                    {kWristLeft, kWristRight},
                                    {kHipLeft, kHipRight},
                                    {kKneeLeft, kKneeRight},
                                    {kAnkleLeft, kAnkleRight},
                                    {kEyeLeft, kEyeRight},
                                    {kEarLeft, kEarRight}}) {
    std::swap(landmarks[left], landmarks[right]);
  }
}

}  // namespace

TEST_CASE("A standing body becomes a skeleton", "[motion][pose]") {
  Body body;
  REQUIRE(PoseToBody(Standing(), 640, 480, false, body));
  // The player's right shoulder at +x, the standard width apart.
  REQUIRE(Near(At(body, Joint::kShoulderRight)[0], kPoseShoulderHalfWidth));
  REQUIRE(Near(At(body, Joint::kShoulderLeft)[0], -kPoseShoulderHalfWidth));
  REQUIRE(At(body, Joint::kHandRight)[0] > 0);
  REQUIRE(At(body, Joint::kHandLeft)[0] < 0);
  REQUIRE(Near(At(body, Joint::kShoulderLeft)[2], kPoseDistance));
  // Up in the picture is up in skeleton space.
  REQUIRE(At(body, Joint::kHead)[1] > At(body, Joint::kShoulderCenter)[1]);
  REQUIRE(At(body, Joint::kShoulderCenter)[1] > At(body, Joint::kHipCenter)[1]);
  REQUIRE(At(body, Joint::kHipCenter)[1] > At(body, Joint::kKneeLeft)[1]);
  REQUIRE(At(body, Joint::kKneeLeft)[1] > At(body, Joint::kAnkleLeft)[1]);
  REQUIRE(Near(
      At(body, Joint::kSpine)[1],
      (At(body, Joint::kHipCenter)[1] + At(body, Joint::kShoulderCenter)[1]) *
          0.5f));
  REQUIRE(At(body, Joint::kFootLeft)[2] < At(body, Joint::kAnkleLeft)[2]);
  REQUIRE(std::fabs(At(body, Joint::kWristLeft)[1] -
                    At(body, Joint::kHandLeft)[1]) <
          std::fabs(At(body, Joint::kElbowLeft)[1] -
                    At(body, Joint::kHandLeft)[1]));
  for (JointState state : body.joint_states) {
    REQUIRE(state == JointState::kTracked);
  }
}

TEST_CASE("A raised arm raises the hand", "[motion][pose]") {
  PoseLandmarks raised = Standing();
  raised[kWristRight] = {260, 90, 0.9f};  // the player's right hand, up
  Body body;
  REQUIRE(PoseToBody(raised, 640, 480, false, body));
  REQUIRE(At(body, Joint::kHandRight)[1] > At(body, Joint::kShoulderRight)[1]);
  REQUIRE(At(body, Joint::kHandLeft)[1] < At(body, Joint::kShoulderLeft)[1]);
}

TEST_CASE("A mirrored picture comes out the same way round", "[motion][pose]") {
  PoseLandmarks mirrored = Standing();
  Mirror(mirrored);
  // The player's right hand, up, on the far side.
  mirrored[kWristLeft] = {640.0f - 260.0f, 90, 0.9f};

  Body body;
  REQUIRE(PoseToBody(mirrored, 640, 480, true, body));
  REQUIRE(Near(At(body, Joint::kShoulderRight)[0], kPoseShoulderHalfWidth));
  REQUIRE(At(body, Joint::kHandRight)[1] > At(body, Joint::kShoulderRight)[1]);

  // Taken as it is, it is the other hand that is up.
  Body unturned;
  REQUIRE(PoseToBody(mirrored, 640, 480, false, unturned));
  REQUIRE(At(unturned, Joint::kHandLeft)[1] >
          At(unturned, Joint::kShoulderLeft)[1]);
}

TEST_CASE("What is not a body is refused", "[motion][pose]") {
  Body body;
  REQUIRE_FALSE(PoseToBody(PoseLandmarks{}, 640, 480, false, body));
  REQUIRE_FALSE(PoseToBody(Standing(), 0, 480, false, body));
  PoseLandmarks squashed = Standing();
  squashed[kShoulderLeft].x = squashed[kShoulderRight].x;
  REQUIRE_FALSE(PoseToBody(squashed, 640, 480, false, body));
}

TEST_CASE("3D depth survives reaching and pulling back", "[motion][pose]") {
  auto points = StandingWorld();
  points[kShoulderRight].world[2] = -0.1f;
  points[kShoulderLeft].world[2] = -0.1f;
  points[kElbowRight].world[2] = -0.25f;
  points[kWristRight].world[2] = -0.5f;
  points[kAnkleRight].world[2] = 0.2f;
  Body forward;
  REQUIRE(PoseToBody(points, 640, 480, false, forward));
  REQUIRE(Near(At(forward, Joint::kHandRight)[2], 2.05f, 0.0001f));
  REQUIRE(Near(At(forward, Joint::kWristRight)[2], 2.0725f, 0.0001f));
  REQUIRE(Near(At(forward, Joint::kSpine)[2], 2.455f, 0.0001f));
  REQUIRE(Near(At(forward, Joint::kFootRight)[2], 2.6f, 0.0001f));
  // The hips anchor the body on every axis.
  REQUIRE(Near(At(forward, Joint::kHipCenter)[0], 0, 0.0001f));
  REQUIRE(Near(At(forward, Joint::kHipCenter)[1], 0, 0.0001f));
  REQUIRE(Near(At(forward, Joint::kHipCenter)[2], kPoseDistance, 0.0001f));

  points[kWristRight].world[2] = 0.2f;
  Body back;
  REQUIRE(PoseToBody(points, 640, 480, false, back));
  REQUIRE(Near(At(back, Joint::kHandRight)[2], 2.68f, 0.0001f));
}

TEST_CASE("3D scale uses the full shoulder distance", "[motion][pose]") {
  auto points = StandingWorld();
  points[kShoulderLeft].world = {0.12f, -0.65f, 0.16f};
  points[kShoulderRight].world = {-0.12f, -0.65f, -0.16f};
  // Edge on, the shoulders can be at one place in the picture.
  points[kShoulderRight].x = points[kShoulderLeft].x;
  points[kWristRight].world[2] = -0.5f;
  Body body;
  REQUIRE(PoseToBody(points, 640, 480, false, body));
  // Turning does not make the reach deeper.
  REQUIRE(Near(At(body, Joint::kHandRight)[2], 2.05f, 0.0001f));

  // Where the model puts its origin does not move the skeleton.
  for (auto& point : points) {
    point.world[0] += 0.4f;
    point.world[1] -= 0.2f;
    point.world[2] += 0.3f;
  }
  Body moved;
  REQUIRE(PoseToBody(points, 640, 480, false, moved));
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    for (size_t axis = 0; axis < 3; ++axis) {
      REQUIRE(
          Near(body.joints[joint][axis], moved.joints[joint][axis], 0.0001f));
    }
  }
}

TEST_CASE("A mirrored 3D body keeps its sides and depth", "[motion][pose]") {
  auto points = StandingWorld();
  points[kWristRight].world = {-0.35f, -0.8f, -0.5f};
  auto mirrored = points;
  Mirror(mirrored);
  Body normal, corrected;
  REQUIRE(PoseToBody(points, 640, 480, false, normal));
  REQUIRE(PoseToBody(mirrored, 640, 480, true, corrected));
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    for (size_t axis = 0; axis < 3; ++axis) {
      REQUIRE(Near(normal.joints[joint][axis], corrected.joints[joint][axis],
                   0.0001f));
    }
  }
}

TEST_CASE("A broken 3D pose leaves the last skeleton alone", "[motion][pose]") {
  Body original;
  for (auto& joint : original.joints) {
    joint = {7, 8, 9};
  }
  auto refused = [&](const PoseLandmarks& points) {
    Body body = original;
    REQUIRE_FALSE(PoseToBody(points, 640, 480, false, body));
    REQUIRE(body.joints == original.joints);
  };
  for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity(), 100.0f}) {
    auto points = StandingWorld();
    points[kWristRight].world[2] = invalid;
    refused(points);
  }
  auto points = StandingWorld();
  points[kEarLeft].has_world = false;
  refused(points);
  points = StandingWorld();
  points[kShoulderRight].world = points[kShoulderLeft].world;
  refused(points);
  points = StandingWorld();
  points[kHipLeft].score = std::numeric_limits<float>::quiet_NaN();
  refused(points);
}

}  // namespace xe::hid::motion::test
