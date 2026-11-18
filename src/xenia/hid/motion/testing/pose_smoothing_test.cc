/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// Ported from Free Riders Recompiled (tests/pose_smoothing_test.cpp), written
// by its author YuutaTsubasa.

#include <algorithm>
#include <cmath>
#include <limits>

#include "xenia/hid/motion/pose_smoothing.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

constexpr double kInterval = 1.0 / 30.0;  // thirty pictures a second

PoseLandmarks At(float x, float y) {
  PoseLandmarks landmarks{};
  for (auto& point : landmarks) {
    point.x = x;
    point.y = y;
    point.score = 0.9f;
  }
  return landmarks;
}

PoseLandmarks WorldAt(float value) {
  auto landmarks = At(320, 240);
  for (auto& point : landmarks) {
    point.world = {value, value, value};
    point.has_world = true;
  }
  return landmarks;
}

// The wandering of a point the model reads afresh from every picture.
float Noise(int step) { return std::sin(float(step) * 2.7f) * 2.0f; }

constexpr uint32_t kWrist = pose_point::kWristRight;

}  // namespace

TEST_CASE("A still hand stops wandering", "[motion][pose_smoothing]") {
  PoseSmoothing smoothing;
  float worst_in = 0, worst_out = 0;
  for (int step = 0; step < 90; ++step) {
    auto landmarks = At(320.0f + Noise(step), 240.0f);
    float given = landmarks[kWrist].x;
    smoothing.Smooth(landmarks, kInterval);
    if (step < 30) {
      continue;  // the filter settles first
    }
    worst_in = std::max(worst_in, std::fabs(given - 320.0f));
    worst_out = std::max(worst_out, std::fabs(landmarks[kWrist].x - 320.0f));
  }
  REQUIRE(worst_in > 1.5f);  // the test's own noise is worth filtering
  REQUIRE(worst_out < worst_in * 0.35f);
}

TEST_CASE("A moving hand is not held back", "[motion][pose_smoothing]") {
  PoseSmoothing smoothing;
  float x = 320;
  // Half a second at 300 pixels a second.
  for (int step = 0; step < 30; ++step) {
    auto landmarks = At(x, 240.0f);
    smoothing.Smooth(landmarks, kInterval);
    if (step == 29) {
      REQUIRE(std::fabs(landmarks[kWrist].x - x) < 25.0f);
    }
    x += 300.0f * float(kInterval);
  }
}

TEST_CASE("After a gap the new place is taken as it is",
          "[motion][pose_smoothing]") {
  PoseSmoothing smoothing;
  for (int step = 0; step < 30; ++step) {
    auto landmarks = At(100.0f, 100.0f);
    smoothing.Smooth(landmarks, kInterval);
  }
  auto landmarks = At(500.0f, 400.0f);
  smoothing.Smooth(landmarks, 3.0);  // the camera stalled, or the player left
  REQUIRE(landmarks[kWrist].x == 500.0f);
}

TEST_CASE("A cutoff of zero passes the points through",
          "[motion][pose_smoothing]") {
  PoseSmoothing smoothing(0.0f, 0.05f);
  auto landmarks = At(1.0f, 2.0f);
  smoothing.Smooth(landmarks, kInterval);
  smoothing.Smooth(landmarks, kInterval);
  REQUIRE(landmarks[pose_point::kNose].x == 1.0f);
}

TEST_CASE("3D noise is smoothed on every axis", "[motion][pose_smoothing]") {
  PoseSmoothing smoothing;
  float worst = 0;
  for (int step = 0; step < 90; ++step) {
    auto landmarks = WorldAt(Noise(step) / 250.0f);
    smoothing.Smooth(landmarks, kInterval);
    REQUIRE(landmarks[kWrist].has_world);
    if (step >= 30) {
      for (float axis : landmarks[kWrist].world) {
        worst = std::max(worst, std::fabs(axis));
      }
    }
  }
  REQUIRE(worst < 0.0028f);  // under 35% of the 8 mm noise
}

TEST_CASE("3D movement is filtered but kept up with",
          "[motion][pose_smoothing]") {
  PoseSmoothing smoothing;
  for (int step = 0; step < 30; ++step) {
    float value = float(step) * 1.2f * float(kInterval);
    auto landmarks = WorldAt(value);
    smoothing.Smooth(landmarks, kInterval);
    if (step == 29) {
      for (float axis : landmarks[kWrist].world) {
        REQUIRE(axis < value);
        REQUIRE(value - axis < 0.1f);
      }
    }
    // 3D movement does not leak into the picture's axes.
    REQUIRE(landmarks[kWrist].x == 320);
  }
}

TEST_CASE("3D points start again after a gap or a loss",
          "[motion][pose_smoothing]") {
  for (int reset = 0; reset < 5; ++reset) {
    PoseSmoothing smoothing;
    auto landmarks = WorldAt(0);
    smoothing.Smooth(landmarks, kInterval);
    if (reset == 0) {
      smoothing.Forget();
    }
    if (reset == 1) {
      auto missing = At(320, 240);
      smoothing.Smooth(missing, kInterval);
    }
    if (reset == 2) {
      auto lost = WorldAt(0);
      for (auto& point : lost) {
        point.score = 0;
      }
      smoothing.Smooth(lost, kInterval);
    }
    landmarks = WorldAt(0.8f);
    double interval = reset == 3   ? 2.0
                      : reset == 4 ? std::numeric_limits<double>::quiet_NaN()
                                   : kInterval;
    smoothing.Smooth(landmarks, interval);
    for (float axis : landmarks[kWrist].world) {
      REQUIRE(axis == 0.8f);
    }
  }
}

TEST_CASE("A broken reading does not spoil later ones",
          "[motion][pose_smoothing]") {
  PoseSmoothing smoothing;
  auto landmarks = WorldAt(0);
  smoothing.Smooth(landmarks, kInterval);
  landmarks[kWrist].world[2] = std::numeric_limits<float>::quiet_NaN();
  landmarks[kWrist].x = std::numeric_limits<float>::infinity();
  smoothing.Smooth(landmarks, kInterval);
  REQUIRE_FALSE(landmarks[kWrist].has_world);

  landmarks = WorldAt(0.6f);
  smoothing.Smooth(landmarks, kInterval);
  REQUIRE(landmarks[kWrist].x == 320);
  for (float axis : landmarks[kWrist].world) {
    REQUIRE(axis == 0.6f);
  }
}

}  // namespace xe::hid::motion::test
