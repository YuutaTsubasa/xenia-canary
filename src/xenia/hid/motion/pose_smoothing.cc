/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/pose_smoothing.h"

#include <cmath>
#include <numbers>

namespace xe {
namespace hid {
namespace motion {

namespace {

// The weight a low-pass filter gives the newest value, for a cutoff in Hz
// over an interval in seconds.
float SmoothingFactor(float cutoff, double interval) {
  double corner = 2.0 * std::numbers::pi * double(cutoff) * interval;
  return float(corner / (corner + 1.0));
}

// The speed is itself measured between two noisy pictures, so it is
// low-passed at a fixed cutoff before it may open up the filter. One Hz is
// what the paper uses.
constexpr float kSpeedCutoff = 1.0f;

// Past this, the player was away or the camera stalled: what was remembered
// says nothing about where they are now.
constexpr double kLongestInterval = 0.5;

// 3D points are in metres, about 250 times smaller numbers than pixels.
constexpr float kPixelsPerMetre = 250.0f;

}  // namespace

PoseSmoothing::PoseSmoothing(float cutoff_at_rest, float speed_coefficient)
    : cutoff_at_rest_(cutoff_at_rest), speed_coefficient_(speed_coefficient) {}

void PoseSmoothing::Forget() {
  for (auto& point : points_) {
    for (auto& axis : point) {
      axis.started = false;
    }
  }
  for (auto& point : world_points_) {
    for (auto& axis : point) {
      axis.started = false;
    }
  }
}

void PoseSmoothing::SmoothAxis(Axis& axis, float& value, double interval,
                               float speed_coefficient) {
  if (!std::isfinite(value)) {
    axis.started = false;
    return;
  }
  if (!axis.started) {
    axis = {value, 0.0f, true};
    return;
  }
  float speed = float((double(value) - double(axis.value)) / interval);
  if (!std::isfinite(speed)) {
    axis = {value, 0.0f, true};
    return;
  }
  axis.speed += SmoothingFactor(kSpeedCutoff, interval) * (speed - axis.speed);
  float cutoff = cutoff_at_rest_ + speed_coefficient * std::fabs(axis.speed);
  axis.value += SmoothingFactor(cutoff, interval) * (value - axis.value);
  value = axis.value;
}

void PoseSmoothing::Smooth(PoseLandmarks& landmarks, double interval_seconds) {
  if (cutoff_at_rest_ <= 0.0f) {
    return;
  }
  if (!std::isfinite(interval_seconds) || interval_seconds <= 0.0 ||
      interval_seconds > kLongestInterval) {
    Forget();
    if (!std::isfinite(interval_seconds) || interval_seconds <= 0.0) {
      return;
    }
  }
  for (uint32_t point = 0; point < pose_point::kCount; ++point) {
    auto& landmark = landmarks[point];
    SmoothAxis(points_[point][0], landmark.x, interval_seconds,
               speed_coefficient_);
    SmoothAxis(points_[point][1], landmark.y, interval_seconds,
               speed_coefficient_);

    bool valid_world = landmark.has_world && std::isfinite(landmark.score) &&
                       landmark.score > 0.0f;
    for (float value : landmark.world) {
      valid_world =
          valid_world && std::isfinite(value) && std::fabs(value) <= 10.0f;
    }
    if (!valid_world) {
      for (auto& axis : world_points_[point]) {
        axis.started = false;
      }
      landmark.has_world = false;
      continue;
    }
    for (uint32_t axis = 0; axis < 3; ++axis) {
      SmoothAxis(world_points_[point][axis], landmark.world[axis],
                 interval_seconds, speed_coefficient_ * kPixelsPerMetre);
    }
  }
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
