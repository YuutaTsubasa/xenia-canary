/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/body_placement.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace xe {
namespace hid {
namespace motion {

BodyPlacement::BodyPlacement(float horizontal_fov_degrees, float sensor_height)
    : tan_half_fov_(std::tan(std::clamp(horizontal_fov_degrees, 10.0f, 170.0f) *
                             std::numbers::pi_v<float> / 360.0f)),
      sensor_height_(sensor_height) {}

void BodyPlacement::Place(Body& body, float hip_x, float hip_y,
                          uint32_t picture_width, uint32_t picture_height,
                          double interval_seconds) {
  if (!picture_width || !picture_height) {
    return;
  }
  const auto& hips = body.joints[size_t(Joint::kHipCenter)];
  float distance = hips[2];
  float feet = std::min(body.joints[size_t(Joint::kFootLeft)][1],
                        body.joints[size_t(Joint::kFootRight)][1]) -
               hips[1];

  if (!started_ || !std::isfinite(interval_seconds) || interval_seconds <= 0 ||
      interval_seconds > 0.5) {
    started_ = true;
    usual_hip_y_ = hip_y;
    usual_feet_ = feet;
  } else {
    float follow = float(1.0 - std::exp(-interval_seconds / kFollowSeconds));
    usual_hip_y_ += follow * (hip_y - usual_hip_y_);
    usual_feet_ += follow * (feet - usual_feet_);
  }

  // Pixels to metres at the body's distance, through the webcam's lens (an
  // ideal pinhole as wide as its field of view).
  float metres_per_pixel =
      distance * tan_half_fov_ / (0.5f * float(picture_width));
  // The picture's left is the sensor's left (+x); its top is up.
  float across = (0.5f * float(picture_width) - hip_x) * metres_per_pixel;
  float lift = (usual_hip_y_ - hip_y) * metres_per_pixel;
  // Where the hips are when the usual feet stand on the floor, then the
  // jump or crouch on top.
  float hip_height = -sensor_height_ + kFootRadius - usual_feet_ + lift;

  float shift_x = across - hips[0];
  float shift_y = hip_height - hips[1];
  for (auto& joint : body.joints) {
    joint[0] += shift_x;
    joint[1] += shift_y;
  }
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
