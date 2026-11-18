/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_POSE_LANDMARKS_H_
#define XENIA_HID_MOTION_POSE_LANDMARKS_H_

#include <array>
#include <cstdint>

// What a pose model finds in one webcam picture. Ported from Free Riders
// Recompiled (pose_estimator.h), written by its author YuutaTsubasa and
// brought here under this file's license.

namespace xe {
namespace hid {
namespace motion {

// The seventeen body points, in the order COCO numbers them. "Left" and
// "right" are the person's own sides, not the picture's.
namespace pose_point {
constexpr uint32_t kNose = 0, kEyeLeft = 1, kEyeRight = 2, kEarLeft = 3,
                   kEarRight = 4;
constexpr uint32_t kShoulderLeft = 5, kShoulderRight = 6, kElbowLeft = 7,
                   kElbowRight = 8;
constexpr uint32_t kWristLeft = 9, kWristRight = 10, kHipLeft = 11,
                   kHipRight = 12;
constexpr uint32_t kKneeLeft = 13, kKneeRight = 14, kAnkleLeft = 15,
                   kAnkleRight = 16;
constexpr uint32_t kCount = 17;
}  // namespace pose_point

// One point, in the picture's pixels, with the model's confidence (0..1).
struct PoseLandmark {
  float x = 0.0f, y = 0.0f, score = 0.0f;
  // Metres around the hips, oriented like the picture: +x to the picture's
  // right, +y down, +z away from the camera. Only models that estimate 3D
  // (MediaPipe pose) fill it.
  std::array<float, 3> world = {};
  bool has_world = false;
};
using PoseLandmarks = std::array<PoseLandmark, pose_point::kCount>;

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_POSE_LANDMARKS_H_
