/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_POSE_SMOOTHING_H_
#define XENIA_HID_MOTION_POSE_SMOOTHING_H_

#include <array>

#include "xenia/hid/motion/pose_landmarks.h"

// Steadies a pose model's points between pictures. Ported from Free Riders
// Recompiled (pose_smoothing.cpp), written by its author YuutaTsubasa and
// brought here under this file's license.

namespace xe {
namespace hid {
namespace motion {

// A pose model reads every picture on its own, so a point that is not moving
// still wanders a pixel or two from one picture to the next. Drawn into the
// depth picture, that wandering makes the title's tracker see a trembling
// body.
//
// This is the 1 Euro filter (Casiez, Roussel and Vogel, 2012): a low-pass
// filter whose cutoff rises with how fast the point moves. A still point is
// smoothed hard, so it settles; a real movement raises the cutoff at once, so
// the body does not lag behind the player.
class PoseSmoothing {
 public:
  // The defaults hold a still hand near 1 Hz and open to about 11 Hz for a
  // hand moving 200 pixels a second.
  static constexpr float kDefaultCutoffAtRest = 1.0f;       // Hz
  static constexpr float kDefaultSpeedCoefficient = 0.05f;  // Hz per px/s

  // The cutoff at rest in Hz (0 passes points through untouched), and how
  // much each pixel a second of speed raises it. 3D points use the same
  // tuning at 250 pixels to the metre.
  PoseSmoothing(float cutoff_at_rest = kDefaultCutoffAtRest,
                float speed_coefficient = kDefaultSpeedCoefficient);

  // Smooths in place. `interval_seconds` is the time since the previous
  // picture; the first picture, or one after a gap of more than half a
  // second, starts the filter again. A 3D point that is missing, not finite
  // or has no confidence is marked has_world = false and starts again.
  void Smooth(PoseLandmarks& landmarks, double interval_seconds);
  // Forgets every point, as after a gap.
  void Forget();

 private:
  struct Axis {
    float value = 0.0f, speed = 0.0f;
    bool started = false;
  };
  void SmoothAxis(Axis& axis, float& value, double interval,
                  float speed_coefficient);

  float cutoff_at_rest_, speed_coefficient_;
  std::array<std::array<Axis, 2>, pose_point::kCount> points_ = {};
  std::array<std::array<Axis, 3>, pose_point::kCount> world_points_ = {};
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_POSE_SMOOTHING_H_
