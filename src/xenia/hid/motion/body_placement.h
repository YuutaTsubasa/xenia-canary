/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_BODY_PLACEMENT_H_
#define XENIA_HID_MOTION_BODY_PLACEMENT_H_

#include <cstdint>

#include "xenia/hid/motion/motion_types.h"

namespace xe {
namespace hid {
namespace motion {

// Puts a webcam's skeleton where the player is in the room. PoseToBody
// gives every body with its hips at the sensor's height, 2.5 m ahead, and a
// pose model's 3D points are measured from the hips, so neither says where
// the player stands or whether they jump. The picture does, roughly:
//
// - Sideways: where the hips are across the picture, through the webcam's
//   field of view, at the body's distance. A player stepping left in front
//   of the sensor moves left in the room.
// - Up and down: how far the hips are above or below where they usually
//   are (a slowly following average of their height in the picture). A jump
//   lifts the whole body; a crouch lowers the hips while the knees bend.
// - The feet: their usual height below the hips (also a slow average) is
//   put on the floor, so a player of any build stands on it rather than
//   floating or sinking.
//
// New code, not from Free Riders Recompiled (there the title was handed the
// skeleton, and where it stood did not matter).
class BodyPlacement {
 public:
  // The webcam's horizontal field of view (most are 55 to 80 degrees), and
  // how high the sensor is above the floor (SyntheticRoom::sensor_height).
  BodyPlacement(float horizontal_fov_degrees, float sensor_height);

  // Moves `body` (as PoseToBody gives it) into the room. `hip_x`, `hip_y`
  // are the hips' centre in the picture, in pixels, with the picture
  // already the right way round (not mirrored). `interval_seconds` is the
  // time since the last call; a first call, or one after more than half a
  // second, starts the averages again.
  void Place(Body& body, float hip_x, float hip_y, uint32_t picture_width,
             uint32_t picture_height, double interval_seconds);

  // Starts again, as for a new player.
  void Forget() { started_ = false; }

  // How long the averages take to follow a change (seconds): long enough
  // that a jump stands out, short enough that stepping closer or farther
  // settles.
  static constexpr double kFollowSeconds = 2.0;
  // The foot capsule's radius in synthetic_depth.cc: the feet's joints sit
  // this far above the floor.
  static constexpr float kFootRadius = 0.045f;

 private:
  float tan_half_fov_;
  float sensor_height_;
  bool started_ = false;
  float usual_hip_y_ = 0.0f;  // pixels
  float usual_feet_ = 0.0f;   // metres below the hips (negative)
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_BODY_PLACEMENT_H_
