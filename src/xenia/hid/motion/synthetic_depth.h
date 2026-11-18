/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_SYNTHETIC_DEPTH_H_
#define XENIA_HID_MOTION_SYNTHETIC_DEPTH_H_

#include "xenia/hid/motion/motion_types.h"

// A depth picture drawn from a skeleton, for devices that see no depth (a
// webcam with a pose model). The title's own tracker reads bodies out of
// depth pictures (docs/motion-capture.md, path A), so the skeleton is drawn
// as a body of rounded limbs standing in a bare room, as the virtual Kinect
// v1 would see it.
//
// Experimental: whether a title's tracker accepts a drawn body is only known
// once tried.

namespace xe {
namespace hid {
namespace motion {

// The room the body is drawn in, in room space: metres from the sensor, +x
// to the sensor's left (skeleton space's +x, the player's right), +y up
// against gravity, +z ahead along the floor.
struct SyntheticRoom {
  // How high the sensor is above the floor. A body whose hips are at
  // skeleton y = 0 (as PoseToBody puts them) has them this high.
  float sensor_height = 0.9f;
  // How far ahead the back wall is; within a Kinect v1's range (about 4 m),
  // so the room reads as a room, not as "too far".
  float back_wall = 3.8f;
  // The sensor's tilt, degrees above level (the virtual tilt motor's
  // angle). The room and the body stay upright; the view turns.
  float tilt_degrees = 0.0f;
};

// The room, and the body in it if `body` is not null (its joints in room
// space; joints that are not tracked, and the limbs to them, are left out),
// as a 640x480 depth picture through the virtual v1's lens (V1DepthLens()),
// ready for EncodeV1DepthFrame. The wall is endless, so within the motor's
// range every pixel sees something; one that saw nothing would read 0.
DepthImage DrawSyntheticDepth(const Body* body, const SyntheticRoom& room);

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_SYNTHETIC_DEPTH_H_
