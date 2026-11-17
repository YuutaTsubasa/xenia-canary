/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_DEPTH_REPROJECTION_H_
#define XENIA_HID_MOTION_DEPTH_REPROJECTION_H_

#include <array>
#include <cstdint>

#include "xenia/hid/motion/motion_types.h"

// Moving a depth image from one camera into another, imaginary one.
//
// A Kinect v2's depth camera is not a Kinect v1's: another resolution
// (512x424), a wider lens and a different place. Resampling its picture
// would stretch the player; instead every depth pixel becomes a point in
// space, the point is moved into the target camera and projected through
// the target's lens, and where several points land on one target pixel the
// nearest wins (as a real camera would see it).
//
// Written from the standard pinhole and Brown-Conrady lens models; the
// distortion inversion is the usual fixed-point iteration.
//
// Lens space throughout: metres, +x to the image's right, +y down, +z
// forward from the lens.

namespace xe {
namespace hid {
namespace motion {

// Where the target camera is relative to the source: a point p in the
// source's lens space is rotation * p + translation in the target's.
// Row-major rotation. The default is "the same place".
struct CameraPose {
  std::array<float, 9> rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::array<float, 3> translation = {0, 0, 0};
};

// The ray (x/z, y/z) of a pixel position (u, v) through `lens`, with the
// lens distortion removed. Pixel (0, 0)'s centre is at (0.5, 0.5).
std::array<float, 2> PixelToRay(const CameraIntrinsics& lens, float u, float v);

// The pixel position a ray (x/z, y/z) falls on through `lens`, distortion
// applied. The inverse of PixelToRay.
std::array<float, 2> RayToPixel(const CameraIntrinsics& lens, float x, float y);

// Reprojects `source` into a camera with `target_lens`, `target_width` x
// `target_height` pixels, standing at `pose` from the source. Pixels
// nothing lands on are 0 (no reading); the target's own distortion is
// ignored (an imaginary camera has none). Each source point covers as many
// target pixels as its size there needs (up to 3x3), so a coarser source
// does not leave a grid of holes.
DepthImage ReprojectDepth(const DepthImage& source,
                          const CameraIntrinsics& target_lens,
                          uint32_t target_width, uint32_t target_height,
                          const CameraPose& pose = {});

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_DEPTH_REPROJECTION_H_
