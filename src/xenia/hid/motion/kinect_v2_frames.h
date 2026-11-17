/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_KINECT_V2_FRAMES_H_
#define XENIA_HID_MOTION_KINECT_V2_FRAMES_H_

#include <cstdint>
#include <span>
#include <vector>

#include "xenia/hid/motion/motion_types.h"

// The parts of the Kinect v2 backend that need no Kinect: turning what the
// Kinect for Windows SDK 2.0 reports into motion-capture types. Kept apart
// from the device code so they build and are tested on every platform.

namespace xe {
namespace hid {
namespace motion {

// The Kinect v2's depth image size.
constexpr uint32_t kV2DepthWidth = 512;
constexpr uint32_t kV2DepthHeight = 424;

// The SDK's depth-to-camera-space table (ICoordinateMapper::
// GetDepthFrameToCameraSpaceTable) gives, per depth pixel, X and Y such that
// the point seen at depth z metres is (X * z, Y * z, z) in the SDK's camera
// space: +X to the sensor's left (the player's right), +Y up, +Z out of the
// sensor. Our lens space looks out of the camera with +x right and +y down,
// so both axes turn round. `table` is X, Y pairs; the result is the rays of
// DepthImage::rays. Entries that are not finite stay not finite (the
// reprojection skips them).
std::vector<float> KinectV2TableToRays(std::span<const float> table);

// Whether a table looks real. Until the sensor has sent its calibration the
// SDK may hand back a table of zeros; a real one spreads out to the edges of
// the roughly 70 by 60 degree view.
bool KinectV2TableLooksValid(std::span<const float> table, uint32_t width,
                             uint32_t height);

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_KINECT_V2_FRAMES_H_
