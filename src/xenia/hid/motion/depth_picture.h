/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_DEPTH_PICTURE_H_
#define XENIA_HID_MOTION_DEPTH_PICTURE_H_

#include <cstdint>
#include <vector>

#include "xenia/hid/motion/motion_types.h"

// Looking at a depth picture from outside: a few numbers for the log, and a
// grey picture a person can open. For a tester's report, when a title does
// not see the player and we cannot tell from the log why.

namespace xe {
namespace hid {
namespace motion {

struct DepthSummary {
  // Share of pixels with a reading, 0 to 1.
  float with_reading = 0.0f;
  // The nearest reading, and the one in the middle of the picture
  // (0 = none there).
  uint16_t nearest_mm = 0;
  uint16_t middle_mm = 0;
  // Pixels nearer than 3 m: about where a player stands.
  uint32_t nearer_than_3m = 0;
};

DepthSummary SummarizeDepth(const DepthImage& depth);

// The picture as an 8-bit grey Windows bitmap (.bmp), top row first: near
// is light, far is dark (0.5 m white, 4.5 m and beyond dim grey), no
// reading is black. Empty if the picture is.
std::vector<uint8_t> DepthToBitmap(const DepthImage& depth);

// The picture's colour (DepthImage::rgba) as a 24-bit Windows bitmap,
// top row first; pixels without colour are mid grey. This is what the
// colour frames the title gets are made from. Empty if there is no colour.
std::vector<uint8_t> ColourToBitmap(const DepthImage& depth);

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_DEPTH_PICTURE_H_
