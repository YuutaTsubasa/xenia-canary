/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/kinect_v2_frames.h"

#include <cmath>

namespace xe {
namespace hid {
namespace motion {

std::vector<float> KinectV2TableToRays(std::span<const float> table) {
  std::vector<float> rays(table.size());
  for (size_t i = 0; i < table.size(); ++i) {
    rays[i] = -table[i];
  }
  return rays;
}

bool KinectV2TableLooksValid(std::span<const float> table, uint32_t width,
                             uint32_t height) {
  if (!width || !height || table.size() != size_t(width) * height * 2) {
    return false;
  }
  // The top-left corner pixel looks well off the axis both ways (about
  // 0.7 and 0.6 for a Kinect v2); 0.2 only rules out an empty table.
  float corner_x = table[0], corner_y = table[1];
  return std::isfinite(corner_x) && std::isfinite(corner_y) &&
         std::abs(corner_x) > 0.2f && std::abs(corner_y) > 0.2f;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
