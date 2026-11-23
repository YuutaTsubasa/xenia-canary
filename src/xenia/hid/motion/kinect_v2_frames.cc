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
#include <cstring>

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

std::vector<uint8_t> KinectV2ColourOnDepth(std::span<const float> points,
                                           std::span<const uint8_t> colour_rgba,
                                           uint32_t colour_width,
                                           uint32_t colour_height) {
  const size_t count = points.size() / 2;
  std::vector<uint8_t> rgba(count * 4, 0);
  if (colour_rgba.size() != size_t(colour_width) * colour_height * 4) {
    return rgba;
  }
  for (size_t i = 0; i < count; ++i) {
    float x = points[i * 2], y = points[i * 2 + 1];
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    // The nearest colour pixel.
    float cx = std::floor(x + 0.5f), cy = std::floor(y + 0.5f);
    if (cx < 0.0f || cy < 0.0f || cx >= float(colour_width) ||
        cy >= float(colour_height)) {
      continue;
    }
    size_t from = (size_t(cy) * colour_width + size_t(cx)) * 4;
    std::memcpy(&rgba[i * 4], &colour_rgba[from], 3);
    rgba[i * 4 + 3] = 255;
  }
  return rgba;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
