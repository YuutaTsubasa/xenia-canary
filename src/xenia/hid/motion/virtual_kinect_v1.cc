/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/virtual_kinect_v1.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "xenia/base/byte_order.h"

namespace xe {
namespace hid {
namespace motion {

CameraIntrinsics V1DepthLens() {
  CameraIntrinsics lens;
  lens.focal_x = kV1DepthFocalLength;
  lens.focal_y = kV1DepthFocalLength;
  lens.centre_x = kV1DepthWidth * 0.5f;
  lens.centre_y = kV1DepthHeight * 0.5f;
  return lens;
}

DepthImage ReprojectToV1Depth(const DepthImage& source,
                              const CameraPose& pose) {
  return ReprojectDepth(source, V1DepthLens(), kV1DepthWidth, kV1DepthHeight,
                        pose);
}

CameraPose V1TiltPose(float degrees) {
  const float angle = degrees * std::numbers::pi_v<float> / 180.0f;
  const float c = std::cos(angle), s = std::sin(angle);
  // Lens space has +y down: a camera tilted up sees what was straight
  // ahead lower in its picture (y' = y cos + z sin).
  CameraPose pose;
  pose.rotation = {1, 0, 0, 0, c, s, 0, -s, c};
  return pose;
}

void FillV1DepthTable(std::span<uint16_t, kV1DepthTableEntries> table) {
  for (size_t i = 0; i < table.size(); ++i) {
    table[i] = uint16_t(i);
  }
}

bool EncodeV1DepthFrame(const DepthImage& depth, std::span<uint8_t> out) {
  if (depth.width != kV1DepthWidth || depth.height != kV1DepthHeight ||
      depth.millimetres.size() != size_t(kV1DepthWidth) * kV1DepthHeight ||
      out.size() != kV1DepthFrameBytes) {
    return false;
  }
  auto* pixels = reinterpret_cast<uint16_t*>(out.data());
  for (size_t i = 0; i < depth.millimetres.size(); ++i) {
    uint16_t mm = std::min(depth.millimetres[i], kV1DepthMaxValue);
    pixels[i] = xe::byte_swap(mm);
  }
  return true;
}

bool EncodeV1ColourFrame(const DepthImage& depth, std::span<uint8_t> out) {
  if (depth.width != kV1ColourWidth || depth.height != kV1ColourHeight ||
      !depth.has_colour() || out.size() != kV1ColourFrameBytes) {
    return false;
  }
  for (uint32_t y = 0; y < kV1ColourHeight; ++y) {
    for (uint32_t x = 0; x < kV1ColourWidth; ++x) {
      size_t i = size_t(y) * kV1ColourWidth + x;
      const uint8_t* rgba = &depth.rgba[i * 4];
      // GRBG: green where x + y is even, red on even rows, blue on odd.
      size_t channel = (x + y) % 2 == 0 ? 1 : (y % 2 == 0 ? 0 : 2);
      out[i] = rgba[3] ? rgba[channel] : kV1ColourGrey;
    }
  }
  return true;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
