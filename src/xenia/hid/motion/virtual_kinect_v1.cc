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

namespace {

// Full-range BT.601 (as JPEG uses), rounded and kept in 0..255.
uint8_t ToByte(float value) {
  return uint8_t(std::clamp(value + 0.5f, 0.0f, 255.0f));
}
float Luma(const uint8_t* rgba) {
  return 0.299f * rgba[0] + 0.587f * rgba[1] + 0.114f * rgba[2];
}

void EncodeUyvy(const DepthImage& depth, std::span<uint8_t> out) {
  for (uint32_t y = 0; y < kV1ColourHeight; ++y) {
    for (uint32_t x = 0; x < kV1ColourWidth; x += 2) {
      size_t i = size_t(y) * kV1ColourWidth + x;
      const uint8_t* left = &depth.rgba[i * 4];
      const uint8_t* right = left + 4;
      uint8_t* pair = &out[i * 2];
      if (!left[3] && !right[3]) {
        std::fill_n(pair, 4, kV1ColourGrey);
        continue;
      }
      // A pixel the colour camera didn't see borrows its neighbour's colour.
      if (!left[3]) left = right;
      if (!right[3]) right = left;
      float r = (left[0] + right[0]) * 0.5f;
      float g = (left[1] + right[1]) * 0.5f;
      float b = (left[2] + right[2]) * 0.5f;
      pair[0] = ToByte(128.0f - 0.168736f * r - 0.331264f * g + 0.5f * b);
      pair[1] = ToByte(Luma(left));
      pair[2] = ToByte(128.0f + 0.5f * r - 0.418688f * g - 0.081312f * b);
      pair[3] = ToByte(Luma(right));
    }
  }
}

}  // namespace

bool EncodeV1ColourFrame(const DepthImage& depth, std::span<uint8_t> out) {
  if (depth.width != kV1ColourWidth || depth.height != kV1ColourHeight ||
      !depth.has_colour()) {
    return false;
  }
  if (out.size() == kV1ColourFrameBytes * 2) {
    EncodeUyvy(depth, out);
    return true;
  }
  if (out.size() != kV1ColourFrameBytes) {
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
