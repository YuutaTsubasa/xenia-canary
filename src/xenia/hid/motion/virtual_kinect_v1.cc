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

#include "xenia/base/byte_order.h"

namespace xe {
namespace hid {
namespace motion {

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

}  // namespace motion
}  // namespace hid
}  // namespace xe
