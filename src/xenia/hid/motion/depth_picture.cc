/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/depth_picture.h"

#include <algorithm>

namespace xe {
namespace hid {
namespace motion {

namespace {

void PutU16(std::vector<uint8_t>& out, size_t at, uint32_t value) {
  out[at] = uint8_t(value);
  out[at + 1] = uint8_t(value >> 8);
}

void PutU32(std::vector<uint8_t>& out, size_t at, uint32_t value) {
  PutU16(out, at, value & 0xFFFF);
  PutU16(out, at + 2, value >> 16);
}

uint8_t Grey(uint16_t mm) {
  if (!mm) {
    return 0;
  }
  constexpr int kNear = 500, kFar = 4500;
  int clamped = std::clamp(int(mm), kNear, kFar);
  // 255 at 0.5 m down to 40 at 4.5 m, so the far wall is still not black.
  return uint8_t(255 - (clamped - kNear) * (255 - 40) / (kFar - kNear));
}

}  // namespace

DepthSummary SummarizeDepth(const DepthImage& depth) {
  DepthSummary summary;
  const size_t count = size_t(depth.width) * depth.height;
  if (!count || depth.millimetres.size() != count) {
    return summary;
  }
  size_t with_reading = 0;
  for (uint16_t mm : depth.millimetres) {
    if (!mm) {
      continue;
    }
    ++with_reading;
    if (!summary.nearest_mm || mm < summary.nearest_mm) {
      summary.nearest_mm = mm;
    }
    if (mm < 3000) {
      ++summary.nearer_than_3m;
    }
  }
  summary.with_reading = float(with_reading) / float(count);
  summary.middle_mm =
      depth.millimetres[size_t(depth.height / 2) * depth.width +
                        depth.width / 2];
  return summary;
}

std::vector<uint8_t> DepthToBitmap(const DepthImage& depth) {
  const size_t count = size_t(depth.width) * depth.height;
  if (!count || depth.millimetres.size() != count) {
    return {};
  }
  // BITMAPFILEHEADER (14) + BITMAPINFOHEADER (40) + 256 grey palette
  // entries, then rows padded to 4 bytes. A negative height means the rows
  // are stored top first.
  const uint32_t row_bytes = (depth.width + 3) & ~3u;
  const uint32_t pixels_at = 14 + 40 + 256 * 4;
  std::vector<uint8_t> out(pixels_at + size_t(row_bytes) * depth.height, 0);
  out[0] = 'B';
  out[1] = 'M';
  PutU32(out, 2, uint32_t(out.size()));
  PutU32(out, 10, pixels_at);
  PutU32(out, 14, 40);
  PutU32(out, 18, depth.width);
  PutU32(out, 22, uint32_t(-int32_t(depth.height)));
  PutU16(out, 26, 1);  // planes
  PutU16(out, 28, 8);  // bits per pixel
  PutU32(out, 46, 256);  // palette entries
  for (uint32_t i = 0; i < 256; ++i) {
    size_t at = 54 + i * 4;
    out[at] = out[at + 1] = out[at + 2] = uint8_t(i);
  }
  for (uint32_t y = 0; y < depth.height; ++y) {
    for (uint32_t x = 0; x < depth.width; ++x) {
      out[pixels_at + size_t(y) * row_bytes + x] =
          Grey(depth.millimetres[size_t(y) * depth.width + x]);
    }
  }
  return out;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
