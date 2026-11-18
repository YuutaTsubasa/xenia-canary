/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// The parts of webcam capture every platform shares: pixel conversion and
// picking a camera by name. YUV conversion is the BT.601 studio-range one
// (ITU-R BT.601; Y 16..235, U and V 16..240), in 8.8 fixed point.

#include <algorithm>
#include <cctype>

#include "xenia/hid/motion/webcam_capture.h"

namespace xe {
namespace hid {
namespace motion {

namespace {

uint8_t Clamp(int value) { return uint8_t(std::clamp(value, 0, 255)); }

void YuvToRgba(int y, int u, int v, uint8_t* out) {
  int c = 298 * (y - 16);
  int d = u - 128, e = v - 128;
  out[0] = Clamp((c + 409 * e + 128) >> 8);
  out[1] = Clamp((c - 100 * d - 208 * e + 128) >> 8);
  out[2] = Clamp((c + 516 * d + 128) >> 8);
  out[3] = 255;
}

bool Fits(std::span<const uint8_t> source, uint32_t row_bytes, uint32_t stride,
          uint32_t rows) {
  return rows && stride >= row_bytes &&
         source.size() >= size_t(stride) * (rows - 1) + row_bytes;
}

void Size(ColourImage& picture, uint32_t width, uint32_t height) {
  picture.width = width;
  picture.height = height;
  picture.rgba.resize(size_t(width) * height * 4);
}

}  // namespace

WebcamCapture::~WebcamCapture() = default;

bool BgrxToRgba(std::span<const uint8_t> source, uint32_t width,
                uint32_t height, uint32_t stride, ColourImage& picture) {
  if (!width || !Fits(source, width * 4, stride, height)) {
    return false;
  }
  Size(picture, width, height);
  for (uint32_t y = 0; y < height; ++y) {
    const uint8_t* in = source.data() + size_t(y) * stride;
    uint8_t* out = picture.rgba.data() + size_t(y) * width * 4;
    for (uint32_t x = 0; x < width; ++x, in += 4, out += 4) {
      out[0] = in[2];
      out[1] = in[1];
      out[2] = in[0];
      out[3] = 255;
    }
  }
  return true;
}

bool Yuy2ToRgba(std::span<const uint8_t> source, uint32_t width,
                uint32_t height, uint32_t stride, ColourImage& picture) {
  if (!width || width % 2 || !Fits(source, width * 2, stride, height)) {
    return false;
  }
  Size(picture, width, height);
  for (uint32_t y = 0; y < height; ++y) {
    const uint8_t* in = source.data() + size_t(y) * stride;
    uint8_t* out = picture.rgba.data() + size_t(y) * width * 4;
    for (uint32_t x = 0; x < width; x += 2, in += 4, out += 8) {
      YuvToRgba(in[0], in[1], in[3], out);
      YuvToRgba(in[2], in[1], in[3], out + 4);
    }
  }
  return true;
}

bool Nv12ToRgba(std::span<const uint8_t> source, uint32_t width,
                uint32_t height, uint32_t stride, ColourImage& picture) {
  if (!width || width % 2 || height % 2 ||
      !Fits(source, width, stride, height + height / 2)) {
    return false;
  }
  Size(picture, width, height);
  const uint8_t* chroma = source.data() + size_t(stride) * height;
  for (uint32_t y = 0; y < height; ++y) {
    const uint8_t* luma = source.data() + size_t(y) * stride;
    const uint8_t* pairs = chroma + size_t(y / 2) * stride;
    uint8_t* out = picture.rgba.data() + size_t(y) * width * 4;
    for (uint32_t x = 0; x < width; ++x, out += 4) {
      YuvToRgba(luma[x], pairs[x & ~1u], pairs[(x & ~1u) + 1], out);
    }
  }
  return true;
}

int ChooseWebcam(const std::vector<std::string>& names,
                 const std::string& wanted) {
  if (names.empty()) {
    return -1;
  }
  if (wanted.empty()) {
    return 0;
  }
  auto lower = [](std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return char(std::tolower(c)); });
    return text;
  };
  std::string needle = lower(wanted);
  for (size_t i = 0; i < names.size(); ++i) {
    if (lower(names[i]).find(needle) != std::string::npos) {
      return int(i);
    }
  }
  return -1;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
