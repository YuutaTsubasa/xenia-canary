/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_WEBCAM_CAPTURE_H_
#define XENIA_HID_MOTION_WEBCAM_CAPTURE_H_

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "xenia/hid/motion/motion_types.h"

// Pictures from a webcam, one platform backend each: Media Foundation on
// Windows (webcam_capture_win.cc), V4L2 on Linux (webcam_capture_v4l2.cc).

namespace xe {
namespace hid {
namespace motion {

class WebcamCapture {
 public:
  virtual ~WebcamCapture();

  // The camera's name as List() gives it.
  virtual const std::string& name() const = 0;

  // Waits up to about a frame's time for the next picture, as RGBA (alpha
  // 255). False when none came; `picture` is then left alone.
  virtual bool Read(ColourImage& picture) = 0;

  // The cameras there are, by name.
  static std::vector<std::string> List();

  // Opens the first camera whose name contains `wanted` (ignoring case), or
  // the first camera if `wanted` is empty, at the picture size nearest
  // `width` x `height` (at least that, if it can). Null, with `reason` in
  // words a player can act on, when none can be opened.
  //
  // On Windows, call it from a thread with COM initialised (multithreaded),
  // and use the capture from that thread only.
  static std::unique_ptr<WebcamCapture> Open(const std::string& wanted,
                                             uint32_t width, uint32_t height,
                                             std::string& reason);
};

// Pixel formats cameras send, to RGBA. `stride` is the bytes from one row's
// start to the next; it must cover the row. Each returns false (leaving
// `picture` alone) when `source` is too short.

// 4 bytes a pixel: blue, green, red, then unused.
bool BgrxToRgba(std::span<const uint8_t> source, uint32_t width,
                uint32_t height, uint32_t stride, ColourImage& picture);
// YUY2 (YUYV): Y0 U Y1 V for each two pixels, BT.601 studio range.
bool Yuy2ToRgba(std::span<const uint8_t> source, uint32_t width,
                uint32_t height, uint32_t stride, ColourImage& picture);
// NV12: a full Y plane, then a half-size plane of U V pairs, BT.601 studio
// range; both planes have `stride`.
bool Nv12ToRgba(std::span<const uint8_t> source, uint32_t width,
                uint32_t height, uint32_t stride, ColourImage& picture);

// Which of `names` `wanted` picks (first containing it, ignoring case; the
// first name if `wanted` is empty), or -1 for none.
int ChooseWebcam(const std::vector<std::string>& names,
                 const std::string& wanted);

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_WEBCAM_CAPTURE_H_
