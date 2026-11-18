/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// Platforms without a webcam backend yet.

#include "xenia/hid/motion/webcam_capture.h"

namespace xe {
namespace hid {
namespace motion {

std::vector<std::string> WebcamCapture::List() { return {}; }

std::unique_ptr<WebcamCapture> WebcamCapture::Open(const std::string& wanted,
                                                   uint32_t width,
                                                   uint32_t height,
                                                   std::string& reason) {
  reason = "Webcams are not supported on this platform yet";
  return nullptr;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
