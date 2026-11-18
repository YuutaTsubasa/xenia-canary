/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_MOTION_CAPTURE_H_
#define XENIA_HID_MOTION_MOTION_CAPTURE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xenia/hid/motion/motion_device.h"

namespace xe {
namespace hid {
namespace motion {

// Which kind of device the player chose (the motion_device option).
enum class MotionDeviceKind : uint8_t {
  kNone,
  kAuto,  // the first one found: Kinect v1, then Kinect v2, then webcam
  kKinectV1,
  kKinectV2,
  kWebcam,
  kTest,  // no hardware: one player standing still (test_device.h)
};

// Reads the option's text ("none", "auto", "kinect_v1", "kinect_v2",
// "webcam", "test"); false for anything else.
bool ParseMotionDeviceKind(const std::string& text, MotionDeviceKind& kind);
const char* MotionDeviceKindName(MotionDeviceKind kind);

// The one place the emulator gets motion-capture frames from. It picks a
// device from the motion_device option, opens it the first time the guest
// asks (a camera takes a moment to start, and most titles never ask), and
// closes it when the emulator shuts down.
//
// Owned by hid::InputSystem. Safe to call from any thread.
class MotionCaptureSystem {
 public:
  MotionCaptureSystem();
  ~MotionCaptureSystem();

  MotionDeviceKind kind() const { return kind_; }

  // Opens the chosen device if it isn't yet. Returns whether a device is
  // running afterwards. Only the first call tries; a device that failed is
  // not retried until Reset().
  bool Acquire();

  // Closes the device; the next Acquire() opens it again.
  void Reset();

  MotionCapabilities capabilities();
  MotionStatus status();
  bool GetNewestFrame(SensorFrame& frame);

 private:
  // The backends to try for kind_, in order (auto lists every one this
  // build has); empty when this build has none.
  std::vector<std::unique_ptr<MotionDevice>> CreateDevices();

  std::mutex lock_;
  MotionDeviceKind kind_ = MotionDeviceKind::kNone;
  std::unique_ptr<MotionDevice> device_;
  bool tried_ = false;
  bool open_ = false;
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_MOTION_CAPTURE_H_
