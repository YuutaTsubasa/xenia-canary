/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_TEST_DEVICE_H_
#define XENIA_HID_MOTION_TEST_DEVICE_H_

#include <chrono>

#include "xenia/hid/motion/motion_device.h"

namespace xe {
namespace hid {
namespace motion {

// A device with no hardware behind it, for trying titles on a computer with
// no Kinect or webcam: it is always present and sees one player standing
// still 2.5 m ahead, 30 times a second, drawn as a depth picture like the
// webcam's (synthetic_depth). Whether a title's own tracker finds that
// player is not the point; whether the title gets as far as asking is.
// motion_device = test.
class TestMotionDevice final : public MotionDevice {
 public:
  const char* name() const override { return "test"; }
  bool Open() override;
  void Close() override {}

  MotionCapabilities capabilities() const override;
  MotionStatus status() const override;
  bool GetNewestFrame(SensorFrame& frame) override;

  // The player it sees, in room space (feet on the floor below a sensor
  // 0.9 m up).
  static Body StandingPlayer();

 private:
  std::chrono::steady_clock::time_point opened_at_ = {};
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_TEST_DEVICE_H_
