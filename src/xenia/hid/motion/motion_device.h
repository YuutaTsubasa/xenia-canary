/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_MOTION_DEVICE_H_
#define XENIA_HID_MOTION_MOTION_DEVICE_H_

#include "xenia/hid/motion/motion_types.h"

namespace xe {
namespace hid {
namespace motion {

// One host motion-capture device (a backend). A backend only opens, hands
// over its newest frame and says what it can do; turning frames into what
// the guest expects is not its job.
class MotionDevice {
 public:
  virtual ~MotionDevice();

  // A short name for logs ("Kinect v2").
  virtual const char* name() const = 0;

  // Starts the device. False when it cannot run; status() says why.
  virtual bool Open() = 0;
  virtual void Close() = 0;

  virtual MotionCapabilities capabilities() const = 0;
  virtual MotionStatus status() const = 0;

  // Copies the newest frame into `frame` if it is newer than frame.number,
  // and returns whether it did. Never blocks for long: a device captures on
  // its own thread.
  virtual bool GetNewestFrame(SensorFrame& frame) = 0;

  // The tilt motor, for devices that have one (capabilities().tilt_motor).
  // Degrees above level.
  virtual bool SetTilt(float degrees) { return false; }
  virtual bool GetTilt(float& degrees) { return false; }
};

// The device used when none is chosen: never opens, never has a frame.
class NullMotionDevice final : public MotionDevice {
 public:
  const char* name() const override { return "none"; }
  bool Open() override { return false; }
  void Close() override {}
  MotionCapabilities capabilities() const override { return {}; }
  MotionStatus status() const override { return {}; }
  bool GetNewestFrame(SensorFrame& frame) override { return false; }
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_MOTION_DEVICE_H_
