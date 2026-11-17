/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_KINECT_V2_DEVICE_H_
#define XENIA_HID_MOTION_KINECT_V2_DEVICE_H_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "xenia/hid/motion/motion_device.h"

namespace xe {
namespace hid {
namespace motion {

// A Kinect for Xbox One / Kinect for Windows v2, through Microsoft's Kinect
// for Windows Runtime 2.0 (Windows only). Gives depth for now; the title
// tracks the player from it itself.
//
// Everything that talks to the runtime happens on the device's own capture
// thread, so the runtime's COM objects never meet the emulator's threads.
class KinectV2Device final : public MotionDevice {
 public:
  KinectV2Device();
  ~KinectV2Device() override;

  const char* name() const override { return "Kinect v2"; }
  // Loads the runtime and waits a few seconds for a sensor to be ready.
  bool Open() override;
  void Close() override;

  MotionCapabilities capabilities() const override;
  MotionStatus status() const override;
  bool GetNewestFrame(SensorFrame& frame) override;

 private:
  void CaptureMain();
  void SetStatus(MotionDeviceState state, std::string reason);

  void* library_ = nullptr;  // Kinect20.dll
  std::thread thread_;
  std::atomic<bool> stopping_ = false;

  mutable std::mutex lock_;
  std::condition_variable status_changed_;
  MotionStatus status_;
  SensorFrame newest_;
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_KINECT_V2_DEVICE_H_
