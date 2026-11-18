/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/motion_capture.h"

#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/base/platform.h"

#include "xenia/hid/motion/test_device.h"
#include "xenia/hid/motion/webcam_device.h"

#if XE_PLATFORM_WIN32
#include "xenia/hid/motion/kinect_v2_device.h"
#endif

DEFINE_string(motion_device, "none",
              "Motion-capture device Kinect titles see as their Kinect camera. "
              "Use: [none, auto, kinect_v1, kinect_v2, webcam, test]. test: no "
              "hardware, one player standing still, to see how far a title "
              "gets.",
              "HID");

namespace xe {
namespace hid {
namespace motion {

bool ParseMotionDeviceKind(const std::string& text, MotionDeviceKind& kind) {
  static constexpr struct {
    const char* text;
    MotionDeviceKind kind;
  } kNames[] = {
      {"none", MotionDeviceKind::kNone},
      {"auto", MotionDeviceKind::kAuto},
      {"kinect_v1", MotionDeviceKind::kKinectV1},
      {"kinect_v2", MotionDeviceKind::kKinectV2},
      {"webcam", MotionDeviceKind::kWebcam},
      {"test", MotionDeviceKind::kTest},
  };
  for (const auto& name : kNames) {
    if (text == name.text) {
      kind = name.kind;
      return true;
    }
  }
  return false;
}

const char* MotionDeviceKindName(MotionDeviceKind kind) {
  switch (kind) {
    case MotionDeviceKind::kNone:
      return "none";
    case MotionDeviceKind::kAuto:
      return "auto";
    case MotionDeviceKind::kKinectV1:
      return "kinect_v1";
    case MotionDeviceKind::kKinectV2:
      return "kinect_v2";
    case MotionDeviceKind::kWebcam:
      return "webcam";
    case MotionDeviceKind::kTest:
      return "test";
  }
  return "unknown";
}

MotionCaptureSystem::MotionCaptureSystem() {
  if (!ParseMotionDeviceKind(cvars::motion_device, kind_)) {
    XELOGW("Motion capture: unknown motion_device \"{}\", using none",
           cvars::motion_device);
    kind_ = MotionDeviceKind::kNone;
  }
  device_ = std::make_unique<NullMotionDevice>();
}

MotionCaptureSystem::~MotionCaptureSystem() { Reset(); }

std::vector<std::unique_ptr<MotionDevice>>
MotionCaptureSystem::CreateDevices() {
  // Backends are added here one by one (docs/motion-capture.md, commit
  // plan). Auto tries every one this build has, a Kinect before the webcam.
  std::vector<std::unique_ptr<MotionDevice>> devices;
  bool any = kind_ == MotionDeviceKind::kAuto;
#if XE_PLATFORM_WIN32
  if (any || kind_ == MotionDeviceKind::kKinectV2) {
    devices.push_back(std::make_unique<KinectV2Device>());
  }
#endif
  if (any || kind_ == MotionDeviceKind::kWebcam) {
    devices.push_back(std::make_unique<WebcamDevice>());
  }
  // Never chosen by auto: it is there only when asked for.
  if (kind_ == MotionDeviceKind::kTest) {
    devices.push_back(std::make_unique<TestMotionDevice>());
  }
  if (devices.empty() && kind_ != MotionDeviceKind::kNone) {
    XELOGW("Motion capture: motion_device = {} is not available in this build",
           MotionDeviceKindName(kind_));
  }
  return devices;
}

bool MotionCaptureSystem::Acquire() {
  std::lock_guard<std::mutex> guard(lock_);
  if (!tried_) {
    tried_ = true;
    for (auto& device : CreateDevices()) {
      open_ = device->Open();
      MotionStatus status = device->status();
      XELOGI("Motion capture: {} {}{}{}", device->name(),
             open_ ? "running" : "not running",
             status.reason.empty() ? "" : ": ", status.reason);
      device_ = std::move(device);
      if (open_) {
        break;
      }
    }
  }
  return open_;
}

void MotionCaptureSystem::Reset() {
  std::lock_guard<std::mutex> guard(lock_);
  if (open_) {
    device_->Close();
  }
  device_ = std::make_unique<NullMotionDevice>();
  tried_ = false;
  open_ = false;
}

MotionCapabilities MotionCaptureSystem::capabilities() {
  std::lock_guard<std::mutex> guard(lock_);
  return open_ ? device_->capabilities() : MotionCapabilities();
}

MotionStatus MotionCaptureSystem::status() {
  std::lock_guard<std::mutex> guard(lock_);
  return device_->status();
}

bool MotionCaptureSystem::GetNewestFrame(SensorFrame& frame) {
  std::lock_guard<std::mutex> guard(lock_);
  return open_ && device_->GetNewestFrame(frame);
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
