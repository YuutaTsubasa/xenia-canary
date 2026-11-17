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

#if XE_PLATFORM_WIN32
#include "xenia/hid/motion/kinect_v2_device.h"
#endif

DEFINE_string(motion_device, "none",
              "Motion-capture device Kinect titles see as their Kinect camera. "
              "Use: [none, auto, kinect_v1, kinect_v2, webcam]",
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

std::unique_ptr<MotionDevice> MotionCaptureSystem::CreateDevice() {
  // Backends are added here one by one (docs/motion-capture.md, commit
  // plan). Auto takes the first that exists in this build.
  switch (kind_) {
    case MotionDeviceKind::kAuto:
    case MotionDeviceKind::kKinectV2:
#if XE_PLATFORM_WIN32
      return std::make_unique<KinectV2Device>();
#else
      break;
#endif
    default:
      break;
  }
  if (kind_ != MotionDeviceKind::kNone) {
    XELOGW("Motion capture: motion_device = {} is not available in this build",
           MotionDeviceKindName(kind_));
  }
  return std::make_unique<NullMotionDevice>();
}

bool MotionCaptureSystem::Acquire() {
  std::lock_guard<std::mutex> guard(lock_);
  if (!tried_) {
    tried_ = true;
    device_ = CreateDevice();
    open_ = device_->Open();
    if (kind_ != MotionDeviceKind::kNone) {
      MotionStatus status = device_->status();
      XELOGI("Motion capture: {} {}{}{}", device_->name(),
             open_ ? "running" : "not running",
             status.reason.empty() ? "" : ": ", status.reason);
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
