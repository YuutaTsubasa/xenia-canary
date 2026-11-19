/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/webcam_device.h"

#include <chrono>
#include <cmath>
#include <numbers>

#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/base/platform.h"
#include "xenia/base/threading.h"
#include "xenia/hid/motion/body_placement.h"
#include "xenia/hid/motion/pose_estimator.h"
#include "xenia/hid/motion/pose_skeleton.h"
#include "xenia/hid/motion/pose_smoothing.h"
#include "xenia/hid/motion/synthetic_depth.h"
#include "xenia/hid/motion/webcam_capture.h"

#if XE_PLATFORM_WIN32
#include "xenia/base/platform_win.h"
#endif

DEFINE_string(webcam_device, "",
              "For motion_device = webcam: part of the webcam's name. Empty: "
              "the first webcam.",
              "HID");
DEFINE_bool(webcam_mirrored, false,
            "For motion_device = webcam: the webcam sends a mirrored picture "
            "(as phone and video-call cameras often do), so left and right "
            "are swapped back.",
            "HID");
DEFINE_double(webcam_fov, 60.0,
              "For motion_device = webcam: the webcam's horizontal field of "
              "view in degrees (most are 55 to 80), to tell how far the "
              "player moves sideways.",
              "HID");

namespace xe {
namespace hid {
namespace motion {

namespace {

using Clock = std::chrono::steady_clock;

// The picture size asked of the webcam: plenty for the pose model.
constexpr uint32_t kPictureWidth = 640, kPictureHeight = 480;
// Model loading and a camera's start can take a few seconds.
constexpr auto kStartTimeout = std::chrono::seconds(10);
// A lost player's body is kept this long, so one missed picture does not
// make them blink out of the room.
constexpr auto kKeepLostBody = std::chrono::milliseconds(300);

}  // namespace

WebcamDevice::WebcamDevice() = default;

WebcamDevice::~WebcamDevice() { Close(); }

bool WebcamDevice::Open() {
  if (thread_.joinable()) {
    return status().state == MotionDeviceState::kRunning;
  }
  SetStatus(MotionDeviceState::kNotFound, "Starting the webcam");
  stopping_ = false;
  {
    std::lock_guard<std::mutex> guard(lock_);
    capture_ended_ = false;
  }
  thread_ = std::thread([this]() { CaptureMain(); });

  bool running;
  {
    std::unique_lock<std::mutex> guard(lock_);
    status_changed_.wait_for(guard, kStartTimeout, [this]() {
      return status_.state == MotionDeviceState::kRunning || capture_ended_;
    });
    running = status_.state == MotionDeviceState::kRunning;
  }
  if (!running) {
    Close();
    std::lock_guard<std::mutex> guard(lock_);
    if (!capture_ended_) {
      status_.reason = "The webcam sent no picture within 10 seconds";
    }
  }
  return running;
}

void WebcamDevice::Close() {
  if (thread_.joinable()) {
    stopping_ = true;
    thread_.join();
  }
}

MotionCapabilities WebcamDevice::capabilities() const {
  MotionCapabilities capabilities;
  capabilities.depth = true;  // drawn from the skeleton
  capabilities.colour = true;
  capabilities.bodies = true;
  capabilities.max_bodies = 1;
  return capabilities;
}

MotionStatus WebcamDevice::status() const {
  std::lock_guard<std::mutex> guard(lock_);
  return status_;
}

bool WebcamDevice::GetNewestFrame(SensorFrame& frame) {
  std::lock_guard<std::mutex> guard(lock_);
  if (newest_.number <= frame.number) {
    return false;
  }
  frame = newest_;
  return true;
}

void WebcamDevice::SetStatus(MotionDeviceState state, std::string reason) {
  {
    std::lock_guard<std::mutex> guard(lock_);
    status_.state = state;
    status_.reason = std::move(reason);
  }
  status_changed_.notify_all();
}

void WebcamDevice::CaptureMain() {
  xe::threading::set_name("Webcam");
#if XE_PLATFORM_WIN32
  // Media Foundation is used from this thread only.
  HRESULT com_result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
#endif
  Capture();
#if XE_PLATFORM_WIN32
  if (SUCCEEDED(com_result)) {
    CoUninitialize();
  }
#endif
  {
    std::lock_guard<std::mutex> guard(lock_);
    capture_ended_ = true;
  }
  status_changed_.notify_all();
}

void WebcamDevice::Capture() {
  std::string reason;
  auto estimator = PoseEstimator::Open(PoseEstimator::DefaultFolder(), reason);
  if (!estimator) {
    SetStatus(MotionDeviceState::kDriverMissing, reason);
    return;
  }
  auto webcam = WebcamCapture::Open(cvars::webcam_device, kPictureWidth,
                                    kPictureHeight, reason);
  if (!webcam) {
    SetStatus(MotionDeviceState::kNotFound, reason);
    return;
  }

  const bool mirrored = cvars::webcam_mirrored;
  const float fov = float(cvars::webcam_fov);
  SyntheticRoom room;
  PoseSmoothing smoothing;
  BodyPlacement placement(fov, room.sensor_height);
  PoseLandmarks landmarks{};
  ColourImage picture;
  Body body;
  body.tracking_id = 1;
  bool have_body = false;
  Clock::time_point last_found = {};
  uint64_t frame_number = 0;

  while (!stopping_) {
    if (!webcam->Read(picture)) {
      // No picture (a gap, or the camera stopped): try again shortly.
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    Clock::time_point now = Clock::now();
    if (estimator->Estimate(picture, landmarks)) {
      double interval =
          have_body ? std::chrono::duration<double>(now - last_found).count()
                    : 0.0;
      smoothing.Smooth(landmarks, interval);
      Body found = body;
      if (PoseToBody(landmarks, picture.width, picture.height, mirrored,
                     found)) {
        float hip_x = 0.5f * (landmarks[pose_point::kHipLeft].x +
                              landmarks[pose_point::kHipRight].x);
        float hip_y = 0.5f * (landmarks[pose_point::kHipLeft].y +
                              landmarks[pose_point::kHipRight].y);
        if (mirrored) {
          hip_x = float(picture.width) - hip_x;
        }
        placement.Place(found, hip_x, hip_y, picture.width, picture.height,
                        interval);
        body = found;
        have_body = true;
        last_found = now;
      }
    } else if (have_body && now - last_found > kKeepLostBody) {
      have_body = false;
      smoothing.Forget();
      placement.Forget();
    }

    room.tilt_degrees = view_tilt_;
    DepthImage depth = DrawSyntheticDepth(have_body ? &body : nullptr, room);
    {
      std::lock_guard<std::mutex> guard(lock_);
      newest_.number = ++frame_number;
      newest_.captured_at = now;
      newest_.depth = std::move(depth);
      newest_.colour.width = picture.width;
      newest_.colour.height = picture.height;
      // An ideal pinhole as wide as the webcam's field of view.
      float focal = 0.5f * float(picture.width) /
                    std::tan(fov * std::numbers::pi_v<float> / 360.0f);
      newest_.colour.lens = {};
      newest_.colour.lens.focal_x = newest_.colour.lens.focal_y = focal;
      newest_.colour.lens.centre_x = 0.5f * float(picture.width);
      newest_.colour.lens.centre_y = 0.5f * float(picture.height);
      newest_.colour.rgba = picture.rgba;
      newest_.bodies.clear();
      if (have_body) {
        newest_.bodies.push_back(body);
      }
      // The drawn floor, in skeleton space: y = -sensor_height.
      newest_.floor_plane = {0.0f, 1.0f, 0.0f, room.sensor_height};
    }
    if (frame_number == 1) {
      SetStatus(MotionDeviceState::kRunning, "");
    }
  }
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
