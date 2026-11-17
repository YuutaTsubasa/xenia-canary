/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/kinect_v2_device.h"

#include <chrono>
#include <span>
#include <vector>

#include "xenia/base/logging.h"
#include "xenia/base/threading.h"
#include "xenia/hid/motion/kinect_v2_frames.h"
#include "xenia/hid/motion/kinect_v2_sdk.h"

// After platform_win.h, which sets up windows.h (NOMINMAX) for Xenia.
#include <wrl/client.h>

namespace xe {
namespace hid {
namespace motion {

namespace {

using Microsoft::WRL::ComPtr;

// A sensor takes a second or two after Open to report itself available and
// send its calibration; past this, Open gives up.
constexpr auto kStartTimeout = std::chrono::seconds(5);
// How often the capture thread looks for a new frame (the sensor sends one
// every 33 ms) and, while waiting for the sensor, for it to be ready.
constexpr auto kFramePollInterval = std::chrono::milliseconds(4);
constexpr auto kWaitPollInterval = std::chrono::milliseconds(50);

}  // namespace

KinectV2Device::KinectV2Device() = default;

KinectV2Device::~KinectV2Device() { Close(); }

bool KinectV2Device::Open() {
  if (thread_.joinable()) {
    return status().state == MotionDeviceState::kRunning;
  }
  HMODULE library = LoadLibraryW(L"Kinect20.dll");
  if (!library) {
    SetStatus(MotionDeviceState::kDriverMissing,
              "Kinect for Windows Runtime 2.0 is not installed "
              "(Kinect20.dll was not found)");
    return false;
  }
  library_ = library;
  SetStatus(MotionDeviceState::kNotFound, "Waiting for a Kinect v2");
  stopping_ = false;
  thread_ = std::thread([this]() { CaptureMain(); });

  bool running;
  {
    std::unique_lock<std::mutex> guard(lock_);
    status_changed_.wait_for(guard, kStartTimeout, [this]() {
      return status_.state == MotionDeviceState::kRunning ||
             status_.state == MotionDeviceState::kFailed;
    });
    running = status_.state == MotionDeviceState::kRunning;
  }
  if (!running) {
    Close();
    std::lock_guard<std::mutex> guard(lock_);
    if (status_.state == MotionDeviceState::kNotFound) {
      status_.reason =
          "No Kinect v2 became ready within 5 seconds (is it plugged in, "
          "with its power supply on?)";
    }
  }
  return running;
}

void KinectV2Device::Close() {
  if (thread_.joinable()) {
    stopping_ = true;
    thread_.join();
  }
  if (library_) {
    FreeLibrary(HMODULE(library_));
    library_ = nullptr;
  }
}

MotionCapabilities KinectV2Device::capabilities() const {
  MotionCapabilities capabilities;
  capabilities.depth = true;
  return capabilities;
}

MotionStatus KinectV2Device::status() const {
  std::lock_guard<std::mutex> guard(lock_);
  return status_;
}

bool KinectV2Device::GetNewestFrame(SensorFrame& frame) {
  std::lock_guard<std::mutex> guard(lock_);
  if (newest_.number <= frame.number) {
    return false;
  }
  frame = newest_;
  return true;
}

void KinectV2Device::SetStatus(MotionDeviceState state, std::string reason) {
  {
    std::lock_guard<std::mutex> guard(lock_);
    status_.state = state;
    status_.reason = std::move(reason);
  }
  status_changed_.notify_all();
}

void KinectV2Device::CaptureMain() {
  xe::threading::set_name("Kinect v2");
  // The runtime is a COM library; this thread is the only one that uses it.
  HRESULT com_result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);

  [this]() {
    auto get_sensor =
        reinterpret_cast<kinect20::GetDefaultKinectSensorFunction>(
            GetProcAddress(HMODULE(library_), "GetDefaultKinectSensor"));
    ComPtr<kinect20::IKinectSensor> sensor;
    if (!get_sensor || FAILED(get_sensor(&sensor)) || !sensor ||
        FAILED(sensor->Open())) {
      SetStatus(MotionDeviceState::kFailed,
                "The Kinect for Windows Runtime 2.0 would not start");
      return;
    }
    ComPtr<kinect20::IDepthFrameSource> source;
    ComPtr<kinect20::IDepthFrameReader> reader;
    ComPtr<kinect20::IFrameDescription> description;
    ComPtr<kinect20::ICoordinateMapper> mapper;
    int width = 0, height = 0;
    if (FAILED(sensor->get_DepthFrameSource(&source)) ||
        FAILED(source->OpenReader(&reader)) ||
        FAILED(source->get_FrameDescription(&description)) ||
        FAILED(description->get_Width(&width)) ||
        FAILED(description->get_Height(&height)) || width <= 0 || height <= 0 ||
        FAILED(sensor->get_CoordinateMapper(&mapper))) {
      SetStatus(MotionDeviceState::kFailed,
                "The Kinect v2's depth camera would not start");
      sensor->Close();
      return;
    }
    XELOGI("Motion capture: Kinect v2 runtime loaded, depth {}x{}", width,
           height);

    std::vector<uint16_t> depth(size_t(width) * height);
    std::vector<float> rays;
    uint64_t frame_number = 0;
    bool running = false;
    while (!stopping_) {
      BOOLEAN available = FALSE;
      if (FAILED(sensor->get_IsAvailable(&available)) || !available) {
        if (running) {
          running = false;
          SetStatus(MotionDeviceState::kNotFound,
                    "The Kinect v2 was unplugged or stopped");
        }
        std::this_thread::sleep_for(kWaitPollInterval);
        continue;
      }
      if (rays.empty()) {
        // Each pixel's direction, from the sensor's own calibration. It is
        // there once the sensor has started; until then, keep waiting.
        UINT32 count = 0;
        kinect20::PointF* entries = nullptr;
        if (SUCCEEDED(
                mapper->GetDepthFrameToCameraSpaceTable(&count, &entries)) &&
            entries) {
          std::span<const float> table(reinterpret_cast<const float*>(entries),
                                       size_t(count) * 2);
          if (KinectV2TableLooksValid(table, width, height)) {
            rays = KinectV2TableToRays(table);
          }
          CoTaskMemFree(entries);
        }
        if (rays.empty()) {
          std::this_thread::sleep_for(kWaitPollInterval);
          continue;
        }
      }
      if (!running) {
        running = true;
        SetStatus(MotionDeviceState::kRunning, "");
      }

      ComPtr<kinect20::IDepthFrame> frame;
      if (FAILED(reader->AcquireLatestFrame(&frame)) || !frame ||
          FAILED(
              frame->CopyFrameDataToArray(UINT(depth.size()), depth.data()))) {
        std::this_thread::sleep_for(kFramePollInterval);
        continue;
      }
      std::lock_guard<std::mutex> guard(lock_);
      newest_.number = ++frame_number;
      newest_.captured_at = std::chrono::steady_clock::now();
      newest_.depth.width = uint32_t(width);
      newest_.depth.height = uint32_t(height);
      newest_.depth.lens = {};  // the rays say where each pixel looks
      newest_.depth.millimetres = depth;
      if (newest_.depth.rays.size() != rays.size()) {
        newest_.depth.rays = rays;
      }
    }
    sensor->Close();
  }();

  if (SUCCEEDED(com_result)) {
    CoUninitialize();
  }
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
