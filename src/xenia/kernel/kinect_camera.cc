/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/kinect_camera.h"

#include <array>
#include <chrono>
#include <cstring>
#include <span>

#include "xenia/base/logging.h"
#include "xenia/cpu/processor.h"
#include "xenia/emulator.h"
#include "xenia/hid/input_system.h"
#include "xenia/hid/motion/motion_capture.h"
#include "xenia/hid/motion/virtual_kinect_v1.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/xthread.h"
#include "xenia/memory.h"

namespace xe {
namespace kernel {

namespace {

// What the title gives up on (§2, code 35's callback treats it as final).
// The console's exact value for "no camera" is not known; this is NT's
// STATUS_DEVICE_NOT_CONNECTED.
constexpr X_STATUS kStatusDeviceNotConnected = X_STATUS(0xC000009D);
// What queued frame reads finish with once the camera stops: NT's
// STATUS_CANCELLED. The title only needs a failure here, and every read
// finished, or it waits for them forever when it shuts down (section 7).
constexpr X_STATUS kStatusCancelled = X_STATUS(0xC0000120);
// A wrong buffer length in a frame read: NT's STATUS_INFO_LENGTH_MISMATCH.
constexpr X_STATUS kStatusLengthMismatch = X_STATUS(0xC0000004);

// Request codes (§2). Only those handled so far are listed.
enum PsCamRequest : uint32_t {
  kRequestGetState = 0,
  kRequestStop = 2,
  kRequestReadFrame = 5,
  kRequestCalibrationOld = 11,
  kRequestVersion = 13,
  kRequestDepthTable = 24,
  kRequestTiming = 25,
  kRequestStart = 27,
  kRequestUnknown33 = 33,
  kRequestColourSettings = 35,
};

// The device state code 0 reports at block+4 (§3).
enum PsCamState : uint32_t {
  kStateNoDevice = 0,
  kStateStreaming = 2,
  kStateIdle = 4,  // present and ready to start
};

// A frame read's block (section 4): +20 callback, +24 stream, +28 buffer,
// +32 buffer length, +40 where the frame number goes. As word indices:
constexpr size_t kReadCallback = 5;
constexpr size_t kReadStream = 6;
constexpr size_t kReadBuffer = 7;
constexpr size_t kReadLength = 8;
constexpr size_t kReadInfo = 10;
enum PsCamStream : uint32_t {
  kStreamColour = 2,
  kStreamDepth = 4,
};

// How often the worker looks for a new device frame when nothing wakes it.
constexpr auto kWorkerPollInterval = std::chrono::milliseconds(5);

hid::motion::MotionCaptureSystem* MotionSystem(KernelState* kernel_state) {
  auto input_system = kernel_state->emulator()->input_system();
  return input_system ? input_system->GetMotionCapture() : nullptr;
}

}  // namespace

KinectCamera::KinectCamera(KernelState* kernel_state)
    : kernel_state_(kernel_state) {}

KinectCamera::~KinectCamera() {
  // Stops the worker without running guest code: the title is gone.
  if (worker_) {
    {
      std::lock_guard<std::mutex> guard(lock_);
      worker_running_ = false;
    }
    wake_.notify_all();
    worker_->Wait(0, 0, 0, nullptr);
    worker_.reset();
  }
}

bool KinectCamera::CameraPresent() {
  auto motion = MotionSystem(kernel_state_);
  if (!motion || motion->kind() == hid::motion::MotionDeviceKind::kNone) {
    return false;
  }
  return motion->Acquire();
}

X_STATUS KinectCamera::Request(uint32_t block) {
  if (!block) {
    return X_STATUS_INVALID_PARAMETER;
  }
  auto* words =
      kernel_state_->memory()->TranslateVirtual<xe::be<uint32_t>*>(block);
  const uint32_t request = words[0];

  if (request == kRequestGetState) {
    // No device: state 0, and the title carries on without Kinect, as it
    // does on a console with nothing plugged in.
    if (!CameraPresent()) {
      words[1] = kStateNoDevice;
    } else {
      std::lock_guard<std::mutex> guard(lock_);
      words[1] = streaming_ ? kStateStreaming : kStateIdle;
    }
    return X_STATUS_SUCCESS;
  }

  if (!CameraPresent()) {
    return kStatusDeviceNotConnected;
  }
  // The title sends these through a wrapper that would wait for a callback
  // (+16/+20) if the result were STATUS_PENDING; finishing them here and
  // returning success is just as good for it (§0).
  switch (request) {
    case kRequestStart: {
      std::lock_guard<std::mutex> guard(lock_);
      streaming_ = true;
      return X_STATUS_SUCCESS;
    }
    case kRequestStop: {
      // Queued reads are cancelled by the worker, where guest callbacks
      // can run.
      {
        std::lock_guard<std::mutex> guard(lock_);
        streaming_ = false;
      }
      wake_.notify_all();
      return X_STATUS_SUCCESS;
    }
    case kRequestReadFrame:
      return QueueFrameRead(block, words);
    case kRequestVersion: {
      // Version 4.0 (section 12): below 5.1 the title asks for the older
      // calibration format, code 11, which has a safe all-zero answer.
      auto* bytes = reinterpret_cast<uint8_t*>(words);
      bytes[4] = 4;  // major
      bytes[5] = 0;  // minor
      return X_STATUS_SUCCESS;
    }
    case kRequestCalibrationOld:
      return ReplyCalibration(words);
    case kRequestColourSettings:
      // Colour-camera settings (exposure, white balance). The virtual
      // camera has none to set; this status makes the title stop asking
      // instead of retrying (section 2).
      return kStatusDeviceNotConnected;
    case kRequestUnknown33:
      // Its meaning is unknown and the title ignores the result.
      return X_STATUS_SUCCESS;
    case kRequestDepthTable:
      return ReplyDepthTable(words);
    case kRequestTiming:
      return ReplyTiming(words);
    default:
      XELOGW("PsCamDeviceRequest: request code {} not handled yet", request);
      return X_STATUS_NOT_IMPLEMENTED;
  }
}

// Code 24 (§2, §11): +8 points at a u16 table of +12 entries, raw depth
// value to millimetres. The virtual Kinect v1 delivers millimetres, so the
// table maps each value to itself.
X_STATUS KinectCamera::ReplyDepthTable(xe::be<uint32_t>* words) {
  const uint32_t table_address = words[2];
  const uint32_t entries = words[3];
  if (!table_address || entries > hid::motion::kV1DepthTableEntries) {
    return X_STATUS_INVALID_PARAMETER;
  }
  std::array<uint16_t, hid::motion::kV1DepthTableEntries> table;
  hid::motion::FillV1DepthTable(table);
  auto* out = kernel_state_->memory()->TranslateVirtual<xe::be<uint16_t>*>(
      table_address);
  for (uint32_t i = 0; i < entries; ++i) {
    out[i] = table[i];
  }
  return X_STATUS_SUCCESS;
}

// Code 5 (section 4): one frame read. The title keeps four depth and four
// colour reads queued and sends each again, unchanged, after using its
// frame. A read is finished later, on the worker, through the callback at
// +20; the request itself returns STATUS_PENDING.
X_STATUS KinectCamera::QueueFrameRead(uint32_t block, xe::be<uint32_t>* words) {
  const uint32_t stream = words[kReadStream];
  const uint32_t length = words[kReadLength];
  if (!words[kReadCallback] || !words[kReadBuffer]) {
    return X_STATUS_INVALID_PARAMETER;
  }
  if (stream != kStreamDepth && stream != kStreamColour) {
    return X_STATUS_INVALID_PARAMETER;
  }
  if (stream == kStreamDepth && length != hid::motion::kV1DepthFrameBytes) {
    return kStatusLengthMismatch;
  }
  {
    std::lock_guard<std::mutex> guard(lock_);
    if (stream == kStreamDepth) {
      depth_reads_.push_back(block);
    } else {
      // Colour reads wait until the colour format is known (Bayer or YUV,
      // docs/kinect-pscam.md section 11); they are cancelled when the
      // camera stops. The title tracks from depth alone.
      colour_reads_.push_back(block);
    }
  }
  StartWorker();
  wake_.notify_all();
  return X_STATUS_PENDING;
}

void KinectCamera::StartWorker() {
  std::lock_guard<std::mutex> guard(lock_);
  if (worker_) {
    return;
  }
  worker_running_ = true;
  worker_ = object_ref<XHostThread>(new XHostThread(
      kernel_state_, 128 * 1024, 0,
      [this]() {
        WorkerMain();
        return 0;
      },
      kernel_state_->GetSystemProcess()));
  // It runs guest callbacks, so the debugger must be able to suspend it.
  worker_->set_can_debugger_suspend(true);
  worker_->set_name("Kinect Camera");
  worker_->Create();
}

void KinectCamera::WorkerMain() {
  while (true) {
    std::deque<uint32_t> cancelled;
    bool want_depth = false;
    {
      std::unique_lock<std::mutex> guard(lock_);
      wake_.wait_for(guard, kWorkerPollInterval);
      if (!worker_running_) {
        return;
      }
      if (!streaming_) {
        cancelled.swap(depth_reads_);
        cancelled.insert(cancelled.end(), colour_reads_.begin(),
                         colour_reads_.end());
        colour_reads_.clear();
      } else {
        want_depth = !depth_reads_.empty();
      }
    }
    for (uint32_t block : cancelled) {
      CompleteRead(block, kStatusCancelled, 0);
    }
    if (!want_depth) {
      continue;
    }
    auto motion = MotionSystem(kernel_state_);
    if (!motion || !motion->GetNewestFrame(frame_) ||
        frame_.number == delivered_frame_ || frame_.depth.empty()) {
      continue;  // no new depth yet
    }
    delivered_frame_ = frame_.number;
    uint32_t block;
    {
      std::lock_guard<std::mutex> guard(lock_);
      if (depth_reads_.empty()) {
        continue;
      }
      block = depth_reads_.front();
      depth_reads_.pop_front();
    }
    if (FillDepthRead(block)) {
      CompleteRead(block, X_STATUS_SUCCESS,
                   uint32_t(hid::motion::kV1DepthFrameBytes));
    } else {
      CompleteRead(block, kStatusLengthMismatch, 0);
    }
  }
}

// Writes the newest depth frame into a read's buffer, as the virtual
// Kinect v1 sees it, and its frame number where +40 points.
bool KinectCamera::FillDepthRead(uint32_t block) {
  auto memory = kernel_state_->memory();
  auto* words = memory->TranslateVirtual<xe::be<uint32_t>*>(block);
  hid::motion::DepthImage reprojected;
  const hid::motion::DepthImage* depth = &frame_.depth;
  if (depth->width != hid::motion::kV1DepthWidth ||
      depth->height != hid::motion::kV1DepthHeight) {
    // Another camera's picture: seen again through the v1's lens, from the
    // same place.
    reprojected = hid::motion::ReprojectToV1Depth(frame_.depth);
    depth = &reprojected;
  }
  auto* buffer = memory->TranslateVirtual<uint8_t*>(words[kReadBuffer]);
  if (!hid::motion::EncodeV1DepthFrame(
          *depth, std::span<uint8_t>(buffer, words[kReadLength]))) {
    return false;
  }
  if (uint32_t info = words[kReadInfo]) {
    *memory->TranslateVirtual<xe::be<uint32_t>*>(info) = ++frame_number_;
  }
  return true;
}

// Finishes a read the way the title expects (section 0): callback(block,
// status, bytes transferred), on this worker.
void KinectCamera::CompleteRead(uint32_t block, X_STATUS status,
                                uint32_t bytes) {
  auto* words =
      kernel_state_->memory()->TranslateVirtual<xe::be<uint32_t>*>(block);
  const uint32_t callback = words[kReadCallback];
  uint64_t args[] = {block, status, bytes};
  kernel_state_->processor()->Execute(worker_->thread_state(), callback, args,
                                      xe::countof(args));
}

// Code 11 (section 12): the depth-to-colour registration, used only to
// line the colour picture up with depth (the player's video), never by the
// skeleton tracker. The reply runs to +4215 and the title does not clear
// it, so every byte is written. All zero means "colour pixel = twice the
// depth pixel, no parallax", which has no divisions and no NaN; a failure
// instead would stop the title's start sequence (section 12).
X_STATUS KinectCamera::ReplyCalibration(xe::be<uint32_t>* words) {
  constexpr size_t kReplyEnd = 4216;  // bytes, from the block's start
  auto* bytes = reinterpret_cast<uint8_t*>(words);
  std::memset(bytes + 4, 0, kReplyEnd - 4);
  return X_STATUS_SUCCESS;
}

// Code 25 (§2, §5): 14 timing words at +4..+56, in microseconds. +4 and +24
// look like the depth and colour frame periods. +36 left 0 makes the title
// skip its frame-timing lock (code 15), which a virtual camera has no clock
// to lock to.
X_STATUS KinectCamera::ReplyTiming(xe::be<uint32_t>* words) {
  constexpr uint32_t kFramePeriodMicroseconds = 33333;  // 30 frames a second
  for (uint32_t i = 1; i <= 14; ++i) {
    words[i] = 0;
  }
  words[1] = kFramePeriodMicroseconds;  // +4
  words[6] = kFramePeriodMicroseconds;  // +24
  return X_STATUS_SUCCESS;
}

}  // namespace kernel
}  // namespace xe
