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

#include "xenia/base/logging.h"
#include "xenia/emulator.h"
#include "xenia/hid/input_system.h"
#include "xenia/hid/motion/motion_capture.h"
#include "xenia/hid/motion/virtual_kinect_v1.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/memory.h"

namespace xe {
namespace kernel {

namespace {

// What the title gives up on (§2, code 35's callback treats it as final).
// The console's exact value for "no camera" is not known; this is NT's
// STATUS_DEVICE_NOT_CONNECTED.
constexpr X_STATUS kStatusDeviceNotConnected = X_STATUS(0xC000009D);

// Request codes (§2). Only those handled so far are listed.
enum PsCamRequest : uint32_t {
  kRequestGetState = 0,
  kRequestStop = 2,
  kRequestDepthTable = 24,
  kRequestTiming = 25,
  kRequestStart = 27,
  kRequestUnknown33 = 33,
};

// The device state code 0 reports at block+4 (§3).
enum PsCamState : uint32_t {
  kStateNoDevice = 0,
  kStateStreaming = 2,
  kStateIdle = 4,  // present and ready to start
};

}  // namespace

KinectCamera::KinectCamera(KernelState* kernel_state)
    : kernel_state_(kernel_state) {}

KinectCamera::~KinectCamera() = default;

bool KinectCamera::CameraPresent() {
  auto input_system = kernel_state_->emulator()->input_system();
  auto motion = input_system ? input_system->GetMotionCapture() : nullptr;
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
      std::lock_guard<std::mutex> guard(lock_);
      streaming_ = false;
      return X_STATUS_SUCCESS;
    }
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
