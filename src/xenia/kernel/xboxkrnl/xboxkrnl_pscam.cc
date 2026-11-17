/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// PsCamDeviceRequest: the Kinect camera driver a title's NUI library talks
// to. Its frames come from the host's motion-capture system
// (hid/motion/motion_capture.h), through the virtual Kinect v1
// (hid/motion/virtual_kinect_v1.h).
//
// Everything here follows our own tracing of how a title calls the driver,
// written down in docs/kinect-pscam.md (section numbers below refer to it).
// In short: one argument, a request block whose first word is the request
// code; the result is an NTSTATUS.

#include <array>
#include <mutex>

#include "xenia/base/logging.h"
#include "xenia/emulator.h"
#include "xenia/hid/input_system.h"
#include "xenia/hid/motion/motion_capture.h"
#include "xenia/hid/motion/virtual_kinect_v1.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xboxkrnl {

namespace {

// What the title gives up on (docs/kinect-pscam.md §2, code 35's callback
// treats it as final). The console's exact value for "no camera" is not
// known; this is NT's STATUS_DEVICE_NOT_CONNECTED.
constexpr X_STATUS kStatusDeviceNotConnected = X_STATUS(0xC000009D);

// Request codes (§2). Only those handled so far are listed.
enum PsCamRequest : uint32_t {
  kRequestGetState = 0,
  kRequestStop = 2,
  kRequestStart = 27,
  kRequestDepthTable = 24,
  kRequestTiming = 25,
  kRequestUnknown33 = 33,
};

// The device state code 0 reports at block+4 (§3).
enum PsCamState : uint32_t {
  kStateNoDevice = 0,
  kStateStreaming = 2,
  kStateIdle = 4,  // present and ready to start
};

// What the driver remembers between requests.
struct CameraState {
  std::mutex lock;
  bool streaming = false;  // between a start (27) and a stop (2)
};
CameraState& camera() {
  static CameraState state;
  return state;
}

hid::motion::MotionCaptureSystem* motion_capture() {
  auto input_system = kernel_state()->emulator()->input_system();
  return input_system ? input_system->GetMotionCapture() : nullptr;
}

// Whether a camera is there for the title. Opens the host device the first
// time it is asked, since that is when a title starts looking for one.
bool CameraPresent() {
  auto motion = motion_capture();
  if (!motion || motion->kind() == hid::motion::MotionDeviceKind::kNone) {
    return false;
  }
  return motion->Acquire();
}

// Code 24 (§2, §11): +8 points at a u16 table of +12 entries, raw depth
// value to millimetres. The virtual Kinect v1 delivers millimetres, so the
// table maps each value to itself.
X_STATUS ReplyDepthTable(xe::be<uint32_t>* words) {
  const uint32_t table_address = words[2];
  const uint32_t entries = words[3];
  if (!table_address || entries > hid::motion::kV1DepthTableEntries) {
    return X_STATUS_INVALID_PARAMETER;
  }
  std::array<uint16_t, hid::motion::kV1DepthTableEntries> table;
  hid::motion::FillV1DepthTable(table);
  auto* out =
      kernel_memory()->TranslateVirtual<xe::be<uint16_t>*>(table_address);
  for (uint32_t i = 0; i < entries; ++i) {
    out[i] = table[i];
  }
  return X_STATUS_SUCCESS;
}

// Code 25 (§2, §5): 14 timing words at +4..+56, in microseconds. +4 and +24
// look like the depth and colour frame periods. +36 left 0 makes the title
// skip its frame-timing lock (code 15), which a virtual camera has no clock
// to lock to.
X_STATUS ReplyTiming(xe::be<uint32_t>* words) {
  constexpr uint32_t kFramePeriodMicroseconds = 33333;  // 30 frames a second
  for (uint32_t i = 1; i <= 14; ++i) {
    words[i] = 0;
  }
  words[1] = kFramePeriodMicroseconds;  // +4
  words[6] = kFramePeriodMicroseconds;  // +24
  return X_STATUS_SUCCESS;
}

}  // namespace

dword_result_t PsCamDeviceRequest_entry(lpvoid_t block) {
  if (!block) {
    return X_STATUS_INVALID_PARAMETER;
  }
  auto* words = block.as<xe::be<uint32_t>*>();
  const uint32_t request = words[0];

  switch (request) {
    case kRequestGetState:
      // No device: state 0, and the title carries on without Kinect, as it
      // does on a console with nothing plugged in.
      if (!CameraPresent()) {
        words[1] = kStateNoDevice;
      } else {
        std::lock_guard<std::mutex> guard(camera().lock);
        words[1] = camera().streaming ? kStateStreaming : kStateIdle;
      }
      return X_STATUS_SUCCESS;
    default:
      break;
  }

  if (!CameraPresent()) {
    return kStatusDeviceNotConnected;
  }
  // The title sends these through a wrapper that would wait for a callback
  // (+16/+20) if the result were STATUS_PENDING; finishing them here and
  // returning success is just as good for it (§0).
  switch (request) {
    case kRequestStart: {
      std::lock_guard<std::mutex> guard(camera().lock);
      camera().streaming = true;
      return X_STATUS_SUCCESS;
    }
    case kRequestStop: {
      std::lock_guard<std::mutex> guard(camera().lock);
      camera().streaming = false;
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
DECLARE_XBOXKRNL_EXPORT1(PsCamDeviceRequest, kNone, kSketchy);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(PsCam);
