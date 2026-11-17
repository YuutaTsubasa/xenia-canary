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

#include "xenia/base/logging.h"
#include "xenia/emulator.h"
#include "xenia/hid/input_system.h"
#include "xenia/hid/motion/motion_capture.h"
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
};

// The device state code 0 reports at block+4 (§3).
enum PsCamState : uint32_t {
  kStateNoDevice = 0,
  kStateStreaming = 2,
  kStateIdle = 4,  // present and ready to start
};

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
      words[1] = CameraPresent() ? kStateIdle : kStateNoDevice;
      return X_STATUS_SUCCESS;
    default:
      if (!CameraPresent()) {
        return kStatusDeviceNotConnected;
      }
      XELOGW("PsCamDeviceRequest: request code {} not handled yet", request);
      return X_STATUS_NOT_IMPLEMENTED;
  }
}
DECLARE_XBOXKRNL_EXPORT1(PsCamDeviceRequest, kNone, kSketchy);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(PsCam);
