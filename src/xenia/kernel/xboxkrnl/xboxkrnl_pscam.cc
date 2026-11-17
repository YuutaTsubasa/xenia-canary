/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// PsCamDeviceRequest: the Kinect camera driver a title's NUI library talks
// to. The driver itself is KinectCamera (kernel/kinect_camera.h); see
// docs/kinect-pscam.md for how titles use it.

#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/kinect_camera.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xboxkrnl {

dword_result_t PsCamDeviceRequest_entry(lpvoid_t block) {
  return kernel_state()->kinect_camera()->Request(block.guest_address());
}
DECLARE_XBOXKRNL_EXPORT1(PsCamDeviceRequest, kNone, kSketchy);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(PsCam);
