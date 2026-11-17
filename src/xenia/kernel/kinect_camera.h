/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_KINECT_CAMERA_H_
#define XENIA_KERNEL_KINECT_CAMERA_H_

#include <cstdint>
#include <mutex>

#include "xenia/base/byte_order.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {

class KernelState;

// The Kinect camera driver behind PsCamDeviceRequest: what a title's NUI
// library talks to for depth and colour frames. Its frames come from the
// host's motion-capture system (hid/motion) through the virtual Kinect v1.
//
// Owned by KernelState, so that anything it runs in the background stops
// before the kernel goes away. Everything follows our own tracing of how a
// title calls the driver, in docs/kinect-pscam.md (section numbers in the
// code refer to it).
class KinectCamera {
 public:
  explicit KinectCamera(KernelState* kernel_state);
  ~KinectCamera();

  // One PsCamDeviceRequest: `block` is the guest address of the request
  // block, whose first word is the request code. Returns an NTSTATUS.
  X_STATUS Request(uint32_t block);

 private:
  // Whether a camera is there for the title. Opens the host device the
  // first time it is asked.
  bool CameraPresent();

  X_STATUS ReplyDepthTable(xe::be<uint32_t>* words);
  X_STATUS ReplyTiming(xe::be<uint32_t>* words);

  KernelState* kernel_state_;
  std::mutex lock_;
  bool streaming_ = false;  // between a start (27) and a stop (2)
};

}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_KINECT_CAMERA_H_
