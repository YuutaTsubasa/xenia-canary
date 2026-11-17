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

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>

#include "xenia/base/byte_order.h"
#include "xenia/hid/motion/motion_types.h"
#include "xenia/kernel/xobject.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {

class KernelState;
class XHostThread;

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
  X_STATUS ReplyCalibration(xe::be<uint32_t>* words);
  X_STATUS QueueFrameRead(uint32_t block, xe::be<uint32_t>* words);

  // The worker: completes queued frame reads with the newest frames, and
  // cancels them all once the camera stops. Guest callbacks run on it.
  void StartWorker();
  void WorkerMain();
  bool FillDepthRead(uint32_t block);
  void CompleteRead(uint32_t block, X_STATUS status, uint32_t bytes);

  KernelState* kernel_state_;
  std::mutex lock_;
  std::condition_variable wake_;
  bool streaming_ = false;  // between a start (27) and a stop (2)
  // Frame reads (code 5) waiting for a frame: request block addresses.
  std::deque<uint32_t> depth_reads_;
  std::deque<uint32_t> colour_reads_;

  object_ref<XHostThread> worker_;
  std::atomic<bool> worker_running_ = false;

  // Worker-only: the newest device frame and the frame counter the title
  // sees.
  hid::motion::SensorFrame frame_;
  uint64_t delivered_frame_ = 0;
  uint32_t frame_number_ = 0;
};

}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_KINECT_CAMERA_H_
