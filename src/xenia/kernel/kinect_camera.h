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

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include "xenia/base/byte_order.h"
#include "xenia/hid/motion/motion_types.h"
#include "xenia/hid/motion/virtual_tilt.h"
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

  // Whether the player chose a motion-capture device (motion_device is not
  // none). Without one, the XAM Kinect functions keep Xenia's earlier
  // behaviour (driven by the console settings' Kinect flag).
  bool DeviceChosen();
  // Whether a camera is there for the title. Opens the host device the
  // first time it is asked.
  bool CameraPresent();

  // The Kinect's tilt motor and accelerometer, as XAM's tilt functions see
  // them. Virtual for now: no backend drives a real motor yet.
  hid::motion::VirtualTilt& tilt() { return tilt_; }

 private:
  X_STATUS HandleRequest(uint32_t block, uint32_t request,
                         xe::be<uint32_t>* words);
  X_STATUS ReplyDepthTable(xe::be<uint32_t>* words);
  X_STATUS ReplyTiming(xe::be<uint32_t>* words);
  X_STATUS ReplyCalibration(xe::be<uint32_t>* words);
  X_STATUS QueueFrameRead(uint32_t block, xe::be<uint32_t>* words);

  // The worker: completes queued frame reads with the newest frames, and
  // cancels them all once the camera stops. Guest callbacks run on it.
  void StartWorker();
  void WorkerMain();
  bool FillDepthRead(uint32_t block);
  // Every 10 s for the first two minutes of depth: a log line on what the
  // title was given, and with motion_save_depth_pictures the pictures.
  void ReportDepth(std::chrono::steady_clock::time_point now);
  // When the virtual motor's angle changed since the last frame: a log
  // line on how many frames the title got at the old angle, and with
  // motion_save_depth_pictures the first picture at the new one.
  void ReportTilt();
  // With motion_save_depth_pictures: `depth` as <name>.bmp in the
  // kinect_depth folder.
  void SaveDepthPicture(const hid::motion::DepthImage& depth,
                        const std::string& name);
  void CompleteRead(uint32_t block, X_STATUS status, uint32_t bytes);

  KernelState* kernel_state_;
  hid::motion::VirtualTilt tilt_;
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

  // For the log, so a tester's xenia.log shows how far a title got with the
  // camera: each request code is logged the first time it comes (under
  // lock_), and the worker logs its first frame and a stall once.
  std::array<bool, 64> request_logged_ = {};
  bool colour_read_logged_ = false;
  bool depth_frame_logged_ = false;
  bool depth_failure_logged_ = false;
  bool stall_logged_ = false;
  bool cancel_logged_ = false;
  std::chrono::steady_clock::time_point last_depth_frame_ = {};

  // Worker-only, for ReportDepth: the picture the title got last (the
  // device's own or reprojected_), when depth started and how many frames
  // went since the last report.
  hid::motion::DepthImage reprojected_;
  const hid::motion::DepthImage* title_depth_ = nullptr;
  std::chrono::steady_clock::time_point first_depth_at_ = {};
  uint32_t depth_reports_ = 0;
  uint32_t frames_since_report_ = 0;
  // Worker-only, for ReportTilt: the motor angle the last picture was made
  // at, the one before it, and how many frames went at that one.
  int32_t filled_angle_ = 0;
  int32_t shown_angle_ = 0;
  uint32_t frames_at_angle_ = 0;
  bool saving_logged_ = false;
};

}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_KINECT_CAMERA_H_
