/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_WEBCAM_DEVICE_H_
#define XENIA_HID_MOTION_WEBCAM_DEVICE_H_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "xenia/hid/motion/motion_device.h"

namespace xe {
namespace hid {
namespace motion {

// A plain webcam with a pose model standing in for a Kinect. Each picture:
// MediaPipe finds the player (pose_estimator), the points are steadied
// (pose_smoothing), made a skeleton (pose_skeleton) and placed in the room
// (body_placement), and the skeleton is drawn as a depth picture
// (synthetic_depth) for the title's own tracker. The frame also carries the
// picture itself and the skeleton.
//
// Experimental: whether a title's tracker accepts the drawn body is only
// known once tried. Settings: webcam_device, webcam_mirrored, webcam_fov,
// motion_pose_folder.
class WebcamDevice final : public MotionDevice {
 public:
  WebcamDevice();
  ~WebcamDevice() override;

  const char* name() const override { return "webcam"; }
  // Loads the pose model, opens the webcam, and waits until the first
  // picture has come (or a few seconds pass).
  bool Open() override;
  void Close() override;

  MotionCapabilities capabilities() const override;
  MotionStatus status() const override;
  bool GetNewestFrame(SensorFrame& frame) override;

 private:
  void CaptureMain();
  void Capture();
  void SetStatus(MotionDeviceState state, std::string reason);

  std::thread thread_;
  std::atomic<bool> stopping_ = false;

  mutable std::mutex lock_;
  std::condition_variable status_changed_;
  MotionStatus status_;
  // The capture thread has finished (it gave up, or was stopped).
  bool capture_ended_ = false;
  SensorFrame newest_;
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_WEBCAM_DEVICE_H_
