/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_POSE_ESTIMATOR_H_
#define XENIA_HID_MOTION_POSE_ESTIMATOR_H_

#include <filesystem>
#include <memory>
#include <string>

#include "xenia/base/platform.h"
#include "xenia/hid/motion/motion_types.h"
#include "xenia/hid/motion/pose_landmarks.h"

// Finds a body in a webcam picture with MediaPipe pose, run by ONNX Runtime.
// Ported from Free Riders Recompiled (pose_estimator_onnx.cpp,
// pose_mediapipe_onnx.cpp), written by its author YuutaTsubasa and brought
// here under this file's license.

namespace xe {
namespace hid {
namespace motion {

// The files the estimator needs, all in one folder (the motion_pose_folder
// cvar; by default "pose" beside the Xenia executable). None are shipped
// with Xenia; docs/motion-capture.md says where to get them.
constexpr char kPoseDetectorModel[] = "person_detection_mediapipe_2023mar.onnx";
constexpr char kPoseLandmarkModel[] = "pose_estimation_mediapipe_2023mar.onnx";
#if XE_PLATFORM_WIN32
constexpr char kOnnxRuntimeLibrary[] = "onnxruntime.dll";
#else
constexpr char kOnnxRuntimeLibrary[] = "libonnxruntime.so";
#endif

// One model pair, one session each, used from one thread: the estimate runs
// where the camera pictures are read, never on the emulator's threads.
class PoseEstimator {
 public:
  virtual ~PoseEstimator();

  // The body in `picture` (RGBA). False when the model found no one it is
  // confident about; `landmarks` is then left alone.
  virtual bool Estimate(const ColourImage& picture,
                        PoseLandmarks& landmarks) = 0;

  // The folder the motion_pose_folder cvar names.
  static std::filesystem::path DefaultFolder();

  // Loads ONNX Runtime (from `folder` first, then wherever the system finds
  // it) and both models from `folder`. Null, with `reason` in words a
  // player can act on, when any of them is missing or unusable.
  static std::unique_ptr<PoseEstimator> Open(
      const std::filesystem::path& folder, std::string& reason);
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_POSE_ESTIMATOR_H_
