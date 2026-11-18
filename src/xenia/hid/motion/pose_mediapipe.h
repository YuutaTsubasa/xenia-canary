/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_POSE_MEDIAPIPE_H_
#define XENIA_HID_MOTION_POSE_MEDIAPIPE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "xenia/hid/motion/motion_types.h"
#include "xenia/hid/motion/pose_landmarks.h"

// The parts of MediaPipe pose that do not need a model runtime: where the
// person detector's answer puts the person, what each network is fed, and
// how the pose network's answer becomes PoseLandmarks. Ported from Free
// Riders Recompiled (pose_mediapipe.cpp), written by its author YuutaTsubasa
// and brought here under this file's license.
//
// The models are MediaPipe's 2023 person detector and full-body pose
// landmarker as converted to ONNX by OpenCV Zoo (Apache-2.0):
// person_detection_mediapipe_2023mar.onnx and
// pose_estimation_mediapipe_2023mar.onnx.

namespace xe {
namespace hid {
namespace motion {

// A square crop in picture pixels: its centre, half its side, and the angle
// that turns crop coordinates back into the picture's (0 for an upright
// person).
struct PoseRegion {
  float x = 0.0f, y = 0.0f, radius = 0.0f, angle = 0.0f;
};

// The detector's 2254 anchors: 28x28 cells twice, 14x14 twice, 7x7 six
// times, each the centre of its cell (0..1 across the 224x224 input).
constexpr size_t kPersonAnchorCount = 2254;
std::array<float, 2> PersonAnchor(size_t index);

// The detector's best person (boxes: 2254 x 12 floats, scores: 2254 logits)
// as the crop for the pose network, or nothing when no detection reaches
// `threshold`. One player: only the best detection is kept.
std::optional<PoseRegion> DecodePerson(std::span<const float> boxes,
                                       std::span<const float> scores,
                                       uint32_t width, uint32_t height,
                                       float threshold);

// What a network is fed from `picture` (RGBA) seen through `region`: for the
// detector, 224x224 RGB planes (NCHW) in -1..1; for the pose network,
// 256x256 interleaved RGB (NHWC) in 0..1. Empty when the picture is
// incomplete or the region is not a usable crop.
std::vector<float> MediaPipeInput(const ColourImage& picture,
                                  const PoseRegion& region, bool detector);

// The pose network's answer (image: 39 x 5 floats, world: 39 x 3, the
// presence `confidence`) as the seventeen COCO points with 3D positions.
// False when anything is not finite, the presence or a torso point is below
// `threshold`; then `output` is left alone. `next` becomes the crop the
// network suggests for the next picture, if it is a sensible one.
bool DecodeMediaPipe(std::span<const float> image, std::span<const float> world,
                     float confidence, const PoseRegion& region,
                     float threshold, PoseLandmarks& output,
                     std::optional<PoseRegion>& next);

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_POSE_MEDIAPIPE_H_
