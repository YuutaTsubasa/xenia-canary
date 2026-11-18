/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/pose_mediapipe.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace xe {
namespace hid {
namespace motion {

namespace {

constexpr uint32_t kDetectorSize = 224;
constexpr uint32_t kPoseSize = 256;
// The pose network's 33 BlazePose points plus two helper points (33: hip
// centre, 34: a point setting the full body's size and turn).
constexpr size_t kPoseNetworkPoints = 39;

float Probability(float logit) {
  return 1.0f / (1.0f + std::exp(-std::clamp(logit, -80.0f, 80.0f)));
}

bool Usable(const PoseRegion& region) {
  return std::isfinite(region.x) && std::isfinite(region.y) &&
         std::isfinite(region.radius) && std::isfinite(region.angle) &&
         std::abs(region.x) < 100000.0f && std::abs(region.y) < 100000.0f &&
         region.radius >= 1.0f && region.radius < 100000.0f;
}

// The crop around a centre point, sized and turned by a second point.
// MediaPipe's PoseDetectionToRoi and PoseLandmarksToRoi both add the 1.25
// margin the models were trained with; without it, a tracked crop shrinks
// every picture.
PoseRegion RegionFromPoints(float x, float y, float end_x, float end_y) {
  return {x, y, 1.25f * std::hypot(end_x - x, end_y - y),
          std::remainder(std::numbers::pi_v<float> / 2.0f +
                             std::atan2(end_y - y, end_x - x),
                         2.0f * std::numbers::pi_v<float>)};
}

// A point in crop coordinates (pixels from the crop's centre) in the
// picture.
std::array<float, 2> PicturePoint(const PoseRegion& region, float x, float y) {
  float c = std::cos(region.angle), s = std::sin(region.angle);
  return {region.x + c * x - s * y, region.y + s * x + c * y};
}

}  // namespace

std::array<float, 2> PersonAnchor(size_t index) {
  for (auto [grid, copies] :
       {std::pair{28, 2}, std::pair{14, 2}, std::pair{7, 6}}) {
    size_t count = size_t(grid) * grid * copies;
    if (index < count) {
      size_t cell = index / copies;
      return {(float(cell % grid) + 0.5f) / grid,
              (float(cell / grid) + 0.5f) / grid};
    }
    index -= count;
  }
  return {};
}

std::optional<PoseRegion> DecodePerson(std::span<const float> boxes,
                                       std::span<const float> scores,
                                       uint32_t width, uint32_t height,
                                       float threshold) {
  if (boxes.size() != kPersonAnchorCount * 12 ||
      scores.size() != kPersonAnchorCount || !width || !height) {
    return std::nullopt;
  }
  // The picture was letterboxed into a square of its longer side.
  float scale = float(std::max(width, height));
  float best = threshold;
  std::optional<PoseRegion> result;
  for (size_t i = 0; i < kPersonAnchorCount; ++i) {
    float score = Probability(scores[i]);
    if (!std::isfinite(scores[i]) || score < best) {
      continue;
    }
    auto anchor = PersonAnchor(i);
    auto point = [&](size_t at) {
      return std::array<float, 2>{
          (boxes[i * 12 + at] / kDetectorSize + anchor[0]) * scale -
              (scale - width) / 2.0f,
          (boxes[i * 12 + at + 1] / kDetectorSize + anchor[1]) * scale -
              (scale - height) / 2.0f};
    };
    // Keypoints 2 and 3 of the box (floats 4..7): the hip centre and the
    // point that sets the whole body's size and turn.
    auto hip = point(4), full = point(6);
    PoseRegion crop = RegionFromPoints(hip[0], hip[1], full[0], full[1]);
    if (!Usable(crop) || crop.radius > scale * 2.0f) {
      continue;
    }
    best = score;
    result = crop;
  }
  return result;
}

std::vector<float> MediaPipeInput(const ColourImage& picture,
                                  const PoseRegion& region, bool detector) {
  if (!picture.width || !picture.height ||
      picture.rgba.size() / 4 < size_t(picture.width) * picture.height ||
      !Usable(region)) {
    return {};
  }
  const uint32_t size = detector ? kDetectorSize : kPoseSize;
  std::vector<float> out(size_t(size) * size * 3);
  float step = 2.0f * region.radius / size;
  float c = std::cos(region.angle), s = std::sin(region.angle);
  for (uint32_t y = 0; y < size; ++y) {
    for (uint32_t x = 0; x < size; ++x) {
      float dx = (float(x) + 0.5f) * step - region.radius;
      float dy = (float(y) + 0.5f) * step - region.radius;
      float sx = region.x + c * dx - s * dy - 0.5f;
      float sy = region.y + s * dx + c * dy - 0.5f;
      int ix = int(std::floor(sx)), iy = int(std::floor(sy));
      float fx = sx - ix, fy = sy - iy;
      for (uint32_t channel = 0; channel < 3; ++channel) {
        auto sample = [&](int xx, int yy) {
          // Outside the picture the detector sees 0 after its -1..1
          // scaling; the pose network sees black.
          if (xx < 0 || yy < 0 || xx >= int(picture.width) ||
              yy >= int(picture.height)) {
            return 0.0f;
          }
          float value =
              picture.rgba[(size_t(yy) * picture.width + xx) * 4 + channel] /
              255.0f;
          return detector ? value * 2.0f - 1.0f : value;
        };
        float upper = std::lerp(sample(ix, iy), sample(ix + 1, iy), fx);
        float lower = std::lerp(sample(ix, iy + 1), sample(ix + 1, iy + 1), fx);
        size_t at = detector ? size_t(channel) * size * size + y * size + x
                             : (size_t(y) * size + x) * 3 + channel;
        out[at] = std::lerp(upper, lower, fy);
      }
    }
  }
  return out;
}

bool DecodeMediaPipe(std::span<const float> image, std::span<const float> world,
                     float confidence, const PoseRegion& region,
                     float threshold, PoseLandmarks& output,
                     std::optional<PoseRegion>& next) {
  next.reset();
  if (image.size() != kPoseNetworkPoints * 5 ||
      world.size() != kPoseNetworkPoints * 3 || !std::isfinite(confidence) ||
      confidence < threshold || !Usable(region)) {
    return false;
  }
  // The BlazePose point behind each COCO point.
  constexpr uint32_t kBlazePose[pose_point::kCount] = {
      0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28};
  PoseLandmarks landmarks{};
  float scale = 2.0f * region.radius / kPoseSize;
  float c = std::cos(region.angle), s = std::sin(region.angle);
  for (uint32_t i = 0; i < pose_point::kCount; ++i) {
    size_t n = kBlazePose[i];
    for (size_t j = 0; j < 5; ++j) {
      if (!std::isfinite(image[n * 5 + j])) {
        return false;
      }
    }
    for (size_t j = 0; j < 3; ++j) {
      if (!std::isfinite(world[n * 3 + j]) || std::abs(world[n * 3 + j]) > 10) {
        return false;
      }
    }
    // Image points: x, y in the 256x256 crop, z, then visibility and
    // presence logits.
    auto xy = PicturePoint(region, (image[n * 5] - 128.0f) * scale,
                           (image[n * 5 + 1] - 128.0f) * scale);
    auto& point = landmarks[i];
    point.x = xy[0];
    point.y = xy[1];
    point.score =
        std::min(Probability(image[n * 5 + 3]), Probability(image[n * 5 + 4]));
    // The crop's turn is undone for x and y only; z is already metric.
    point.world = {c * world[n * 3] - s * world[n * 3 + 1],
                   s * world[n * 3] + c * world[n * 3 + 1], world[n * 3 + 2]};
    point.has_world = true;
  }
  for (uint32_t n : {pose_point::kShoulderLeft, pose_point::kShoulderRight,
                     pose_point::kHipLeft, pose_point::kHipRight}) {
    if (landmarks[n].score < threshold) {
      return false;
    }
  }
  auto hip = PicturePoint(region, (image[33 * 5] - 128.0f) * scale,
                          (image[33 * 5 + 1] - 128.0f) * scale);
  auto full = PicturePoint(region, (image[34 * 5] - 128.0f) * scale,
                           (image[34 * 5 + 1] - 128.0f) * scale);
  PoseRegion crop = RegionFromPoints(hip[0], hip[1], full[0], full[1]);
  if (Usable(crop) && crop.radius > region.radius * 0.5f &&
      crop.radius < region.radius * 2.0f) {
    next = crop;
  }
  output = landmarks;
  return true;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
