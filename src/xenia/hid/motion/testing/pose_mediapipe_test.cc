/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// Ported from Free Riders Recompiled (tests/pose_mediapipe_test.cpp), written
// by its author YuutaTsubasa.

#include <cmath>
#include <limits>
#include <numbers>

#include "xenia/hid/motion/pose_mediapipe.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

bool Near(float a, float b) { return std::abs(a - b) < 0.001f; }

}  // namespace

TEST_CASE("The person detector's anchors and best person",
          "[motion][mediapipe]") {
  REQUIRE(Near(PersonAnchor(0)[0], 0.5f / 28));
  REQUIRE(PersonAnchor(0) == PersonAnchor(1));  // two per first-grid cell
  REQUIRE(Near(PersonAnchor(1568)[0], 0.5f / 14));
  REQUIRE(Near(PersonAnchor(2253)[0], 6.5f / 7));

  std::vector<float> boxes(kPersonAnchorCount * 12),
      scores(kPersonAnchorCount, -100);
  const size_t index = 800;
  auto anchor = PersonAnchor(index);
  scores[index] = 10;
  boxes[index * 12 + 4] = (0.5f - anchor[0]) * 224;
  boxes[index * 12 + 5] = (0.5f - anchor[1]) * 224;
  boxes[index * 12 + 6] = (0.5f - anchor[0]) * 224;
  boxes[index * 12 + 7] = (0.125f - anchor[1]) * 224;
  auto region = DecodePerson(boxes, scores, 640, 480, 0.5f);
  REQUIRE(region);
  // The landscape letterbox is taken off again.
  REQUIRE(Near(region->x, 320));
  REQUIRE(Near(region->y, 240));
  // Hip centre to the full-body point, with the 1.25 margin, upright.
  REQUIRE(Near(region->radius, 300));
  REQUIRE(Near(region->angle, 0));

  scores[index] = -100;
  REQUIRE_FALSE(DecodePerson(boxes, scores, 640, 480, 0.5f));
  REQUIRE_FALSE(DecodePerson({}, scores, 640, 480, 0.5f));
}

TEST_CASE("What each network is fed", "[motion][mediapipe]") {
  ColourImage picture;
  picture.width = picture.height = 2;
  picture.rgba = {255, 0, 0, 255, 255, 0, 0, 255,
                  255, 0, 0, 255, 255, 0, 0, 255};  // red
  const PoseRegion region{1, 1, 1, 0};

  auto pose = MediaPipeInput(picture, region, false);
  REQUIRE(pose.size() == 256 * 256 * 3);  // interleaved RGB
  const size_t p = (128 * 256 + 128) * 3;
  REQUIRE(Near(pose[p], 1));
  REQUIRE(Near(pose[p + 1], 0));
  REQUIRE(Near(pose[p + 2], 0));

  auto detector = MediaPipeInput(picture, region, true);
  const size_t d = 112 * 224 + 112;
  REQUIRE(detector.size() == 224 * 224 * 3);  // RGB planes, -1..1
  REQUIRE(Near(detector[d], 1));
  REQUIRE(Near(detector[224 * 224 + d], -1));

  picture.rgba.clear();
  REQUIRE(MediaPipeInput(picture, region, false).empty());
}

TEST_CASE("The pose network's answer becomes 3D points",
          "[motion][mediapipe]") {
  std::vector<float> image(195, 0), world(117, 0);
  for (size_t i = 0; i < 39; ++i) {
    image[i * 5] = 128;
    image[i * 5 + 1] = 128;
    image[i * 5 + 3] = image[i * 5 + 4] = 10;
  }
  // The right wrist (BlazePose 16) reaching forward, whatever its place in
  // the picture.
  world[16 * 3] = 0.2f;
  world[16 * 3 + 1] = -0.1f;
  world[16 * 3 + 2] = -0.4f;
  image[33 * 5] = 128;
  image[33 * 5 + 1] = 128;
  image[34 * 5] = 128;
  image[34 * 5 + 1] = 0;

  PoseLandmarks out{};
  std::optional<PoseRegion> next;
  const PoseRegion crop{320, 240, 240, 0};
  REQUIRE(DecodeMediaPipe(image, world, 0.9f, crop, 0.5f, out, next));
  const auto& wrist = out[pose_point::kWristRight];
  // Depth stays in metres, not scaled with the crop.
  REQUIRE(wrist.has_world);
  REQUIRE(Near(wrist.world[2], -0.4f));
  REQUIRE(Near(wrist.x, 320));
  REQUIRE(Near(wrist.y, 240));
  // The next crop keeps the training margin, so it does not shrink.
  REQUIRE(next);
  REQUIRE(Near(next->radius, 300));
  REQUIRE(Near(next->angle, 0));

  // A turned crop: x and y are turned back, z is not.
  REQUIRE(DecodeMediaPipe(image, world, 0.9f,
                          {320, 240, 240, std::numbers::pi_v<float> / 2}, 0.5f,
                          out, next));
  REQUIRE(Near(wrist.world[0], 0.1f));
  REQUIRE(Near(wrist.world[1], 0.2f));
  REQUIRE(Near(wrist.world[2], -0.4f));

  // A broken depth is refused, and the last points stay.
  world[16 * 3 + 2] = std::numeric_limits<float>::quiet_NaN();
  auto old = out[pose_point::kWristRight].world;
  REQUIRE_FALSE(DecodeMediaPipe(image, world, 0.9f, crop, 0.5f, out, next));
  REQUIRE(out[pose_point::kWristRight].world == old);
  REQUIRE_FALSE(next);

  // A torso point without confidence refuses the whole body.
  world[16 * 3 + 2] = -0.4f;
  image[11 * 5 + 4] = -10;
  REQUIRE_FALSE(DecodeMediaPipe(image, world, 0.9f, crop, 0.5f, out, next));
}

}  // namespace xe::hid::motion::test
