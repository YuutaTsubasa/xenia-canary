/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <cstdlib>

#include "xenia/hid/motion/pose_estimator.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

TEST_CASE("Missing pose models are reported, not crashed on",
          "[motion][pose_estimator]") {
  std::string reason;
  auto estimator = PoseEstimator::Open("this folder does not exist", reason);
  REQUIRE_FALSE(estimator);
  REQUIRE(reason.find(kPoseDetectorModel) != std::string::npos);
}

// Runs the real models when XENIA_TEST_POSE_FOLDER names a folder holding
// them and ONNX Runtime (none are in the repository).
TEST_CASE("The real pose models load and look at a picture",
          "[motion][pose_estimator]") {
  const char* folder = std::getenv("XENIA_TEST_POSE_FOLDER");
  if (!folder || !*folder) {
    SUCCEED("XENIA_TEST_POSE_FOLDER not set");
    return;
  }
  std::string reason;
  auto estimator = PoseEstimator::Open(folder, reason);
  INFO(reason);
  REQUIRE(estimator);
  // A grey picture holds no one.
  ColourImage picture;
  picture.width = 640;
  picture.height = 480;
  picture.rgba.assign(size_t(640) * 480 * 4, 128);
  PoseLandmarks landmarks{};
  REQUIRE_FALSE(estimator->Estimate(picture, landmarks));
}

}  // namespace xe::hid::motion::test
