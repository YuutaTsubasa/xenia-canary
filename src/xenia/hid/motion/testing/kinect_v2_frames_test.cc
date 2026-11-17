/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <cmath>
#include <vector>

#include "xenia/hid/motion/kinect_v2_frames.h"
#include "xenia/hid/motion/virtual_kinect_v1.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

// A Kinect v2 frame that sees nothing but one pixel, whose table entry is
// (table_x, table_y) in SDK camera space, at `mm`.
DepthImage OnePoint(float table_x, float table_y, uint16_t mm) {
  std::vector<float> table(size_t(kV2DepthWidth) * kV2DepthHeight * 2, 0.0f);
  table[0] = table_x;
  table[1] = table_y;
  DepthImage depth;
  depth.width = kV2DepthWidth;
  depth.height = kV2DepthHeight;
  depth.millimetres.assign(size_t(kV2DepthWidth) * kV2DepthHeight, 0);
  depth.millimetres[0] = mm;
  depth.rays = KinectV2TableToRays(table);
  return depth;
}

// Where the one point lands in the virtual v1's picture: the middle of the
// pixels it covers.
bool FindPoint(const DepthImage& v1, float& x, float& y) {
  float sum_x = 0, sum_y = 0;
  int count = 0;
  for (uint32_t py = 0; py < v1.height; ++py) {
    for (uint32_t px = 0; px < v1.width; ++px) {
      if (v1.millimetres[size_t(py) * v1.width + px]) {
        sum_x += px + 0.5f;
        sum_y += py + 0.5f;
        ++count;
      }
    }
  }
  if (!count) {
    return false;
  }
  x = sum_x / count;
  y = sum_y / count;
  return true;
}

}  // namespace

// A Kinect v1 sends its picture as the camera sees it, not mirrored
// (docs/kinect-pscam.md section 11). Something on the sensor's own left, the
// SDK's +X, is on the left of that picture; something above is at the top.
TEST_CASE("Kinect v2 points keep their side in the v1 picture",
          "[motion][kinect_v2]") {
  float x, y;
  REQUIRE(FindPoint(ReprojectToV1Depth(OnePoint(0.3f, 0.0f, 2000)), x, y));
  REQUIRE(x < 320);
  REQUIRE(std::abs(y - 240.0f) <= 1.0f);

  REQUIRE(FindPoint(ReprojectToV1Depth(OnePoint(-0.3f, 0.0f, 2000)), x, y));
  REQUIRE(x >= 320);

  REQUIRE(FindPoint(ReprojectToV1Depth(OnePoint(0.0f, 0.2f, 2000)), x, y));
  REQUIRE(y < 240);
  REQUIRE(std::abs(x - 320.0f) <= 1.0f);
}

TEST_CASE("Kinect v2 point lands where the v1 lens puts it",
          "[motion][kinect_v2]") {
  float x, y;
  REQUIRE(FindPoint(ReprojectToV1Depth(OnePoint(0.3f, -0.1f, 2000)), x, y));
  CameraIntrinsics lens = V1DepthLens();
  // x/z = -0.3, y/z = +0.1 in lens space.
  REQUIRE(std::abs(x - (lens.centre_x - 0.3f * lens.focal_x)) <= 1.0f);
  REQUIRE(std::abs(y - (lens.centre_y + 0.1f * lens.focal_y)) <= 1.0f);
}

TEST_CASE("An empty Kinect v2 table is not taken as real",
          "[motion][kinect_v2]") {
  std::vector<float> table(size_t(kV2DepthWidth) * kV2DepthHeight * 2, 0.0f);
  REQUIRE_FALSE(KinectV2TableLooksValid(table, kV2DepthWidth, kV2DepthHeight));
  table[0] = 0.72f;
  table[1] = 0.6f;
  REQUIRE(KinectV2TableLooksValid(table, kV2DepthWidth, kV2DepthHeight));
  table[0] = std::nanf("");
  REQUIRE_FALSE(KinectV2TableLooksValid(table, kV2DepthWidth, kV2DepthHeight));
  REQUIRE_FALSE(KinectV2TableLooksValid(table, 10, 10));
}

}  // namespace xe::hid::motion::test
