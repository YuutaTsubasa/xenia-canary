/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/depth_picture.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

// 5x3: a wall at 4 m with one pixel missing and a near point in the middle.
DepthImage SmallPicture() {
  DepthImage depth;
  depth.width = 5;
  depth.height = 3;
  depth.millimetres.assign(15, 4000);
  depth.millimetres[0] = 0;
  depth.millimetres[7] = 1200;  // row 1, column 2: the middle
  return depth;
}

}  // namespace

TEST_CASE("A depth summary counts readings and finds the nearest",
          "[motion][depth_picture]") {
  DepthSummary summary = SummarizeDepth(SmallPicture());
  REQUIRE(summary.with_reading == Approx(14.0f / 15.0f));
  REQUIRE(summary.nearest_mm == 1200);
  REQUIRE(summary.middle_mm == 1200);
  REQUIRE(summary.nearer_than_3m == 1);

  DepthSummary none = SummarizeDepth(DepthImage());
  REQUIRE(none.with_reading == 0.0f);
  REQUIRE(none.nearest_mm == 0);
}

TEST_CASE("A depth picture becomes a grey bitmap, top row first",
          "[motion][depth_picture]") {
  std::vector<uint8_t> bmp = DepthToBitmap(SmallPicture());
  // Rows of 5 pixels are padded to 8 bytes.
  const size_t pixels_at = 14 + 40 + 256 * 4;
  REQUIRE(bmp.size() == pixels_at + 8 * 3);
  REQUIRE(bmp[0] == 'B');
  REQUIRE(bmp[1] == 'M');
  REQUIRE(bmp[28] == 8);
  // Stored top row first: no reading (black) at the very start.
  REQUIRE(bmp[pixels_at + 0] == 0);
  // The near point is lighter than the wall beside it.
  uint8_t near = bmp[pixels_at + 8 + 2];
  uint8_t wall = bmp[pixels_at + 8 + 1];
  REQUIRE(near > wall);
  REQUIRE(wall > 0);

  REQUIRE(DepthToBitmap(DepthImage()).empty());
}

}  // namespace xe::hid::motion::test
