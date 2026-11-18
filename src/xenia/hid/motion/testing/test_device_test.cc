/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/test_device.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

TEST_CASE("The test device is always there and sees one player",
          "[motion][test_device]") {
  TestMotionDevice device;
  REQUIRE(device.Open());
  REQUIRE(device.status().state == MotionDeviceState::kRunning);

  SensorFrame frame;
  REQUIRE(device.GetNewestFrame(frame));
  REQUIRE(frame.number >= 1);
  REQUIRE(frame.bodies.size() == 1);
  REQUIRE(frame.depth.width == 640);
  REQUIRE(frame.depth.height == 480);
  // The player stands in the middle, 2.5 m away, nearer than the back wall.
  uint16_t middle = frame.depth.millimetres[size_t(220) * 640 + 320];
  REQUIRE(middle > 2300);
  REQUIRE(middle < 2600);

  // Asking again within the same 1/30 s gives nothing new.
  SensorFrame again = frame;
  again.number = frame.number + 1000000;
  REQUIRE_FALSE(device.GetNewestFrame(again));
}

}  // namespace xe::hid::motion::test
