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

TEST_CASE("The test player raises a hand in front of them now and then",
          "[motion][test_device]") {
  Body standing = TestMotionDevice::StandingPlayer();
  // Standing for the first 4 s.
  Body early = TestMotionDevice::RaisingHandPlayer(1.0);
  REQUIRE(early.joints == standing.joints);
  // Then the right hand is up at the shoulder, nearer the sensor.
  for (double seconds : {5.0, 8.0, 11.5, 17.0}) {
    Body up = TestMotionDevice::RaisingHandPlayer(seconds);
    const auto& hand = up.joints[size_t(Joint::kHandRight)];
    const auto& shoulder = up.joints[size_t(Joint::kShoulderRight)];
    REQUIRE(hand[1] > shoulder[1]);
    REQUIRE(hand[2] < shoulder[2] - 0.4f);
    // The left hand stays down.
    REQUIRE(up.joints[size_t(Joint::kHandLeft)] ==
            standing.joints[size_t(Joint::kHandLeft)]);
  }
  // And it moves from side to side.
  float x5 = TestMotionDevice::RaisingHandPlayer(5.0)
                 .joints[size_t(Joint::kHandRight)][0];
  float x7 = TestMotionDevice::RaisingHandPlayer(7.0)
                 .joints[size_t(Joint::kHandRight)][0];
  REQUIRE(x5 - x7 > 0.25f);
  // And comes down again at the start of the next 12 s.
  REQUIRE(TestMotionDevice::RaisingHandPlayer(13.0).joints == standing.joints);
}

}  // namespace xe::hid::motion::test
