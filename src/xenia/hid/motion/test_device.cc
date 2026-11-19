/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/test_device.h"

#include <cmath>

#include "xenia/base/cvar.h"
#include "xenia/hid/motion/synthetic_depth.h"

DEFINE_bool(motion_test_raise_hand, false,
            "With motion_device = test, the player raises the right hand in "
            "front of them for 8 s out of every 12 s and moves it slowly "
            "from side to side, as a player does to bring up a Kinect hand "
            "cursor.",
            "HID");

namespace xe {
namespace hid {
namespace motion {

namespace {

using Clock = std::chrono::steady_clock;

constexpr auto kFrameInterval = std::chrono::microseconds(33333);

}  // namespace

bool TestMotionDevice::Open() {
  opened_at_ = Clock::now();
  return true;
}

MotionCapabilities TestMotionDevice::capabilities() const {
  MotionCapabilities capabilities;
  capabilities.depth = true;
  capabilities.bodies = true;
  capabilities.max_bodies = 1;
  return capabilities;
}

MotionStatus TestMotionDevice::status() const {
  MotionStatus status;
  status.state = MotionDeviceState::kRunning;
  return status;
}

bool TestMotionDevice::GetNewestFrame(SensorFrame& frame) {
  // A frame every 1/30 s since Open, made when asked for.
  Clock::time_point now = Clock::now();
  uint64_t number = uint64_t((now - opened_at_) / kFrameInterval) + 1;
  if (number <= frame.number) {
    return false;
  }
  SyntheticRoom room;
  room.tilt_degrees = view_tilt_;
  Body player =
      cvars::motion_test_raise_hand
          ? RaisingHandPlayer(
                std::chrono::duration<double>(now - opened_at_).count())
          : StandingPlayer();
  frame = SensorFrame();
  frame.number = number;
  frame.captured_at = now;
  frame.depth = DrawSyntheticDepth(&player, room);
  frame.bodies.push_back(player);
  // The drawn floor, in skeleton space: y = -sensor_height.
  frame.floor_plane = {0.0f, 1.0f, 0.0f, room.sensor_height};
  return true;
}

Body TestMotionDevice::StandingPlayer() {
  Body body;
  body.tracking_id = 1;
  auto put = [&](Joint joint, float x, float y, float z = 2.5f) {
    body.joints[size_t(joint)] = {x, y, z};
    body.joint_states[size_t(joint)] = JointState::kTracked;
  };
  // Hips at the sensor's height, feet just above the floor 0.9 m below,
  // arms hanging.
  put(Joint::kHipCenter, 0.0f, 0.0f);
  put(Joint::kSpine, 0.0f, 0.25f);
  put(Joint::kShoulderCenter, 0.0f, 0.5f);
  put(Joint::kHead, 0.0f, 0.75f);
  put(Joint::kShoulderLeft, -0.18f, 0.5f);
  put(Joint::kElbowLeft, -0.2f, 0.22f);
  put(Joint::kWristLeft, -0.21f, -0.03f);
  put(Joint::kHandLeft, -0.21f, -0.06f);
  put(Joint::kShoulderRight, 0.18f, 0.5f);
  put(Joint::kElbowRight, 0.2f, 0.22f);
  put(Joint::kWristRight, 0.21f, -0.03f);
  put(Joint::kHandRight, 0.21f, -0.06f);
  put(Joint::kHipLeft, -0.1f, 0.0f);
  put(Joint::kKneeLeft, -0.1f, -0.45f);
  put(Joint::kAnkleLeft, -0.1f, -0.82f);
  put(Joint::kFootLeft, -0.1f, -0.855f, 2.42f);
  put(Joint::kHipRight, 0.1f, 0.0f);
  put(Joint::kKneeRight, 0.1f, -0.45f);
  put(Joint::kAnkleRight, 0.1f, -0.82f);
  put(Joint::kFootRight, 0.1f, -0.855f, 2.42f);
  return body;
}

Body TestMotionDevice::RaisingHandPlayer(double seconds) {
  Body body = StandingPlayer();
  // Standing for the first 4 s of every 12 s.
  double in_cycle = std::fmod(seconds, 12.0);
  if (in_cycle < 4.0) {
    return body;
  }
  auto put = [&](Joint joint, float x, float y, float z) {
    body.joints[size_t(joint)] = {x, y, z};
  };
  // The right hand held up in front, at shoulder height and about half a
  // metre nearer the sensor than the body, swaying 15 cm to each side every
  // 4 s.
  float sway = 0.15f * float(std::sin((in_cycle - 4.0) * 3.14159265 / 2.0));
  put(Joint::kElbowRight, 0.24f + sway * 0.5f, 0.42f, 2.25f);
  put(Joint::kWristRight, 0.26f + sway, 0.52f, 2.0f);
  put(Joint::kHandRight, 0.26f + sway, 0.56f, 1.95f);
  return body;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
