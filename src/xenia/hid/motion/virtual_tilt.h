/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_VIRTUAL_TILT_H_
#define XENIA_HID_MOTION_VIRTUAL_TILT_H_

#include <array>
#include <cstdint>
#include <mutex>

namespace xe {
namespace hid {
namespace motion {

// The Kinect v1's motor range, in degrees above level. The title's NUI
// library refuses anything outside it (docs/kinect-pscam.md section 13).
constexpr int32_t kV1TiltMinDegrees = -27;
constexpr int32_t kV1TiltMaxDegrees = 27;
// Accelerometer counts per g. The title only uses the direction (section
// 13), so the scale only has to be plausible; this is the Kinect v1's, as
// libfreenect documents it.
constexpr int32_t kV1AccelerometerCountsPerG = 819;

// The Kinect v1's tilt motor and accelerometer, for devices without them
// (a Kinect v2, a webcam) or before a real motor is driven.
//
// What a title needs from it (section 13): the motor stops where it was told
// (the library resets its skeleton tracker while the motor reports moving,
// so this one arrives at once and never reports moving), the accelerometer
// agrees with that angle (otherwise the library sends the angle again), and
// a floor it asked to remember is given back. The camera picture itself does
// not tilt.
//
// Safe to use from any thread.
class VirtualTilt {
 public:
  // The angle the motor is at.
  int32_t angle() const;
  // Moves to `degrees`, clamped to the motor's range. Returns the angle
  // reached.
  int32_t SetAngle(int32_t degrees);

  // The accelerometer: gravity as (x, y, z) counts, with y up through the
  // camera's top when level and z turning towards y as the camera tilts up,
  // so that atan2(z, y) is the tilt angle (how the title reads it back).
  std::array<int32_t, 3> gravity() const;

  // The floor the title last asked to remember (plane nx, ny, nz, d in its
  // skeleton space), and the accelerometer at that moment. A remembered
  // floor has d != 0 (how the title tests for one); none is all zero.
  void RememberFloor(const std::array<float, 4>& plane);
  std::array<float, 4> floor_plane() const;
  std::array<int32_t, 3> floor_gravity() const;

  // Level, nothing remembered: a fresh device.
  void Reset();

 private:
  static std::array<int32_t, 3> GravityAt(int32_t degrees);

  mutable std::mutex lock_;
  int32_t angle_ = 0;
  std::array<float, 4> floor_plane_ = {};
  std::array<int32_t, 3> floor_gravity_ = {};
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_VIRTUAL_TILT_H_
