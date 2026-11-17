/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/virtual_tilt.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace xe {
namespace hid {
namespace motion {

int32_t VirtualTilt::angle() const {
  std::lock_guard<std::mutex> guard(lock_);
  return angle_;
}

int32_t VirtualTilt::SetAngle(int32_t degrees) {
  std::lock_guard<std::mutex> guard(lock_);
  angle_ = std::clamp(degrees, kV1TiltMinDegrees, kV1TiltMaxDegrees);
  return angle_;
}

std::array<int32_t, 3> VirtualTilt::gravity() const {
  std::lock_guard<std::mutex> guard(lock_);
  return GravityAt(angle_);
}

void VirtualTilt::RememberFloor(const std::array<float, 4>& plane) {
  std::lock_guard<std::mutex> guard(lock_);
  floor_plane_ = plane;
  floor_gravity_ = GravityAt(angle_);
}

std::array<float, 4> VirtualTilt::floor_plane() const {
  std::lock_guard<std::mutex> guard(lock_);
  return floor_plane_;
}

std::array<int32_t, 3> VirtualTilt::floor_gravity() const {
  std::lock_guard<std::mutex> guard(lock_);
  return floor_gravity_;
}

void VirtualTilt::Reset() {
  std::lock_guard<std::mutex> guard(lock_);
  angle_ = 0;
  floor_plane_ = {};
  floor_gravity_ = {};
}

std::array<int32_t, 3> VirtualTilt::GravityAt(int32_t degrees) {
  double radians = degrees * std::numbers::pi / 180.0;
  return {0,
          int32_t(std::lround(kV1AccelerometerCountsPerG * std::cos(radians))),
          int32_t(std::lround(kV1AccelerometerCountsPerG * std::sin(radians)))};
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
