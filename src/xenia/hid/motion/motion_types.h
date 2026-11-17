/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_MOTION_TYPES_H_
#define XENIA_HID_MOTION_MOTION_TYPES_H_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

// Host-side data a motion-capture device (a Kinect v1, a Kinect v2, a webcam
// with a pose model) hands to the emulator. Nothing here is a guest type: the
// guest-facing code turns these into what an Xbox 360 Kinect camera sends.
// See docs/motion-capture.md.

namespace xe {
namespace hid {
namespace motion {

// A pinhole lens with radial/tangential distortion (Brown-Conrady, the model
// both Kinect SDKs and libfreenect2 report). Pixel coordinates have their
// origin at the top-left pixel's top-left corner. A lens with all-zero
// distortion is an ideal pinhole.
struct CameraIntrinsics {
  float focal_x = 0.0f;
  float focal_y = 0.0f;
  float centre_x = 0.0f;
  float centre_y = 0.0f;
  // k1, k2, k3 radial; p1, p2 tangential.
  float k1 = 0.0f, k2 = 0.0f, k3 = 0.0f;
  float p1 = 0.0f, p2 = 0.0f;

  bool valid() const { return focal_x > 0.0f && focal_y > 0.0f; }
};

// Where the colour camera sits relative to the depth camera: a point p in
// depth-camera space (metres) is rotation * p + translation in colour-camera
// space. Row-major rotation.
struct DepthToColour {
  std::array<float, 9> rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::array<float, 3> translation = {0, 0, 0};
};

// A depth image: one 16-bit distance along the camera's z axis per pixel, in
// millimetres, rows packed top to bottom. 0 means no reading.
struct DepthImage {
  uint32_t width = 0;
  uint32_t height = 0;
  CameraIntrinsics lens;
  std::vector<uint16_t> millimetres;
  // Optional: each pixel's ray as two floats (x/z, y/z, in lens space: +x to
  // the image's right, +y down, +z forward), already free of distortion.
  // A device that knows them exactly (the Kinect v2 SDK's depth-to-camera
  // table) fills these; otherwise they are worked out from `lens`.
  std::vector<float> rays;

  bool empty() const { return millimetres.empty(); }
};

// A colour image: 8-bit R, G, B, A per pixel, rows packed top to bottom.
struct ColourImage {
  uint32_t width = 0;
  uint32_t height = 0;
  CameraIntrinsics lens;
  std::vector<uint8_t> rgba;

  bool empty() const { return rgba.empty(); }
};

// The 20 joints of an Xbox 360 / Kinect for Windows v1 skeleton, in the
// order of the SDK's NUI_SKELETON_POSITION_INDEX. A Kinect v2's 25 joints
// and a pose model's landmarks are mapped onto these.
enum class Joint : uint8_t {
  kHipCenter,
  kSpine,
  kShoulderCenter,
  kHead,
  kShoulderLeft,
  kElbowLeft,
  kWristLeft,
  kHandLeft,
  kShoulderRight,
  kElbowRight,
  kWristRight,
  kHandRight,
  kHipLeft,
  kKneeLeft,
  kAnkleLeft,
  kFootLeft,
  kHipRight,
  kKneeRight,
  kAnkleRight,
  kFootRight,
  kCount,
};
constexpr size_t kJointCount = size_t(Joint::kCount);

enum class JointState : uint8_t { kNotTracked, kInferred, kTracked };

// One tracked person, in skeleton space: metres, +x to the player's right
// (the sensor's left), +y up, +z away from the sensor.
struct Body {
  // The device's id for this person, stable while they stay tracked.
  uint32_t tracking_id = 0;
  std::array<std::array<float, 3>, kJointCount> joints = {};
  std::array<JointState, kJointCount> joint_states = {};
};

// Everything a device saw at one moment. Each part is optional: a device
// fills what it has (see MotionCapabilities).
struct SensorFrame {
  // Counts up from 1 per frame the device produces; 0 means "no frame yet".
  uint64_t number = 0;
  std::chrono::steady_clock::time_point captured_at = {};

  DepthImage depth;
  ColourImage colour;
  DepthToColour depth_to_colour;

  std::vector<Body> bodies;

  // The floor as the device sees it (ax + by + cz + d = 0, in skeleton
  // space) and which way is down. All zero when unknown.
  std::array<float, 4> floor_plane = {};
  std::array<float, 3> gravity = {};
};

// What a device can give; the guest-facing code decides from this, never
// from which device it is.
struct MotionCapabilities {
  bool depth = false;
  bool colour = false;
  bool bodies = false;
  // A real motor the title's tilt requests can move. Without one the tilt is
  // emulated.
  bool tilt_motor = false;
  uint32_t max_bodies = 0;
};

// Whether a device is working, and if not, why, in words a player can act on
// ("Kinect for Windows SDK 2.0 is not installed").
enum class MotionDeviceState : uint8_t {
  kNone,           // no device chosen
  kNotFound,       // chosen, but not connected
  kDriverMissing,  // its runtime or driver is not installed
  kFailed,         // found, but would not start
  kRunning,
};

struct MotionStatus {
  MotionDeviceState state = MotionDeviceState::kNone;
  std::string reason;
};

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_MOTION_TYPES_H_
