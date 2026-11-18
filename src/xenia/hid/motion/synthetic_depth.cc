/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/synthetic_depth.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "xenia/hid/motion/depth_reprojection.h"
#include "xenia/hid/motion/virtual_kinect_v1.h"

// Ray casting from the sensor: each pixel's ray is followed into the room,
// and the nearest of the floor, the back wall and the body's limbs is what
// the pixel reads. Limbs are capsules (a cylinder with a ball on each end);
// the ray-capsule test is the usual one (closest approach to the axis, then
// the end balls).

namespace xe {
namespace hid {
namespace motion {

namespace {

using Vector = std::array<float, 3>;

Vector Subtract(const Vector& a, const Vector& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

float Dot(const Vector& a, const Vector& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

constexpr float kMiss = std::numeric_limits<float>::infinity();

// The distance along the unit ray `direction` from the sensor to a ball, or
// kMiss.
float HitBall(const Vector& direction, const Vector& centre, float radius) {
  float along = Dot(direction, centre);
  float square = along * along - (Dot(centre, centre) - radius * radius);
  if (square < 0.0f) {
    return kMiss;
  }
  float t = along - std::sqrt(square);
  return t > 0.0f ? t : kMiss;
}

// The distance along the unit ray `direction` from the sensor to the capsule
// around segment a-b, or kMiss.
float HitCapsule(const Vector& direction, const Vector& a, const Vector& b,
                 float radius) {
  float nearest =
      std::min(HitBall(direction, a, radius), HitBall(direction, b, radius));
  Vector axis = Subtract(b, a);
  float axis_length2 = Dot(axis, axis);
  if (axis_length2 < 1e-8f) {
    return nearest;
  }
  // The cylinder: points at distance `radius` from the axis line, between
  // the ends. With the ray origin at 0, the offset from a is -a.
  Vector from_a = {-a[0], -a[1], -a[2]};
  float axis_ray = Dot(axis, direction);
  float axis_from = Dot(axis, from_a);
  float k2 = axis_length2 - axis_ray * axis_ray;
  if (k2 > 1e-8f) {
    float k1 = axis_length2 * Dot(direction, from_a) - axis_from * axis_ray;
    float k0 = axis_length2 * Dot(from_a, from_a) - axis_from * axis_from -
               radius * radius * axis_length2;
    float square = k1 * k1 - k2 * k0;
    if (square >= 0.0f) {
      float t = (-k1 - std::sqrt(square)) / k2;
      float along_axis = axis_from + t * axis_ray;
      if (t > 0.0f && along_axis > 0.0f && along_axis < axis_length2) {
        nearest = std::min(nearest, t);
      }
    }
  }
  return nearest;
}

struct Limb {
  Joint from, to;
  float radius;  // metres, for a body with shoulders 0.36 m apart
};

// The body's limbs. Sizes are a typical adult's at the width PoseToBody
// scales every player to; the torso is three capsules side by side so it is
// wider than it is deep.
constexpr Limb kLimbs[] = {
    {Joint::kHead, Joint::kHead, 0.10f},
    {Joint::kShoulderCenter, Joint::kHead, 0.05f},
    {Joint::kHipCenter, Joint::kShoulderCenter, 0.12f},
    {Joint::kHipLeft, Joint::kShoulderLeft, 0.08f},
    {Joint::kHipRight, Joint::kShoulderRight, 0.08f},
    {Joint::kShoulderLeft, Joint::kShoulderRight, 0.06f},
    {Joint::kHipLeft, Joint::kHipRight, 0.09f},
    {Joint::kShoulderLeft, Joint::kElbowLeft, 0.045f},
    {Joint::kElbowLeft, Joint::kWristLeft, 0.038f},
    {Joint::kWristLeft, Joint::kHandLeft, 0.04f},
    {Joint::kShoulderRight, Joint::kElbowRight, 0.045f},
    {Joint::kElbowRight, Joint::kWristRight, 0.038f},
    {Joint::kWristRight, Joint::kHandRight, 0.04f},
    {Joint::kHipLeft, Joint::kKneeLeft, 0.07f},
    {Joint::kKneeLeft, Joint::kAnkleLeft, 0.05f},
    {Joint::kAnkleLeft, Joint::kFootLeft, 0.045f},
    {Joint::kHipRight, Joint::kKneeRight, 0.07f},
    {Joint::kKneeRight, Joint::kAnkleRight, 0.05f},
    {Joint::kAnkleRight, Joint::kFootRight, 0.045f},
};

}  // namespace

DepthImage DrawSyntheticDepth(const Body* body, const SyntheticRoom& room) {
  const CameraIntrinsics lens = V1DepthLens();
  const uint32_t width = kV1DepthWidth, height = kV1DepthHeight;
  const float tilt = room.tilt_degrees * std::numbers::pi_v<float> / 180.0f;
  const float tilt_cos = std::cos(tilt), tilt_sin = std::sin(tilt);

  // Lens space (+x right, +y down) to the sensor's own upright frame (+x
  // left, +y up), then turned by the tilt into room space.
  auto lens_to_room = [&](float x, float y, float z) {
    float up = -y;
    return Vector{-x, up * tilt_cos + z * tilt_sin,
                  -up * tilt_sin + z * tilt_cos};
  };
  // And back, for placing limbs in the picture.
  auto room_to_lens = [&](const Vector& p) {
    float up = p[1] * tilt_cos - p[2] * tilt_sin;
    float ahead = p[1] * tilt_sin + p[2] * tilt_cos;
    return Vector{-p[0], -up, ahead};
  };

  // Each pixel's unit ray in room space, and how far along the lens axis one
  // metre along the ray is (the depth a pixel reads is that distance).
  std::vector<Vector> rays(size_t(width) * height);
  std::vector<float> depth_per_metre(rays.size());
  std::vector<float> nearest(rays.size(), kMiss);
  for (uint32_t v = 0; v < height; ++v) {
    for (uint32_t u = 0; u < width; ++u) {
      auto ray = PixelToRay(lens, float(u) + 0.5f, float(v) + 0.5f);
      float length = std::sqrt(ray[0] * ray[0] + ray[1] * ray[1] + 1.0f);
      size_t i = size_t(v) * width + u;
      rays[i] = lens_to_room(ray[0] / length, ray[1] / length, 1.0f / length);
      depth_per_metre[i] = 1.0f / length;
      // The room: the floor below the sensor and the wall ahead.
      const Vector& d = rays[i];
      if (d[1] < 0.0f) {
        nearest[i] = std::min(nearest[i], -room.sensor_height / d[1]);
      }
      if (d[2] > 0.0f) {
        nearest[i] = std::min(nearest[i], room.back_wall / d[2]);
      }
    }
  }

  if (body) {
    for (const Limb& limb : kLimbs) {
      if (body->joint_states[size_t(limb.from)] == JointState::kNotTracked ||
          body->joint_states[size_t(limb.to)] == JointState::kNotTracked) {
        continue;
      }
      const Vector& a = body->joints[size_t(limb.from)];
      const Vector& b = body->joints[size_t(limb.to)];
      // The pixels the limb can cover: around where its ends fall in the
      // picture, widened by its radius there (generously, since a ball off
      // the lens axis looks a little wider than straight ahead). A limb
      // reaching the sensor's own plane may be anywhere.
      Vector la = room_to_lens(a), lb = room_to_lens(b);
      float closest = std::min(la[2], lb[2]) - limb.radius;
      uint32_t left = 0, top = 0, right = width, bottom = height;
      if (closest > 0.05f) {
        float ua = lens.centre_x + lens.focal_x * la[0] / la[2];
        float va = lens.centre_y + lens.focal_y * la[1] / la[2];
        float ub = lens.centre_x + lens.focal_x * lb[0] / lb[2];
        float vb = lens.centre_y + lens.focal_y * lb[1] / lb[2];
        float margin = 1.5f * limb.radius * lens.focal_x / closest + 2.0f;
        auto clamp = [](float value, uint32_t limit) {
          return uint32_t(std::clamp(value, 0.0f, float(limit)));
        };
        left = clamp(std::min(ua, ub) - margin, width);
        right = clamp(std::max(ua, ub) + margin + 1.0f, width);
        top = clamp(std::min(va, vb) - margin, height);
        bottom = clamp(std::max(va, vb) + margin + 1.0f, height);
      } else if (std::max(la[2], lb[2]) + limb.radius <= 0.0f) {
        continue;  // wholly behind the sensor
      }
      for (uint32_t v = top; v < bottom; ++v) {
        for (uint32_t u = left; u < right; ++u) {
          size_t i = size_t(v) * width + u;
          float t = HitCapsule(rays[i], a, b, limb.radius);
          if (t < nearest[i]) {
            nearest[i] = t;
          }
        }
      }
    }
  }

  DepthImage depth;
  depth.width = width;
  depth.height = height;
  depth.lens = lens;
  depth.millimetres.resize(rays.size());
  for (size_t i = 0; i < rays.size(); ++i) {
    float metres = nearest[i] * depth_per_metre[i];
    depth.millimetres[i] =
        std::isfinite(metres) && metres > 0.0f
            ? uint16_t(std::min(std::lround(metres * 1000.0f), 65535l))
            : 0;
  }
  return depth;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
