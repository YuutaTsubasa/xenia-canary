/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/pose_skeleton.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace xe {
namespace hid {
namespace motion {

namespace {

using Point = std::array<float, 3>;

Point Middle(const Point& a, const Point& b) {
  return {(a[0] + b[0]) * 0.5f, (a[1] + b[1]) * 0.5f, (a[2] + b[2]) * 0.5f};
}

Point Between(const Point& from, const Point& to, float part) {
  return {from[0] + (to[0] - from[0]) * part,
          from[1] + (to[1] - from[1]) * part,
          from[2] + (to[2] - from[2]) * part};
}

JointState LeastSure(JointState a, JointState b) {
  return uint8_t(a) < uint8_t(b) ? a : b;
}

}  // namespace

void PosePointsToJoints(
    const std::array<Point, pose_point::kCount>& points,
    const std::array<JointState, pose_point::kCount>& states, Body& body) {
  auto& joints = body.joints;
  auto& joint_states = body.joint_states;
  auto set = [&](Joint to, uint32_t from) {
    joints[size_t(to)] = points[from];
    joint_states[size_t(to)] = states[from];
  };
  set(Joint::kShoulderRight, pose_point::kShoulderRight);
  set(Joint::kShoulderLeft, pose_point::kShoulderLeft);
  set(Joint::kElbowRight, pose_point::kElbowRight);
  set(Joint::kElbowLeft, pose_point::kElbowLeft);
  set(Joint::kHandRight, pose_point::kWristRight);
  set(Joint::kHandLeft, pose_point::kWristLeft);
  set(Joint::kHipRight, pose_point::kHipRight);
  set(Joint::kHipLeft, pose_point::kHipLeft);
  set(Joint::kKneeRight, pose_point::kKneeRight);
  set(Joint::kKneeLeft, pose_point::kKneeLeft);
  set(Joint::kAnkleRight, pose_point::kAnkleRight);
  set(Joint::kAnkleLeft, pose_point::kAnkleLeft);
  set(Joint::kHead, pose_point::kNose);

  auto at = [&](Joint joint) -> const Point& { return joints[size_t(joint)]; };
  auto made = [&](Joint to, const Point& place, Joint a, Joint b) {
    joints[size_t(to)] = place;
    joint_states[size_t(to)] =
        LeastSure(joint_states[size_t(a)], joint_states[size_t(b)]);
  };
  made(Joint::kWristRight,
       Between(at(Joint::kElbowRight), at(Joint::kHandRight), 0.9f),
       Joint::kElbowRight, Joint::kHandRight);
  made(Joint::kWristLeft,
       Between(at(Joint::kElbowLeft), at(Joint::kHandLeft), 0.9f),
       Joint::kElbowLeft, Joint::kHandLeft);
  // A foot a little lower than its ankle and towards the sensor.
  auto foot = [](const Point& ankle) {
    return Point{ankle[0], ankle[1] - 0.05f, ankle[2] - 0.08f};
  };
  made(Joint::kFootRight, foot(at(Joint::kAnkleRight)), Joint::kAnkleRight,
       Joint::kAnkleRight);
  made(Joint::kFootLeft, foot(at(Joint::kAnkleLeft)), Joint::kAnkleLeft,
       Joint::kAnkleLeft);
  made(Joint::kHipCenter, Middle(at(Joint::kHipLeft), at(Joint::kHipRight)),
       Joint::kHipLeft, Joint::kHipRight);
  made(Joint::kShoulderCenter,
       Middle(at(Joint::kShoulderLeft), at(Joint::kShoulderRight)),
       Joint::kShoulderLeft, Joint::kShoulderRight);
  made(Joint::kSpine,
       Between(at(Joint::kHipCenter), at(Joint::kShoulderCenter), 0.5f),
       Joint::kHipCenter, Joint::kShoulderCenter);
}

bool PoseToBody(const PoseLandmarks& given, uint32_t picture_width,
                uint32_t picture_height, bool picture_is_mirrored, Body& body) {
  using namespace pose_point;
  if (!picture_width || !picture_height) {
    return false;
  }
  PoseLandmarks landmarks = given;
  const bool has_world =
      std::any_of(landmarks.begin(), landmarks.end(),
                  [](const PoseLandmark& point) { return point.has_world; });
  if (has_world) {
    // A 3D model gives all its points together. A partial or broken pose
    // must not fall back to a flat body or replace a good one.
    for (const auto& point : landmarks) {
      if (!point.has_world || !std::isfinite(point.score)) {
        return false;
      }
      for (float axis : point.world) {
        if (!std::isfinite(axis) || std::fabs(axis) > 10.0f) {
          return false;
        }
      }
    }
  }
  if (picture_is_mirrored) {
    for (auto& point : landmarks) {
      point.x = float(picture_width) - point.x;
      if (has_world) {
        point.world[0] = -point.world[0];
      }
    }
    for (const auto& [left, right] : {std::pair{kShoulderLeft, kShoulderRight},
                                      {kElbowLeft, kElbowRight},
                                      {kWristLeft, kWristRight},
                                      {kHipLeft, kHipRight},
                                      {kKneeLeft, kKneeRight},
                                      {kAnkleLeft, kAnkleRight},
                                      {kEyeLeft, kEyeRight},
                                      {kEarLeft, kEarRight}}) {
      std::swap(landmarks[left], landmarks[right]);
    }
  }

  // The torso decides whether there is a body at all: limbs at the edge of
  // the picture are often guessed, these four are not.
  const float torso = std::min(
      std::min(landmarks[kShoulderLeft].score, landmarks[kShoulderRight].score),
      std::min(landmarks[kHipLeft].score, landmarks[kHipRight].score));
  if (!(torso > 0.0f)) {
    return false;
  }
  float shoulder_span =
      std::fabs(landmarks[kShoulderLeft].x - landmarks[kShoulderRight].x);
  if (has_world) {
    const auto& left = landmarks[kShoulderLeft].world;
    const auto& right = landmarks[kShoulderRight].world;
    // All three axes, so a body turned sideways is not scaled up.
    shoulder_span =
        std::hypot(left[0] - right[0], left[1] - right[1], left[2] - right[2]);
    if (shoulder_span < 0.05f || shoulder_span > 2.0f) {
      return false;
    }
  } else if (shoulder_span < 1.0f) {
    return false;  // too small to scale by
  }

  // One scale makes the shoulders the standard width (metres per model
  // metre, or per picture pixel). The camera faces the player, so the
  // picture's x and y run the other way to skeleton space.
  const float scale = kPoseShoulderHalfWidth * 2.0f / shoulder_span;
  const Point centre =
      has_world
          ? Middle(landmarks[kHipLeft].world, landmarks[kHipRight].world)
          : Point{(landmarks[kShoulderLeft].x + landmarks[kShoulderRight].x +
                   landmarks[kHipLeft].x + landmarks[kHipRight].x) *
                      0.25f,
                  (landmarks[kHipLeft].y + landmarks[kHipRight].y) * 0.5f,
                  0.0f};
  auto place = [&](uint32_t point) -> Point {
    if (has_world) {
      const auto& world = landmarks[point].world;
      return {(centre[0] - world[0]) * scale, (centre[1] - world[1]) * scale,
              kPoseDistance + (world[2] - centre[2]) * scale};
    }
    return {(centre[0] - landmarks[point].x) * scale,
            (centre[1] - landmarks[point].y) * scale, kPoseDistance};
  };

  // The model names sides from the body, so they carry straight across:
  // the player's right is skeleton +x, which place() has put on the
  // picture's left, where a facing camera sees it. A webcam's points are
  // all taken as tracked.
  std::array<Point, kCount> points;
  std::array<JointState, kCount> states;
  for (uint32_t point = 0; point < kCount; ++point) {
    points[point] = place(point);
    states[point] = JointState::kTracked;
  }
  PosePointsToJoints(points, states, body);
  return true;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
