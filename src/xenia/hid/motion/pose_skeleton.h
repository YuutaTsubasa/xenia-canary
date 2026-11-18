/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_POSE_SKELETON_H_
#define XENIA_HID_MOTION_POSE_SKELETON_H_

#include <array>
#include <cstdint>

#include "xenia/hid/motion/motion_types.h"
#include "xenia/hid/motion/pose_landmarks.h"

// A pose model's points as the twenty joints of a Kinect v1 skeleton.
// Ported from Free Riders Recompiled (pose_skeleton.cpp, pose_nui_joints.h),
// written by its author YuutaTsubasa and brought here under this file's
// license.

namespace xe {
namespace hid {
namespace motion {

// Where the body is put, and how big: the hips kPoseDistance metres in front
// of the sensor, the shoulders 2 * kPoseShoulderHalfWidth metres apart. A
// webcam cannot measure either, so every player comes out the same size at
// the same distance.
constexpr float kPoseDistance = 2.5f;
constexpr float kPoseShoulderHalfWidth = 0.18f;

// The twenty joints from the seventeen points, once each point is in
// skeleton space (metres, +x to the player's right, +y up, +z away from the
// sensor). The model's points carry across by name (its wrists become the
// hands, its nose the head); the joints it lacks are made from them: the
// wrists most of the way down the forearms, the feet just past the ankles,
// the spine between the hips and the shoulders. A made joint is as sure as
// the least sure point it is made from.
void PosePointsToJoints(
    const std::array<std::array<float, 3>, pose_point::kCount>& points,
    const std::array<JointState, pose_point::kCount>& states, Body& body);

// The skeleton of the body in one picture. False, leaving `body` alone,
// when the torso (shoulders and hips) has no confidence, when the shoulders
// are too close together to scale by, or when a 3D model's points are
// incomplete or implausible (not finite, more than ten metres out, or
// shoulders not 0.05 to 2 metres apart).
//
// With 3D points (has_world), they keep their relative depth; one scale
// sets the shoulder span. Without them, every joint stands at kPoseDistance
// and the picture's shoulder width sets the scale.
//
// A camera faces the player, so the player's right hand is on the picture's
// left. Some cameras send a mirrored picture instead, and nothing in the
// picture says which: a mirrored person looks like a person. So the caller
// says (`picture_is_mirrored`), and a mirrored picture is turned back before
// anything is measured (the model names a mirrored person's right hand
// their left, so the sides are swapped too).
//
// The hips always land at (0, 0, kPoseDistance): where the player stands in
// the room is the caller's to add.
bool PoseToBody(const PoseLandmarks& landmarks, uint32_t picture_width,
                uint32_t picture_height, bool picture_is_mirrored, Body& body);

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_POSE_SKELETON_H_
