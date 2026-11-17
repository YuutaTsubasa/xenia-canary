/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_VIRTUAL_KINECT_V1_H_
#define XENIA_HID_MOTION_VIRTUAL_KINECT_V1_H_

#include <cstddef>
#include <cstdint>
#include <span>

#include "xenia/hid/motion/motion_types.h"

// The virtual Kinect v1: what an Xbox 360 Kinect camera hands to a title's
// NUI library through PsCamDeviceRequest, made from any motion device's
// frames. This file has the depth side; colour, calibration replies and
// reprojection from another lens come in later commits.
//
// What the title expects comes from our own tracing (docs/kinect-pscam.md
// §11): its tracker reads a 640x480 frame of big-endian 16-bit values in
// sensor order (not mirrored), and looks every value up in the table the
// driver returned for request code 24, treating the result as millimetres
// (0 = no reading, above 3999 = far, 8192 or more = invalid). So the
// virtual camera returns an identity table and writes millimetres directly.

namespace xe {
namespace hid {
namespace motion {

// The depth frame a request code 5 read (depth stream) receives.
constexpr uint32_t kV1DepthWidth = 640;
constexpr uint32_t kV1DepthHeight = 480;
constexpr size_t kV1DepthFrameBytes =
    size_t(kV1DepthWidth) * kV1DepthHeight * sizeof(uint16_t);  // 614400

// Request code 24's table: one entry per raw depth value 0..8191.
constexpr size_t kV1DepthTableEntries = 8192;

// The largest value written to a depth frame. The title takes anything
// above 3999 mm as "far", and treats 8192 or more as invalid, so farther
// readings are clamped here to stay "far" instead of turning invalid.
constexpr uint16_t kV1DepthMaxValue = uint16_t(kV1DepthTableEntries - 1);

// Fills request code 24's table (host byte order; the guest-facing code
// swaps it when storing): raw value i means i millimetres, and 0 stays
// "no reading".
void FillV1DepthTable(std::span<uint16_t, kV1DepthTableEntries> table);

// Writes `depth`, which must already be a 640x480 image seen through the
// virtual v1's own depth lens, as a code 5 depth frame: big-endian
// millimetres, top row first, in sensor order. Readings beyond
// kV1DepthMaxValue are clamped; 0 stays 0. Returns false (writing nothing)
// when `depth` is not 640x480 or `out` is not kV1DepthFrameBytes long.
bool EncodeV1DepthFrame(const DepthImage& depth, std::span<uint8_t> out);

}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_VIRTUAL_KINECT_V1_H_
