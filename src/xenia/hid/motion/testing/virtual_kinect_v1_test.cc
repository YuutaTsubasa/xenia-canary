/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "xenia/hid/motion/virtual_kinect_v1.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

DepthImage MakeDepth(uint32_t width, uint32_t height, uint16_t fill) {
  DepthImage depth;
  depth.width = width;
  depth.height = height;
  depth.millimetres.assign(size_t(width) * height, fill);
  return depth;
}

// What the title reads back: a big-endian value at (x, y).
uint16_t ReadBigEndian(const std::vector<uint8_t>& frame, uint32_t x,
                       uint32_t y) {
  size_t offset = (size_t(y) * kV1DepthWidth + x) * 2;
  return uint16_t(frame[offset] << 8 | frame[offset + 1]);
}

}  // namespace

TEST_CASE("V1 depth table maps raw values to themselves",
          "[motion][virtual_kinect_v1]") {
  std::array<uint16_t, kV1DepthTableEntries> table;
  table.fill(0xFFFF);
  FillV1DepthTable(table);
  REQUIRE(table[0] == 0);  // 0 stays "no reading"
  REQUIRE(table[1] == 1);
  REQUIRE(table[1500] == 1500);
  REQUIRE(table[3999] == 3999);
  REQUIRE(table[8191] == 8191);
}

TEST_CASE("V1 depth frame is big-endian millimetres in sensor order",
          "[motion][virtual_kinect_v1]") {
  DepthImage depth = MakeDepth(kV1DepthWidth, kV1DepthHeight, 0);
  depth.millimetres[0] = 1234;                       // top-left
  depth.millimetres[kV1DepthWidth - 1] = 2000;       // top-right
  depth.millimetres[kV1DepthWidth * 479 + 5] = 801;  // bottom row

  std::vector<uint8_t> frame(kV1DepthFrameBytes, 0xAA);
  REQUIRE(EncodeV1DepthFrame(depth, frame));

  REQUIRE(frame[0] == 0x04);  // 1234 = 0x04D2, high byte first
  REQUIRE(frame[1] == 0xD2);
  REQUIRE(ReadBigEndian(frame, 0, 0) == 1234);
  REQUIRE(ReadBigEndian(frame, 639, 0) == 2000);  // not mirrored
  REQUIRE(ReadBigEndian(frame, 5, 479) == 801);
  REQUIRE(ReadBigEndian(frame, 1, 0) == 0);  // no reading stays 0
}

TEST_CASE("V1 depth frame clamps far readings instead of making them invalid",
          "[motion][virtual_kinect_v1]") {
  DepthImage depth = MakeDepth(kV1DepthWidth, kV1DepthHeight, 9000);
  std::vector<uint8_t> frame(kV1DepthFrameBytes);
  REQUIRE(EncodeV1DepthFrame(depth, frame));
  REQUIRE(ReadBigEndian(frame, 320, 240) == kV1DepthMaxValue);
  REQUIRE(kV1DepthMaxValue < kV1DepthTableEntries);
}

TEST_CASE("V1 depth frame rejects the wrong size",
          "[motion][virtual_kinect_v1]") {
  std::vector<uint8_t> frame(kV1DepthFrameBytes, 0xAA);
  // Another camera's image must be reprojected first, not copied.
  REQUIRE_FALSE(EncodeV1DepthFrame(MakeDepth(512, 424, 1000), frame));
  REQUIRE(frame[0] == 0xAA);  // nothing written

  std::vector<uint8_t> short_frame(kV1DepthFrameBytes - 2);
  REQUIRE_FALSE(EncodeV1DepthFrame(
      MakeDepth(kV1DepthWidth, kV1DepthHeight, 1000), short_frame));
}

TEST_CASE("A tilted virtual v1 looks up or down", "[motion][virtual_kinect_v1]") {
  auto turn = [](const CameraPose& pose, std::array<float, 3> p) {
    const auto& r = pose.rotation;
    return std::array<float, 3>{r[0] * p[0] + r[1] * p[1] + r[2] * p[2],
                                r[3] * p[0] + r[4] * p[1] + r[5] * p[2],
                                r[6] * p[0] + r[7] * p[1] + r[8] * p[2]};
  };
  // Level: nothing moves.
  auto same = turn(V1TiltPose(0.0f), {0.1f, 0.2f, 1.0f});
  REQUIRE(same[1] == Approx(0.2f));
  REQUIRE(same[2] == Approx(1.0f));

  // 25 degrees below straight ahead (lens +y is down): straight ahead for a
  // v1 tilted 25 degrees down, and further down still for one tilted up.
  const float below = std::tan(25.0f * 3.14159265f / 180.0f);
  auto down = turn(V1TiltPose(-25.0f), {0.0f, below, 1.0f});
  REQUIRE(down[1] == Approx(0.0f).margin(1e-5));
  REQUIRE(down[2] > 0.0f);
  auto up = turn(V1TiltPose(10.0f), {0.0f, below, 1.0f});
  REQUIRE(up[1] / up[2] > below);
  // x is the axis it turns about.
  REQUIRE(turn(V1TiltPose(-25.0f), {0.3f, 0.0f, 1.0f})[0] == Approx(0.3f));
}

TEST_CASE("A colour frame is the picture through a GRBG Bayer filter",
          "[motion][virtual_kinect_v1]") {
  DepthImage depth = MakeDepth(kV1DepthWidth, kV1DepthHeight, 2000);
  depth.rgba.resize(depth.millimetres.size() * 4);
  for (size_t i = 0; i < depth.millimetres.size(); ++i) {
    depth.rgba[i * 4 + 0] = 200;  // red
    depth.rgba[i * 4 + 1] = 100;  // green
    depth.rgba[i * 4 + 2] = 50;   // blue
    depth.rgba[i * 4 + 3] = 255;
  }
  // One pixel the colour camera didn't see.
  depth.rgba[(size_t(11) * kV1DepthWidth + 7) * 4 + 3] = 0;

  std::vector<uint8_t> frame(kV1ColourFrameBytes, 0);
  REQUIRE(EncodeV1ColourFrame(depth, frame));
  auto at = [&](uint32_t x, uint32_t y) {
    return frame[size_t(y) * kV1ColourWidth + x];
  };
  REQUIRE(at(0, 0) == 100);  // G R
  REQUIRE(at(1, 0) == 200);
  REQUIRE(at(0, 1) == 50);   // B G
  REQUIRE(at(1, 1) == 100);
  REQUIRE(at(638, 478) == 100);
  REQUIRE(at(639, 479) == 100);
  REQUIRE(at(7, 11) == kV1ColourGrey);

  // Without colour, or at another size, nothing is written.
  std::vector<uint8_t> untouched(kV1ColourFrameBytes, 7);
  REQUIRE_FALSE(EncodeV1ColourFrame(
      MakeDepth(kV1DepthWidth, kV1DepthHeight, 2000), untouched));
  REQUIRE(untouched[0] == 7);
  std::vector<uint8_t> three_bytes(kV1ColourFrameBytes * 3);
  REQUIRE_FALSE(EncodeV1ColourFrame(depth, three_bytes));
}

TEST_CASE("A two-byte colour frame is the picture as UYVY",
          "[motion][virtual_kinect_v1]") {
  DepthImage depth = MakeDepth(kV1DepthWidth, kV1DepthHeight, 2000);
  depth.rgba.assign(depth.millimetres.size() * 4, 255);  // white
  // Pixels 0 and 1 of row 0: red and blue.
  const uint8_t red[] = {255, 0, 0, 255}, blue[] = {0, 0, 255, 255};
  std::copy_n(red, 4, &depth.rgba[0]);
  std::copy_n(blue, 4, &depth.rgba[4]);
  // Pixels 2 and 3 of row 5: no colour at all.
  depth.rgba[(size_t(5) * kV1DepthWidth + 2) * 4 + 3] = 0;
  depth.rgba[(size_t(5) * kV1DepthWidth + 3) * 4 + 3] = 0;

  std::vector<uint8_t> frame(kV1ColourFrameBytes * 2, 0);
  REQUIRE(EncodeV1ColourFrame(depth, frame));
  // U Y0 V Y1: the luma of each pixel, the chroma of the pair's average.
  REQUIRE(frame[1] == 76);   // red's luma
  REQUIRE(frame[3] == 29);   // blue's luma
  REQUIRE(frame[0] > 128);   // half blue: U above the middle
  REQUIRE(frame[2] > 128);   // half red: V above the middle
  // White is full luma, no chroma.
  size_t white = (size_t(1) * kV1ColourWidth + 0) * 2;
  REQUIRE(frame[white + 0] == 128);
  REQUIRE(frame[white + 1] == 255);
  REQUIRE(frame[white + 2] == 128);
  REQUIRE(frame[white + 3] == 255);
  size_t unseen = (size_t(5) * kV1ColourWidth + 2) * 2;
  for (int i = 0; i < 4; ++i) {
    REQUIRE(frame[unseen + i] == kV1ColourGrey);
  }
}

}  // namespace xe::hid::motion::test
