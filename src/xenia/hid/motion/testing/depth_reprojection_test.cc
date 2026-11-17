/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <cmath>
#include <cstdlib>

#include "xenia/hid/motion/depth_reprojection.h"
#include "xenia/hid/motion/virtual_kinect_v1.h"

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

// A wide depth camera shaped like a Kinect v2's (512x424, about 70 degrees
// wide). The values are typical, not any real unit's calibration.
CameraIntrinsics WideLens() {
  CameraIntrinsics lens;
  lens.focal_x = 365.0f;
  lens.focal_y = 365.0f;
  lens.centre_x = 256.0f;
  lens.centre_y = 212.0f;
  return lens;
}

// What the wide camera sees of a flat wall `wall_mm` away, square to it,
// with a box `box_mm` away covering the pixels in [x0, x1) x [y0, y1).
DepthImage DrawScene(uint16_t wall_mm, uint16_t box_mm = 0, uint32_t x0 = 0,
                     uint32_t x1 = 0, uint32_t y0 = 0, uint32_t y1 = 0) {
  DepthImage depth;
  depth.width = 512;
  depth.height = 424;
  depth.lens = WideLens();
  depth.millimetres.assign(size_t(512) * 424, wall_mm);
  for (uint32_t y = y0; y < y1; ++y) {
    for (uint32_t x = x0; x < x1; ++x) {
      depth.millimetres[size_t(y) * 512 + x] = box_mm;
    }
  }
  return depth;
}

uint16_t At(const DepthImage& image, uint32_t x, uint32_t y) {
  return image.millimetres[size_t(y) * image.width + x];
}

}  // namespace

TEST_CASE("Pixel and ray conversions are inverses",
          "[motion][depth_reprojection]") {
  CameraIntrinsics lens = WideLens();
  lens.k1 = 0.09f;  // a mild barrel distortion, as on depth cameras
  lens.k2 = -0.27f;
  lens.k3 = 0.09f;  // without it k2 folds the corners back onto the image
  lens.p1 = 0.001f;
  for (float u : {10.5f, 256.0f, 400.25f, 501.5f}) {
    for (float v : {5.5f, 212.0f, 380.75f}) {
      auto ray = PixelToRay(lens, u, v);
      auto pixel = RayToPixel(lens, ray[0], ray[1]);
      REQUIRE(pixel[0] == Approx(u).margin(0.01f));
      REQUIRE(pixel[1] == Approx(v).margin(0.01f));
    }
  }
  // Without distortion the centre is straight ahead.
  auto centre = PixelToRay(WideLens(), 256.0f, 212.0f);
  REQUIRE(centre[0] == Approx(0.0f).margin(1e-6f));
  REQUIRE(centre[1] == Approx(0.0f).margin(1e-6f));
}

TEST_CASE("A wall stays at its distance and leaves no holes",
          "[motion][depth_reprojection]") {
  DepthImage v1 = ReprojectToV1Depth(DrawScene(2000));
  REQUIRE(v1.width == kV1DepthWidth);
  REQUIRE(v1.height == kV1DepthHeight);
  // The v1's view (58.5 degrees) is inside the wide camera's (70), so the
  // wall fills it, every pixel, at 2 m.
  for (uint32_t y = 0; y < kV1DepthHeight; y += 7) {
    for (uint32_t x = 0; x < kV1DepthWidth; x += 7) {
      REQUIRE(std::abs(int(At(v1, x, y)) - 2000) <= 1);
    }
  }
}

TEST_CASE("Moving the target camera changes the depth it sees",
          "[motion][depth_reprojection]") {
  // The v1 standing 0.3 m behind the source: the wall is 0.3 m farther.
  CameraPose pose;
  pose.translation = {0.0f, 0.0f, 0.3f};
  DepthImage v1 = ReprojectToV1Depth(DrawScene(2000), pose);
  REQUIRE(std::abs(int(At(v1, 320, 240)) - 2300) <= 1);
}

TEST_CASE("The nearer surface wins where two land on one pixel",
          "[motion][depth_reprojection]") {
  // A box 1 m away in the middle of the wide picture, wall at 3 m.
  DepthImage v1 = ReprojectToV1Depth(DrawScene(3000, 1000, 236, 276, 192, 232));
  REQUIRE(At(v1, 320, 240) == 1000);  // the box, in the v1's centre
  REQUIRE(At(v1, 20, 20) == 3000);    // the wall, at the edge
  // The box keeps its angular size: 40 wide pixels at f=365 are about
  // 62 v1 pixels at f=571.26.
  uint32_t box_pixels = 0;
  for (uint32_t x = 0; x < kV1DepthWidth; ++x) {
    box_pixels += At(v1, x, 240) == 1000;
  }
  REQUIRE(box_pixels >= 60);
  REQUIRE(box_pixels <= 65);
}

TEST_CASE("Points behind the target camera are dropped",
          "[motion][depth_reprojection]") {
  CameraPose pose;
  pose.translation = {0.0f, 0.0f, -2.5f};  // the v1 2.5 m in front
  DepthImage v1 = ReprojectToV1Depth(DrawScene(2000), pose);
  for (uint16_t mm : v1.millimetres) {
    REQUIRE(mm == 0);
  }
}

TEST_CASE("A ray table gives the same picture as its lens",
          "[motion][depth_reprojection]") {
  DepthImage with_lens = DrawScene(2500, 1500, 200, 300, 150, 250);
  DepthImage with_rays = with_lens;
  with_rays.lens = {};  // only the rays are known
  with_rays.rays.resize(size_t(512) * 424 * 2);
  for (uint32_t y = 0; y < 424; ++y) {
    for (uint32_t x = 0; x < 512; ++x) {
      auto ray = PixelToRay(WideLens(), x + 0.5f, y + 0.5f);
      with_rays.rays[(size_t(y) * 512 + x) * 2] = ray[0];
      with_rays.rays[(size_t(y) * 512 + x) * 2 + 1] = ray[1];
    }
  }
  DepthImage a = ReprojectToV1Depth(with_lens);
  DepthImage b = ReprojectToV1Depth(with_rays);
  size_t differing = 0;
  for (size_t i = 0; i < a.millimetres.size(); ++i) {
    differing += a.millimetres[i] != b.millimetres[i];
  }
  // Footprints may differ by a pixel at the box's edges only.
  REQUIRE(differing < a.millimetres.size() / 200);
}

TEST_CASE("Pixels without a ray are left out", "[motion][depth_reprojection]") {
  DepthImage depth = DrawScene(2000);
  depth.lens = {};
  depth.rays.resize(size_t(512) * 424 * 2);
  for (uint32_t y = 0; y < 424; ++y) {
    for (uint32_t x = 0; x < 512; ++x) {
      auto ray = PixelToRay(WideLens(), x + 0.5f, y + 0.5f);
      // The left half has no rays, as a device's table may have at its edges.
      float nan = std::nanf("");
      depth.rays[(size_t(y) * 512 + x) * 2] = x < 256 ? nan : ray[0];
      depth.rays[(size_t(y) * 512 + x) * 2 + 1] = x < 256 ? nan : ray[1];
    }
  }
  DepthImage v1 = ReprojectToV1Depth(depth);
  for (uint16_t mm : v1.millimetres) {
    REQUIRE((mm == 0 || (mm > 1990 && mm < 2010)));
  }
  REQUIRE(At(v1, 160, 240) == 0);
  REQUIRE(At(v1, 480, 240) != 0);
}

}  // namespace xe::hid::motion::test
