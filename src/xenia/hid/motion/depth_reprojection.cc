/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/depth_reprojection.h"

#include <algorithm>
#include <cmath>

namespace xe {
namespace hid {
namespace motion {

namespace {

// Brown-Conrady: where an ideal ray (x, y) ends up through the lens.
std::array<float, 2> Distort(const CameraIntrinsics& lens, float x, float y) {
  float r2 = x * x + y * y;
  float radial = 1.0f + r2 * (lens.k1 + r2 * (lens.k2 + r2 * lens.k3));
  float xd =
      x * radial + 2.0f * lens.p1 * x * y + lens.p2 * (r2 + 2.0f * x * x);
  float yd =
      y * radial + lens.p1 * (r2 + 2.0f * y * y) + 2.0f * lens.p2 * x * y;
  return {xd, yd};
}

bool HasDistortion(const CameraIntrinsics& lens) {
  return lens.k1 != 0.0f || lens.k2 != 0.0f || lens.k3 != 0.0f ||
         lens.p1 != 0.0f || lens.p2 != 0.0f;
}

}  // namespace

std::array<float, 2> PixelToRay(const CameraIntrinsics& lens, float u,
                                float v) {
  float xd = (u - lens.centre_x) / lens.focal_x;
  float yd = (v - lens.centre_y) / lens.focal_y;
  if (!HasDistortion(lens)) {
    return {xd, yd};
  }
  // Find the ideal ray whose distorted image is (xd, yd). Each round takes
  // the tangential shift off and divides by the radial scale, both
  // evaluated at the current guess; this converges for the distortion of
  // real depth cameras even in the corners, where simply subtracting the
  // error overshoots.
  float x = xd, y = yd;
  for (int i = 0; i < 20; ++i) {
    float r2 = x * x + y * y;
    float radial = 1.0f + r2 * (lens.k1 + r2 * (lens.k2 + r2 * lens.k3));
    float dx = 2.0f * lens.p1 * x * y + lens.p2 * (r2 + 2.0f * x * x);
    float dy = lens.p1 * (r2 + 2.0f * y * y) + 2.0f * lens.p2 * x * y;
    x = (xd - dx) / radial;
    y = (yd - dy) / radial;
  }
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return {xd, yd};  // a lens this strong is out of the model's range
  }
  return {x, y};
}

std::array<float, 2> RayToPixel(const CameraIntrinsics& lens, float x,
                                float y) {
  auto d =
      HasDistortion(lens) ? Distort(lens, x, y) : std::array<float, 2>{x, y};
  return {d[0] * lens.focal_x + lens.centre_x,
          d[1] * lens.focal_y + lens.centre_y};
}

DepthImage ReprojectDepth(const DepthImage& source,
                          const CameraIntrinsics& target_lens,
                          uint32_t target_width, uint32_t target_height,
                          const CameraPose& pose) {
  DepthImage target;
  target.width = target_width;
  target.height = target_height;
  target.lens = target_lens;
  target.millimetres.assign(size_t(target_width) * target_height, 0);

  const size_t pixel_count = size_t(source.width) * source.height;
  if (source.millimetres.size() != pixel_count || !target_lens.valid()) {
    return target;
  }
  const bool have_rays = source.rays.size() == pixel_count * 2;
  if (!have_rays && !source.lens.valid()) {
    return target;
  }
  // How many target pixels one source pixel spans at equal depth; scaled by
  // the depth change below.
  const float source_focal =
      have_rays ? 0.0f : 0.5f * (source.lens.focal_x + source.lens.focal_y);
  const auto& r = pose.rotation;
  const auto& t = pose.translation;

  for (uint32_t sy = 0; sy < source.height; ++sy) {
    for (uint32_t sx = 0; sx < source.width; ++sx) {
      size_t index = size_t(sy) * source.width + sx;
      uint16_t mm = source.millimetres[index];
      if (!mm) {
        continue;
      }
      float ray_x, ray_y;
      if (have_rays) {
        ray_x = source.rays[index * 2];
        ray_y = source.rays[index * 2 + 1];
      } else {
        auto ray = PixelToRay(source.lens, sx + 0.5f, sy + 0.5f);
        ray_x = ray[0];
        ray_y = ray[1];
      }
      float z = mm * 0.001f;
      float px = ray_x * z, py = ray_y * z, pz = z;
      float qx = r[0] * px + r[1] * py + r[2] * pz + t[0];
      float qy = r[3] * px + r[4] * py + r[5] * pz + t[1];
      float qz = r[6] * px + r[7] * py + r[8] * pz + t[2];
      if (qz <= 0.0f) {
        continue;  // behind the target camera
      }
      float u = target_lens.focal_x * qx / qz + target_lens.centre_x;
      float v = target_lens.focal_y * qy / qz + target_lens.centre_y;

      // The source pixel's size in target pixels. With a ray table the
      // source lens is unknown, so neighbouring rays give its width.
      float span;
      if (have_rays) {
        size_t next = sx + 1 < source.width ? index + 1 : index - 1;
        float ray_step = std::abs(source.rays[next * 2] - ray_x);
        span = ray_step * z * target_lens.focal_x / qz;
      } else {
        span = (target_lens.focal_x / source_focal) * (z / qz);
      }
      int size = std::clamp(int(std::ceil(span)), 1, 3);

      uint32_t out_mm = uint32_t(std::lround(qz * 1000.0f));
      if (!out_mm || out_mm > 0xFFFF) {
        continue;
      }
      // The footprint is centred on (u, v).
      int x0 = int(std::floor(u - size * 0.5f + 0.5f));
      int y0 = int(std::floor(v - size * 0.5f + 0.5f));
      for (int dy = 0; dy < size; ++dy) {
        int ty = y0 + dy;
        if (ty < 0 || ty >= int(target_height)) {
          continue;
        }
        for (int dx = 0; dx < size; ++dx) {
          int tx = x0 + dx;
          if (tx < 0 || tx >= int(target_width)) {
            continue;
          }
          uint16_t& cell = target.millimetres[size_t(ty) * target_width + tx];
          if (!cell || out_mm < cell) {
            cell = uint16_t(out_mm);  // the nearest point wins
          }
        }
      }
    }
  }
  return target;
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
