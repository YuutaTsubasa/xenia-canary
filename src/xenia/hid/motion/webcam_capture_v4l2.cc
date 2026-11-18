/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// A webcam on Linux, through V4L2 with memory-mapped buffers. Ported from
// Free Riders Recompiled (camera_capture_v4l2.cpp), written by its author
// YuutaTsubasa and brought here under this file's license.
//
// BGRA is asked for first (few cameras offer it), then YUYV, then NV12;
// webcam_pixels.cc converts them.

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <utility>

#include "xenia/base/logging.h"
#include "xenia/hid/motion/webcam_capture.h"

namespace xe {
namespace hid {
namespace motion {

namespace {

// How long Read waits for a picture: a little more than a frame at 15 fps.
constexpr int kReadTimeoutMs = 100;

int RetryIoctl(int descriptor, unsigned long request, void* argument) {
  int result;
  do {
    result = ioctl(descriptor, request, argument);
  } while (result == -1 && errno == EINTR);
  return result;
}

enum class Pixels { kBgra, kYuyv, kNv12 };

class V4L2Webcam final : public WebcamCapture {
 public:
  struct Buffer {
    void* start = nullptr;
    size_t length = 0;
  };

  V4L2Webcam(std::string name, int descriptor, std::vector<Buffer> buffers,
             Pixels pixels, uint32_t width, uint32_t height, uint32_t stride)
      : name_(std::move(name)),
        descriptor_(descriptor),
        buffers_(std::move(buffers)),
        pixels_(pixels),
        width_(width),
        height_(height),
        stride_(stride) {}

  ~V4L2Webcam() override {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    RetryIoctl(descriptor_, VIDIOC_STREAMOFF, &type);
    for (const Buffer& buffer : buffers_) {
      munmap(buffer.start, buffer.length);
    }
    close(descriptor_);
  }

  const std::string& name() const override { return name_; }

  bool Read(ColourImage& picture) override {
    pollfd ready = {descriptor_, POLLIN, 0};
    if (poll(&ready, 1, kReadTimeoutMs) <= 0) {
      return false;
    }
    v4l2_buffer taken = {};
    taken.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    taken.memory = V4L2_MEMORY_MMAP;
    if (RetryIoctl(descriptor_, VIDIOC_DQBUF, &taken) == -1) {
      return false;
    }
    bool converted = false;
    if (taken.index < buffers_.size()) {
      const Buffer& buffer = buffers_[taken.index];
      std::span<const uint8_t> data(
          static_cast<const uint8_t*>(buffer.start),
          taken.bytesused ? taken.bytesused : buffer.length);
      switch (pixels_) {
        case Pixels::kBgra:
          converted = BgrxToRgba(data, width_, height_, stride_, picture);
          break;
        case Pixels::kYuyv:
          converted = Yuy2ToRgba(data, width_, height_, stride_, picture);
          break;
        case Pixels::kNv12:
          converted = Nv12ToRgba(data, width_, height_, stride_, picture);
          break;
      }
    }
    RetryIoctl(descriptor_, VIDIOC_QBUF, &taken);
    return converted;
  }

 private:
  std::string name_;
  int descriptor_;
  std::vector<Buffer> buffers_;
  Pixels pixels_;
  uint32_t width_, height_, stride_;
};

// /dev/video0 upwards, keeping the nodes that can capture: a driver's
// numbers are not always consecutive, and some nodes are for metadata, not
// pictures. The path is part of the name, so a player can name a node.
std::vector<std::pair<std::string, std::string>> CaptureNodes() {
  std::vector<std::pair<std::string, std::string>> found;
  for (int index = 0; index < 16; ++index) {
    std::string path = "/dev/video" + std::to_string(index);
    int descriptor = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
    if (descriptor == -1) {
      continue;
    }
    v4l2_capability capability = {};
    if (RetryIoctl(descriptor, VIDIOC_QUERYCAP, &capability) == 0 &&
        (capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
        (capability.capabilities & V4L2_CAP_STREAMING)) {
      found.emplace_back(
          path,
          path + " (" + reinterpret_cast<const char*>(capability.card) + ")");
    }
    close(descriptor);
  }
  return found;
}

}  // namespace

std::vector<std::string> WebcamCapture::List() {
  std::vector<std::string> names;
  for (const auto& node : CaptureNodes()) {
    names.push_back(node.second);
  }
  return names;
}

std::unique_ptr<WebcamCapture> WebcamCapture::Open(const std::string& wanted,
                                                   uint32_t width,
                                                   uint32_t height,
                                                   std::string& reason) {
  // Looked up again rather than trusted from an earlier List(): unplugging
  // a camera renumbers the rest.
  auto nodes = CaptureNodes();
  if (nodes.empty()) {
    reason = "No webcam was found";
    return nullptr;
  }
  std::vector<std::string> names;
  for (const auto& node : nodes) {
    names.push_back(node.second);
  }
  int chosen = ChooseWebcam(names, wanted);
  if (chosen < 0) {
    reason = "No webcam is called \"" + wanted + "\"";
    return nullptr;
  }
  const std::string& path = nodes[chosen].first;
  const std::string& name = nodes[chosen].second;
  int descriptor = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
  if (descriptor == -1) {
    reason = "The webcam " + name + " would not open";
    return nullptr;
  }
  std::vector<V4L2Webcam::Buffer> buffers;
  auto give_up = [&](const char* why) -> std::unique_ptr<WebcamCapture> {
    reason = "The webcam " + name + ": " + why;
    for (const auto& buffer : buffers) {
      munmap(buffer.start, buffer.length);
    }
    close(descriptor);
    return nullptr;
  };

  Pixels pixels = Pixels::kBgra;
  v4l2_format format = {};
  bool settled = false;
  for (auto [fourcc, kind] : {std::pair{V4L2_PIX_FMT_ABGR32, Pixels::kBgra},
                              std::pair{V4L2_PIX_FMT_YUYV, Pixels::kYuyv},
                              std::pair{V4L2_PIX_FMT_NV12, Pixels::kNv12}}) {
    format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = fourcc;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (RetryIoctl(descriptor, VIDIOC_S_FMT, &format) == -1 ||
        format.fmt.pix.pixelformat != fourcc) {
      continue;  // refused, or the driver chose another
    }
    pixels = kind;
    settled = true;
    break;
  }
  if (!settled) {
    return give_up("it sends no picture format Xenia reads");
  }

  v4l2_requestbuffers request = {};
  request.count = 4;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  if (RetryIoctl(descriptor, VIDIOC_REQBUFS, &request) == -1 ||
      request.count < 2) {
    return give_up("it gave no buffers");
  }
  for (uint32_t i = 0; i < request.count; ++i) {
    v4l2_buffer description = {};
    description.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    description.memory = V4L2_MEMORY_MMAP;
    description.index = i;
    if (RetryIoctl(descriptor, VIDIOC_QUERYBUF, &description) == -1) {
      return give_up("a buffer could not be queried");
    }
    void* start = mmap(nullptr, description.length, PROT_READ | PROT_WRITE,
                       MAP_SHARED, descriptor, description.m.offset);
    if (start == MAP_FAILED) {
      return give_up("a buffer could not be mapped");
    }
    buffers.push_back({start, description.length});
    if (RetryIoctl(descriptor, VIDIOC_QBUF, &description) == -1) {
      return give_up("a buffer could not be queued");
    }
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (RetryIoctl(descriptor, VIDIOC_STREAMON, &type) == -1) {
    return give_up("it would not start streaming");
  }
  XELOGI("Motion capture: webcam {} open at {}x{}", name, format.fmt.pix.width,
         format.fmt.pix.height);
  return std::make_unique<V4L2Webcam>(
      name, descriptor, std::move(buffers), pixels, format.fmt.pix.width,
      format.fmt.pix.height, format.fmt.pix.bytesperline);
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
