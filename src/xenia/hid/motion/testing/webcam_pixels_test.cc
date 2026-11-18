/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <cstdlib>

#include "xenia/base/platform.h"
#include "xenia/hid/motion/webcam_capture.h"

#if XE_PLATFORM_WIN32
#include "xenia/base/platform_win.h"
#endif

#include "third_party/catch/include/catch.hpp"

namespace xe::hid::motion::test {

namespace {

bool Near(const ColourImage& picture, size_t pixel, int r, int g, int b) {
  const uint8_t* p = picture.rgba.data() + pixel * 4;
  return std::abs(p[0] - r) <= 2 && std::abs(p[1] - g) <= 2 &&
         std::abs(p[2] - b) <= 2 && p[3] == 255;
}

}  // namespace

TEST_CASE("BGRX rows with padding become RGBA", "[motion][webcam]") {
  // 2x2, rows 12 bytes apart (4 bytes of padding).
  std::vector<uint8_t> bgrx = {1, 2, 3, 0, 4, 5, 6,  0,  9,  9,
                               9, 9, 7, 8, 9, 0, 10, 11, 12, 0};
  ColourImage picture;
  REQUIRE(BgrxToRgba(bgrx, 2, 2, 12, picture));
  REQUIRE(picture.width == 2);
  REQUIRE(picture.rgba == std::vector<uint8_t>{3, 2, 1, 255, 6, 5, 4, 255, 9, 8,
                                               7, 255, 12, 11, 10, 255});
  // Too short for the stride.
  bgrx.pop_back();
  REQUIRE_FALSE(BgrxToRgba(bgrx, 2, 2, 12, picture));
}

TEST_CASE("YUY2 becomes RGBA with BT.601 studio range", "[motion][webcam]") {
  // Studio black, studio white; then pure red and pure blue as BT.601
  // gives them (Y U V = 81 90 240 and 41 240 110).
  std::vector<uint8_t> yuy2 = {16, 128, 235, 128, 81, 90,
                               81, 240, 41,  240, 41, 110};
  ColourImage picture;
  REQUIRE(Yuy2ToRgba(yuy2, 2, 3, 4, picture));
  REQUIRE(Near(picture, 0, 0, 0, 0));
  REQUIRE(Near(picture, 1, 255, 255, 255));
  REQUIRE(Near(picture, 2, 255, 0, 0));
  REQUIRE(Near(picture, 4, 0, 0, 255));
  REQUIRE_FALSE(Yuy2ToRgba(yuy2, 3, 1, 6, picture));  // odd width
}

TEST_CASE("NV12 shares each chroma pair between four pixels",
          "[motion][webcam]") {
  // 2x2 luma (white, black / black, white), then one U V pair: neutral.
  std::vector<uint8_t> nv12 = {235, 16, 16, 235, 128, 128};
  ColourImage picture;
  REQUIRE(Nv12ToRgba(nv12, 2, 2, 2, picture));
  REQUIRE(Near(picture, 0, 255, 255, 255));
  REQUIRE(Near(picture, 1, 0, 0, 0));
  REQUIRE(Near(picture, 3, 255, 255, 255));
  nv12.pop_back();
  REQUIRE_FALSE(Nv12ToRgba(nv12, 2, 2, 2, picture));
}

TEST_CASE("A webcam is picked by part of its name", "[motion][webcam]") {
  std::vector<std::string> names = {"Integrated Camera", "Logitech C920"};
  REQUIRE(ChooseWebcam(names, "") == 0);
  REQUIRE(ChooseWebcam(names, "c920") == 1);
  REQUIRE(ChooseWebcam(names, "LOGI") == 1);
  REQUIRE(ChooseWebcam(names, "Kinect") == -1);
  REQUIRE(ChooseWebcam({}, "") == -1);
}

// Opens a real webcam only when XENIA_TEST_WEBCAM names one (or is "*" for
// the first): a test must not switch on someone's camera unasked.
TEST_CASE("A real webcam gives a picture", "[motion][webcam]") {
  const char* wanted = std::getenv("XENIA_TEST_WEBCAM");
  if (!wanted || !*wanted) {
    SUCCEED("XENIA_TEST_WEBCAM not set");
    return;
  }
#if XE_PLATFORM_WIN32
  HRESULT com = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
#endif
  {
    std::string reason;
    auto webcam = WebcamCapture::Open(std::string(wanted) == "*" ? "" : wanted,
                                      640, 480, reason);
    INFO(reason);
    REQUIRE(webcam);
    ColourImage picture;
    bool got = false;
    for (int tries = 0; tries < 60 && !got; ++tries) {
      got = webcam->Read(picture);
    }
    REQUIRE(got);
    REQUIRE(picture.rgba.size() == size_t(picture.width) * picture.height * 4);
  }
#if XE_PLATFORM_WIN32
  if (SUCCEEDED(com)) {
    CoUninitialize();
  }
#endif
}

}  // namespace xe::hid::motion::test
