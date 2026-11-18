/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

// A webcam through Media Foundation's source reader, written from
// Microsoft's documentation ("Enumerating Video Capture Devices", "Using the
// Source Reader to Process Media Data"). The reader is asked for RGB32 and
// converts (and decodes MJPEG) itself.
//
// Media Foundation's functions are looked up at run time: Windows N
// editions have no Media Foundation unless the Media Feature Pack is
// installed, and Xenia must still start there. Only GUIDs come from a
// static library (mfuuid).

#include "xenia/hid/motion/webcam_capture.h"

#include <cmath>

#include "xenia/base/logging.h"
#include "xenia/base/platform_win.h"
#include "xenia/base/string.h"

// After platform_win.h, which sets up windows.h (NOMINMAX) for Xenia.
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <wrl/client.h>

namespace xe {
namespace hid {
namespace motion {

namespace {

using Microsoft::WRL::ComPtr;

// The Media Foundation functions used, found at run time.
struct MediaFoundation {
  HRESULT(WINAPI* startup)(ULONG, DWORD) = nullptr;
  HRESULT(WINAPI* shutdown)() = nullptr;
  HRESULT(WINAPI* create_attributes)(IMFAttributes**, UINT32) = nullptr;
  HRESULT(WINAPI* create_media_type)(IMFMediaType**) = nullptr;
  HRESULT(WINAPI* enum_device_sources)
  (IMFAttributes*, IMFActivate***, UINT32*) = nullptr;
  HRESULT(WINAPI* create_source_reader)
  (IMFMediaSource*, IMFAttributes*, IMFSourceReader**) = nullptr;

  // Loads the libraries (they stay loaded) and starts Media Foundation for
  // this caller; Stop() once per successful Start().
  bool Start() {
    HMODULE plat = LoadLibraryW(L"mfplat.dll");
    HMODULE mf = LoadLibraryW(L"mf.dll");
    HMODULE readwrite = LoadLibraryW(L"mfreadwrite.dll");
    if (!plat || !mf || !readwrite) {
      return false;
    }
    auto find = [](HMODULE library, auto& function, const char* name) {
      function = reinterpret_cast<std::remove_reference_t<decltype(function)>>(
          GetProcAddress(library, name));
      return function != nullptr;
    };
    return find(plat, startup, "MFStartup") &&
           find(plat, shutdown, "MFShutdown") &&
           find(plat, create_attributes, "MFCreateAttributes") &&
           find(plat, create_media_type, "MFCreateMediaType") &&
           find(mf, enum_device_sources, "MFEnumDeviceSources") &&
           find(readwrite, create_source_reader,
                "MFCreateSourceReaderFromMediaSource") &&
           SUCCEEDED(startup(MF_VERSION, MFSTARTUP_NOSOCKET));
  }
  void Stop() { shutdown(); }
};

// The video capture devices there are, with their names.
struct Devices {
  IMFActivate** list = nullptr;
  UINT32 count = 0;
  std::vector<std::string> names;

  bool Find(MediaFoundation& mf) {
    ComPtr<IMFAttributes> query;
    if (FAILED(mf.create_attributes(&query, 1)) ||
        FAILED(
            query->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                           MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID)) ||
        FAILED(mf.enum_device_sources(query.Get(), &list, &count))) {
      list = nullptr;
      count = 0;
      return false;
    }
    for (UINT32 i = 0; i < count; ++i) {
      WCHAR* name = nullptr;
      UINT32 length = 0;
      if (SUCCEEDED(list[i]->GetAllocatedString(
              MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &name, &length))) {
        names.push_back(xe::to_utf8(
            std::u16string(reinterpret_cast<const char16_t*>(name), length)));
        CoTaskMemFree(name);
      } else {
        names.push_back("Camera " + std::to_string(i + 1));
      }
    }
    return true;
  }

  ~Devices() {
    for (UINT32 i = 0; i < count; ++i) {
      list[i]->Release();
    }
    CoTaskMemFree(list);
  }
};

class MediaFoundationWebcam final : public WebcamCapture {
 public:
  MediaFoundationWebcam(MediaFoundation mf, std::string name,
                        ComPtr<IMFActivate> activate,
                        ComPtr<IMFMediaSource> source,
                        ComPtr<IMFSourceReader> reader, uint32_t width,
                        uint32_t height, LONG stride)
      : mf_(mf),
        name_(std::move(name)),
        activate_(std::move(activate)),
        source_(std::move(source)),
        reader_(std::move(reader)),
        width_(width),
        height_(height),
        stride_(stride) {}

  ~MediaFoundationWebcam() override {
    reader_.Reset();
    source_->Shutdown();
    source_.Reset();
    activate_->ShutdownObject();
    activate_.Reset();
    mf_.Stop();
  }

  const std::string& name() const override { return name_; }

  bool Read(ColourImage& picture) override {
    // Blocks until the camera's next frame (the reader is synchronous).
    DWORD stream = 0, flags = 0;
    LONGLONG timestamp = 0;
    ComPtr<IMFSample> sample;
    HRESULT result =
        reader_->ReadSample(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), 0,
                            &stream, &flags, &timestamp, &sample);
    if (FAILED(result) || (flags & MF_SOURCE_READERF_ERROR) ||
        (flags & MF_SOURCE_READERF_ENDOFSTREAM)) {
      if (!reported_) {
        reported_ = true;
        XELOGW("Motion capture: the webcam stopped sending ({:08X})",
               uint32_t(result));
      }
      return false;
    }
    if (!sample) {
      return false;  // a gap in the stream, no picture
    }
    ComPtr<IMFMediaBuffer> buffer;
    if (FAILED(sample->ConvertToContiguousBuffer(&buffer))) {
      return false;
    }
    // A 2D buffer knows its own pitch, which is negative for a picture
    // stored bottom row first.
    ComPtr<IMF2DBuffer> buffer_2d;
    BYTE* first_row = nullptr;
    LONG pitch = 0;
    bool converted = false;
    if (SUCCEEDED(buffer.As(&buffer_2d)) &&
        SUCCEEDED(buffer_2d->Lock2D(&first_row, &pitch))) {
      converted = CopyRows(first_row, pitch, picture);
      buffer_2d->Unlock2D();
    } else {
      BYTE* data = nullptr;
      DWORD length = 0;
      if (SUCCEEDED(buffer->Lock(&data, nullptr, &length))) {
        LONG stride = stride_ ? stride_ : LONG(width_ * 4);
        BYTE* start =
            stride < 0 ? data + size_t(-stride) * (height_ - 1) : data;
        if (size_t(std::abs(stride)) * height_ <= length) {
          converted = CopyRows(start, stride, picture);
        }
        buffer->Unlock();
      }
    }
    return converted;
  }

 private:
  // RGB32 rows from the first row on screen, `pitch` bytes apart (negative
  // when going down the picture means going back in memory).
  bool CopyRows(const BYTE* first_row, LONG pitch, ColourImage& picture) {
    if (std::abs(pitch) < LONG(width_ * 4)) {
      return false;
    }
    picture.width = width_;
    picture.height = height_;
    picture.rgba.resize(size_t(width_) * height_ * 4);
    uint8_t* out = picture.rgba.data();
    for (uint32_t y = 0; y < height_; ++y) {
      const BYTE* in = first_row + ptrdiff_t(y) * pitch;
      for (uint32_t x = 0; x < width_; ++x, in += 4, out += 4) {
        out[0] = in[2];
        out[1] = in[1];
        out[2] = in[0];
        out[3] = 255;
      }
    }
    return true;
  }

  MediaFoundation mf_;
  std::string name_;
  ComPtr<IMFActivate> activate_;
  ComPtr<IMFMediaSource> source_;
  ComPtr<IMFSourceReader> reader_;
  uint32_t width_, height_;
  LONG stride_;
  bool reported_ = false;
};

// The camera's own format nearest the size wanted: the smallest at least
// that big, else the biggest; at 25 frames a second or more if it can.
ComPtr<IMFMediaType> ChooseNativeType(IMFSourceReader* reader, uint32_t width,
                                      uint32_t height) {
  ComPtr<IMFMediaType> best;
  uint64_t best_score = UINT64_MAX;
  for (DWORD i = 0;; ++i) {
    ComPtr<IMFMediaType> type;
    if (FAILED(reader->GetNativeMediaType(
            DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), i, &type))) {
      break;
    }
    UINT32 w = 0, h = 0, rate = 0, per = 1;
    if (FAILED(MFGetAttributeSize(type.Get(), MF_MT_FRAME_SIZE, &w, &h)) ||
        !w || !h) {
      continue;
    }
    MFGetAttributeRatio(type.Get(), MF_MT_FRAME_RATE, &rate, &per);
    bool big_enough = w >= width && h >= height;
    bool fast_enough = per && rate >= 25 * per;
    // Lower is better: fast enough first, then big enough, then the pixel
    // count nearest the wanted one.
    uint64_t pixels = uint64_t(w) * h, wanted = uint64_t(width) * height;
    uint64_t score = (fast_enough ? 0 : 1ull << 62) +
                     (big_enough ? 0 : 1ull << 61) +
                     (pixels > wanted ? pixels - wanted : wanted - pixels);
    if (score < best_score) {
      best_score = score;
      best = type;
    }
  }
  return best;
}

}  // namespace

std::vector<std::string> WebcamCapture::List() {
  MediaFoundation mf;
  if (!mf.Start()) {
    return {};
  }
  std::vector<std::string> names;
  {
    Devices devices;
    devices.Find(mf);
    names = devices.names;
  }
  mf.Stop();
  return names;
}

std::unique_ptr<WebcamCapture> WebcamCapture::Open(const std::string& wanted,
                                                   uint32_t width,
                                                   uint32_t height,
                                                   std::string& reason) {
  MediaFoundation mf;
  if (!mf.Start()) {
    reason =
        "Windows Media Foundation is missing (on a Windows N edition, "
        "install the Media Feature Pack)";
    return nullptr;
  }
  auto fail = [&](std::string why) -> std::unique_ptr<WebcamCapture> {
    reason = std::move(why);
    mf.Stop();
    return nullptr;
  };

  ComPtr<IMFActivate> activate;
  std::string name;
  {
    Devices devices;
    if (!devices.Find(mf) || !devices.count) {
      return fail("No webcam was found");
    }
    int chosen = ChooseWebcam(devices.names, wanted);
    if (chosen < 0) {
      std::string names;
      for (const auto& found : devices.names) {
        names += (names.empty() ? "" : ", ") + found;
      }
      return fail("No webcam is called \"" + wanted +
                  "\" (there are: " + names + ")");
    }
    activate = devices.list[chosen];
    name = devices.names[chosen];
  }

  ComPtr<IMFMediaSource> source;
  if (FAILED(activate->ActivateObject(IID_PPV_ARGS(&source)))) {
    return fail("The webcam \"" + name +
                "\" would not open (is another program using it?)");
  }
  auto fail_open = [&](std::string why) {
    source->Shutdown();
    activate->ShutdownObject();
    return fail(std::move(why));
  };

  // Video processing lets the reader convert whatever the camera sends
  // (YUY2, NV12, MJPEG) to RGB32.
  ComPtr<IMFAttributes> options;
  ComPtr<IMFSourceReader> reader;
  if (FAILED(mf.create_attributes(&options, 1)) ||
      FAILED(
          options->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE)) ||
      FAILED(mf.create_source_reader(source.Get(), options.Get(), &reader))) {
    return fail_open("The webcam \"" + name + "\" could not be read");
  }
  ComPtr<IMFMediaType> native = ChooseNativeType(reader.Get(), width, height);
  ComPtr<IMFMediaType> rgb;
  if (!native ||
      FAILED(reader->SetCurrentMediaType(
          DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, native.Get())) ||
      FAILED(mf.create_media_type(&rgb)) ||
      FAILED(rgb->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video)) ||
      FAILED(rgb->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32)) ||
      FAILED(reader->SetCurrentMediaType(
          DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, rgb.Get()))) {
    return fail_open("The webcam \"" + name +
                     "\" sends no picture Windows can convert");
  }
  ComPtr<IMFMediaType> current;
  UINT32 w = 0, h = 0;
  if (FAILED(reader->GetCurrentMediaType(
          DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), &current)) ||
      FAILED(MFGetAttributeSize(current.Get(), MF_MT_FRAME_SIZE, &w, &h)) ||
      !w || !h) {
    return fail_open("The webcam \"" + name + "\" gave no picture size");
  }
  UINT32 stride_bits = 0;
  LONG stride = 0;
  if (SUCCEEDED(current->GetUINT32(MF_MT_DEFAULT_STRIDE, &stride_bits))) {
    stride = LONG(stride_bits);
  }
  XELOGI("Motion capture: webcam \"{}\" open at {}x{}", name, w, h);
  return std::make_unique<MediaFoundationWebcam>(
      mf, name, std::move(activate), std::move(source), std::move(reader), w, h,
      stride);
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
