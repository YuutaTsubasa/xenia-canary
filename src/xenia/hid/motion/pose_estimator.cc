/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/hid/motion/pose_estimator.h"

#include <algorithm>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <vector>

#include "xenia/base/cvar.h"
#include "xenia/base/filesystem.h"
#include "xenia/base/logging.h"
#include "xenia/hid/motion/pose_mediapipe.h"

#if XE_PLATFORM_WIN32
#include "xenia/base/platform_win.h"
#else
#include <dlfcn.h>
#endif

// ONNX Runtime is loaded at run time, not linked, so the C++ API is pointed
// at the loaded library's API table by hand (Ort::InitApi).
#define ORT_API_MANUAL_INIT
#include "third_party/onnxruntime/include/onnxruntime_cxx_api.h"
#undef ORT_API_MANUAL_INIT

DEFINE_path(motion_pose_folder, "",
            "Folder with ONNX Runtime and the MediaPipe pose models for "
            "motion_device = webcam. Empty: \"pose\" beside the executable.",
            "HID");

namespace xe {
namespace hid {
namespace motion {

namespace {

// A person must reach this confidence (detector score, pose presence and
// each torso point) to count. Free Riders Recompiled's default.
constexpr float kConfidence = 0.5f;

// Loads ONNX Runtime once for the whole run and points the C++ API at it.
// The library stays loaded until Xenia exits (unloading it under live
// sessions is not safe). Returns the reason it could not, or nothing.
std::optional<std::string> LoadOnnxRuntime(
    const std::filesystem::path& folder) {
  static std::mutex lock;
  static bool loaded = false;
  static std::string failure;
  std::lock_guard<std::mutex> guard(lock);
  if (loaded) {
    return std::nullopt;
  }
  if (!failure.empty()) {
    return failure;
  }

  using GetApiBase = const OrtApiBase*(ORT_API_CALL*)();
  GetApiBase get_api_base = nullptr;
#if XE_PLATFORM_WIN32
  HMODULE library =
      LoadLibraryW((folder / kOnnxRuntimeLibrary).wstring().c_str());
  if (!library) {
    library = LoadLibraryW(L"onnxruntime.dll");
  }
  if (library) {
    get_api_base =
        reinterpret_cast<GetApiBase>(GetProcAddress(library, "OrtGetApiBase"));
  }
#else
  void* library = dlopen((folder / kOnnxRuntimeLibrary).c_str(), RTLD_NOW);
  if (!library) {
    library = dlopen(kOnnxRuntimeLibrary, RTLD_NOW);
  }
  if (library) {
    get_api_base =
        reinterpret_cast<GetApiBase>(dlsym(library, "OrtGetApiBase"));
  }
#endif
  if (!library || !get_api_base) {
    failure = fmt::format(
        "ONNX Runtime was not found (put {} from ONNX Runtime 1.30 or newer "
        "in {})",
        kOnnxRuntimeLibrary, xe::path_to_utf8(folder));
    return failure;
  }
  const OrtApiBase* base = get_api_base();
  const OrtApi* api = base ? base->GetApi(ORT_API_VERSION) : nullptr;
  if (!api) {
    // An older library (Windows itself carries one) answers no API newer
    // than its own.
    failure = fmt::format(
        "The ONNX Runtime found is version {}, too old (1.30 or newer is "
        "needed; put it in {})",
        base ? base->GetVersionString() : "unknown", xe::path_to_utf8(folder));
    return failure;
  }
  Ort::InitApi(api);
  loaded = true;
  XELOGI("Motion capture: ONNX Runtime {} loaded", base->GetVersionString());
  return std::nullopt;
}

Ort::SessionOptions SingleThreaded() {
  Ort::SessionOptions options;
  // One thread each: this runs beside the emulator, which wants the cores.
  options.SetIntraOpNumThreads(1);
  options.SetInterOpNumThreads(1);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  return options;
}

// One network with its tensor shapes checked when loaded, so a different
// model fails at once instead of being fed a camera picture.
class PoseNetwork {
 public:
  PoseNetwork(Ort::Env& environment, const std::filesystem::path& file,
              std::vector<int64_t> input_shape,
              std::vector<std::vector<int64_t>> output_shapes)
      : session_(environment, file.c_str(), SingleThreaded()),
        input_shape_(std::move(input_shape)),
        output_shapes_(std::move(output_shapes)) {
    if (session_.GetInputCount() != 1) {
      throw std::runtime_error("it has more than one input");
    }
    // The shape info points into its type info, which must outlive it.
    auto input_type = session_.GetInputTypeInfo(0);
    auto input = input_type.GetTensorTypeAndShapeInfo();
    if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        input.GetShape() != input_shape_) {
      throw std::runtime_error("its input is not the expected one");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_.GetInputNameAllocated(0, allocator).get();
    // Outputs are found by shape, which is unique in these models.
    for (const auto& wanted : output_shapes_) {
      std::string name;
      for (size_t n = 0; n < session_.GetOutputCount(); ++n) {
        auto output_type = session_.GetOutputTypeInfo(n);
        auto output = output_type.GetTensorTypeAndShapeInfo();
        if (output.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
            output.GetShape() == wanted) {
          if (!name.empty()) {
            throw std::runtime_error("two of its outputs look alike");
          }
          name = session_.GetOutputNameAllocated(n, allocator).get();
        }
      }
      if (name.empty()) {
        throw std::runtime_error("an expected output is missing");
      }
      output_names_.push_back(std::move(name));
    }
  }

  std::vector<Ort::Value> Run(std::vector<float>& input) {
    size_t expected = 1;
    for (int64_t dimension : input_shape_) {
      expected *= size_t(dimension);
    }
    if (input.size() != expected) {
      throw std::runtime_error("pose input buffer size");
    }
    auto memory =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto tensor = Ort::Value::CreateTensor<float>(
        memory, input.data(), input.size(), input_shape_.data(),
        input_shape_.size());
    const char* input_name = input_name_.c_str();
    std::vector<const char*> names;
    for (const auto& name : output_names_) {
      names.push_back(name.c_str());
    }
    auto values = session_.Run(Ort::RunOptions{nullptr}, &input_name, &tensor,
                               1, names.data(), names.size());
    for (size_t i = 0; i < values.size(); ++i) {
      if (!values[i].IsTensor() ||
          values[i].GetTensorTypeAndShapeInfo().GetShape() !=
              output_shapes_[i]) {
        throw std::runtime_error("pose output shape changed");
      }
    }
    return values;
  }

 private:
  Ort::Session session_;
  std::vector<int64_t> input_shape_;
  std::vector<std::vector<int64_t>> output_shapes_;
  std::string input_name_;
  std::vector<std::string> output_names_;
};

// MediaPipe's two-step pose: the detector finds the person only when there
// is no crop to follow; after that, each picture's pose suggests the next
// picture's crop.
class MediaPipePose final : public PoseEstimator {
 public:
  MediaPipePose(const std::filesystem::path& detector,
                const std::filesystem::path& landmarks)
      : environment_(ORT_LOGGING_LEVEL_WARNING, "xenia-pose"),
        detector_(environment_, detector, {1, 3, 224, 224},
                  {{1, int64_t(kPersonAnchorCount), 12},
                   {1, int64_t(kPersonAnchorCount), 1}}),
        pose_(environment_, landmarks, {1, 256, 256, 3},
              {{1, 195}, {1, 1}, {1, 117}}) {}

  bool Estimate(const ColourImage& picture, PoseLandmarks& landmarks) override {
    if (!picture.width || !picture.height ||
        picture.rgba.size() / 4 < size_t(picture.width) * picture.height) {
      return false;
    }
    if (picture.width != width_ || picture.height != height_) {
      region_.reset();
      width_ = picture.width;
      height_ = picture.height;
    }
    try {
      if (!region_) {
        PoseRegion whole{picture.width / 2.0f, picture.height / 2.0f,
                         std::max(picture.width, picture.height) / 2.0f, 0.0f};
        auto input = MediaPipeInput(picture, whole, true);
        auto values = detector_.Run(input);
        region_ = DecodePerson(
            {values[0].GetTensorData<float>(), kPersonAnchorCount * 12},
            {values[1].GetTensorData<float>(), kPersonAnchorCount},
            picture.width, picture.height, kConfidence);
        if (!region_) {
          return false;
        }
      }
      auto input = MediaPipeInput(picture, *region_, false);
      auto values = pose_.Run(input);
      std::optional<PoseRegion> next;
      bool found = DecodeMediaPipe({values[0].GetTensorData<float>(), 195},
                                   {values[2].GetTensorData<float>(), 117},
                                   values[1].GetTensorData<float>()[0],
                                   *region_, kConfidence, landmarks, next);
      // A lost or strange crop sends the next picture to the detector; this
      // picture is not tried twice.
      region_ = found ? next : std::nullopt;
      if (region_ && region_->radius > 2.0f * std::max(width_, height_)) {
        region_.reset();
      }
      return found;
    } catch (const std::exception& error) {
      region_.reset();
      if (!reported_) {
        reported_ = true;
        XELOGW("Motion capture: the pose model failed: {}", error.what());
      }
      return false;
    }
  }

 private:
  Ort::Env environment_;
  PoseNetwork detector_, pose_;
  std::optional<PoseRegion> region_;
  uint32_t width_ = 0, height_ = 0;
  bool reported_ = false;
};

}  // namespace

PoseEstimator::~PoseEstimator() = default;

std::filesystem::path PoseEstimator::DefaultFolder() {
  if (!cvars::motion_pose_folder.empty()) {
    return cvars::motion_pose_folder;
  }
  return xe::filesystem::GetExecutableFolder() / "pose";
}

std::unique_ptr<PoseEstimator> PoseEstimator::Open(
    const std::filesystem::path& folder, std::string& reason) {
  auto detector = folder / kPoseDetectorModel;
  auto landmarks = folder / kPoseLandmarkModel;
  std::error_code error;
  for (const auto& model : {detector, landmarks}) {
    if (!std::filesystem::is_regular_file(model, error)) {
      reason =
          fmt::format("The pose model {} is missing", xe::path_to_utf8(model));
      return nullptr;
    }
  }
  if (auto failure = LoadOnnxRuntime(folder)) {
    reason = *failure;
    return nullptr;
  }
  try {
    auto estimator = std::make_unique<MediaPipePose>(detector, landmarks);
    XELOGI("Motion capture: MediaPipe pose models loaded from {}",
           xe::path_to_utf8(folder));
    return estimator;
  } catch (const std::exception& failure) {
    reason = fmt::format("The pose models in {} could not be used: {}",
                         xe::path_to_utf8(folder), failure.what());
    return nullptr;
  }
}

}  // namespace motion
}  // namespace hid
}  // namespace xe
