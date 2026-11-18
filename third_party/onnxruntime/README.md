# ONNX Runtime headers

The C and C++ API headers of [ONNX Runtime](https://github.com/microsoft/onnxruntime)
**1.30.0** (commit `f2c39fe2f838cf35ce7da92824f5a5e3ee6e88a7`), unmodified, from
the official `onnxruntime-win-x64-1.30.0` release package. MIT licence
(`LICENSE`).

Only the headers are here. The runtime library itself (`onnxruntime.dll`,
`libonnxruntime.so`) is not built or shipped with Xenia: the webcam motion
backend (`src/xenia/hid/motion/pose_estimator.*`) loads it when a player
chooses `motion_device = webcam`, and goes without a pose model if it is
missing. That is why the C++ API is used with `ORT_API_MANUAL_INIT`: nothing
links against the library.

The C API is versioned: a library of this version or newer serves the API
these headers describe.

Files: `onnxruntime_c_api.h`, `onnxruntime_cxx_api.h`, `onnxruntime_cxx_inline.h`,
`onnxruntime_float16.h`, `onnxruntime_ep_c_api.h`, `onnxruntime_error_code.h`.
