# Motion capture (Kinect) in Xenia Canary: design

Status: **draft for review** (branch `claude/kinect-one` on
YuutaTsubasa/xenia-canary). Nothing below is implemented yet; this document
is the plan the code will follow, one small commit at a time. Each commit is
listed with a plain explanation in [motion-capture-log.md](motion-capture-log.md).

## Goal

Let Xbox 360 Kinect titles (Kinect Adventures, Kinect Sports, Dance Central,
Sonic Free Riders, ...) run in Xenia with one of three host devices:

1. a **Kinect for Xbox 360** (v1, models 1414 / 1473, or Kinect for Windows v1),
2. a **Kinect for Xbox One** (v2, or Kinect for Windows v2), which no Xbox 360
   title ever knew about,
3. a **plain webcam** with a pose-estimation model.

All three go through one motion-capture API inside Xenia, so the guest-facing
emulation is written once and does not know which device is behind it.

## 1. How the console does it

On the Xbox 360 a Kinect title talks to the sensor through three layers:

| Layer | Where it runs | What it does |
| --- | --- | --- |
| Sensor drivers | kernel (`xboxkrnl`) | `PsCamDeviceRequest` (depth + colour camera), `DetroitDeviceRequest` (tilt motor and accelerometer), `McaDeviceRequest` (microphone array), `UsbdGetNatal*` (the Kinect's USB hub) |
| NUI library | **linked into the title** (XDK) | `NuiInitialize`, `NuiImageStream*`, `NuiSkeletonGetNextFrame`, `NuiTransformSmooth`, ... The skeleton tracker itself runs here, on the title's CPU and GPU time, from the raw camera frames |
| System UI and identity | XAM | Kinect guide, HUD, engaged player, face sign-in (`XamNui*`, XAM messages `0x2Bxxx`/`0x2Cxxx`) |

The important consequence: **a title gets raw sensor data and computes the
skeletons itself.** The camera driver delivers 640x480 depth as raw 11-bit
disparity and 640x480 colour as a Bayer mosaic, plus calibration tables; the
title's own NUI library turns those into players, skeletons and the
depth-with-player-index image its gesture code reads.

## 2. What Xenia Canary has today

- `src/xenia/kernel/nui.{h,cc}`: `NUIController`, a little state (engaged
  tracking id, init flags, versions). "Kinect ready" is the
  `KinectInitialized` retail flag in the console settings, not a device.
- `src/xenia/kernel/xam/xam_nui.cc`: the `XamNui*` exports, mostly stubs:
  device status, tilt status and elevation, HUD, identity session.
- `PsCamDeviceRequest`, `McaDeviceRequest`, `DetroitDeviceRequest` and the
  `UsbdGetNatal*` exports are in the kernel export table but **not
  implemented**. So today a title never sees a camera.

### Prior work by others (none merged for the camera)

| Who | Approach | State |
| --- | --- | --- |
| knuckleslee, [xenia-canary#1274](https://github.com/xenia-canary/xenia-canary/pull/1274) and branch [`kinect-v1-stage1`](https://github.com/knuckleslee/xenia-canary/tree/kinect-v1-stage1) | Implements `PsCamDeviceRequest` with a **real Kinect v1 over libusb** (UsbDk or WinUSB); the title's own NUI library tracks. Played Sonic Free Riders (5345084D) through a full race. Needs GPU fixes for the tracker (#1279, #1280) and `readback_resolve = "full"`, `readback_memexport = true`. | #1274 closed by a maintainer as "really disorganized"; asked to refactor first. Smaller pieces (#1272, #1273, #1278) merged. |
| ryantimjohn, branch [`add-kinect-support`](https://github.com/ryantimjohn/xenia-canary/tree/add-kinect-support), issue [#1290](https://github.com/xenia-canary/xenia-canary/issues/1290) | About 30k lines; a host-side NUI subsystem using the Kinect for Windows runtime. Kinect Adventures and Dance Central 3 reported working. | Being split into stacked PRs, one at a time. |
| JacoMur05, dcwestra (xenia-edge, Linux) | Further v1 forks (Dance Central 2/3, Kinect Sports, Fable: The Journey). | Forks only. |

Nobody supports a **Kinect v2** or a **webcam**; both are asked for in #1290
and in Free-Riders-Recompiled#421. That is what this branch adds. The lesson
from #1274 is that upstream wants small, organized changes; that is how this
branch is built.

## 3. Which guest path

There are two places the emulator could put a host device's data:

**A. Sensor level (recommended).** Emulate the Kinect's camera, tilt and
microphone drivers (`PsCamDeviceRequest`, `DetroitDeviceRequest`, ...). The
title's own NUI library runs unchanged and does the tracking, exactly as on a
console.

- Works for every title the same way: they all link the same XDK library and
  talk to the same driver. No per-title knowledge, no addresses.
- Gestures behave as on the console, because the console's own tracker and
  smoothing make the skeletons.
- Cost: the device's data has to be turned into what a Kinect v1 camera sends
  (raw disparity, Bayer colour, calibration), and the tracker needs GPU
  features Xenia does not fully have yet (see §7).

**B. Skeleton level.** Replace the title's NUI library functions with host
code that hands over skeletons from a host tracker (the Kinect SDK, a pose
model). This is what Free Riders Recompiled does (it knows the addresses in
its one title). In Xenia it would mean finding the library inside every
title, per XDK version; it bypasses the console's tracker, so gestures differ
from the console.

**This design takes path A.** The motion-capture API still carries skeletons
(the Kinect SDKs give them, and the webcam needs them to make a depth image),
so path B stays possible later without changing the backends.

## 4. Architecture

```
 guest (title + its NUI library)
        │  PsCamDeviceRequest / DetroitDeviceRequest / XamNui* device status
        ▼
 ┌───────────────────────────────────────────────┐
 │ 1. Guest-facing emulation (kernel)             │  xboxkrnl_pscam.cc, xboxkrnl_detroit.cc,
 │    camera request codes, frame-read queue,     │  xam_nui.cc (device status, tilt)
 │    tilt state                                  │
 └──────────────────────┬────────────────────────┘
                        │  KinectV1Frame (raw disparity, Bayer, calibration)
 ┌──────────────────────┴────────────────────────┐
 │ 2b. Virtual Kinect v1 (encoder)                │  hid/motion/virtual_kinect_v1.*
 │    any SensorFrame -> what a v1 camera sends   │
 └──────────────────────┬────────────────────────┘
                        │  SensorFrame (depth in mm, colour, lenses, bodies)
 ┌──────────────────────┴────────────────────────┐
 │ 2a. Motion-capture API                         │  hid/motion/motion_capture.*
 │    MotionCaptureSystem: picks a device (cvar), │
 │    owns its thread, newest frame, status       │
 └──────┬──────────────┬───────────────┬─────────┘
        ▼              ▼               ▼
 3. Kinect v1      Kinect v2        Webcam + pose model
    raw USB or     Kinect SDK 2.0   capture -> pose -> skeleton
    Kinect SDK 1.8 (libfreenect2    -> synthetic depth
    (libfreenect   later)
    later)
```

### 4.1 The motion-capture API (`src/xenia/hid/motion/`)

Plain host code, no guest types, unit-testable on every platform.

- `SensorFrame`: one moment from the device.
  - `depth`: 16-bit millimetres (0 = no reading), width/height, and the lens
    it was taken with (`CameraIntrinsics`: focal lengths, centre, distortion).
  - `colour`: 8-bit RGB(A), width/height, its lens, and how to place a depth
    point in it (`DepthToColour`: rotation + translation between the two
    lenses).
  - `bodies` (optional): tracked skeletons in NUI joint order (20 joints,
    metres, +x to the player's right, +z away from the sensor), with
    per-joint state. A v1/v2 SDK fills these; a webcam fills one from its
    pose model; raw USB leaves them empty.
  - `floor_plane`, `gravity` (optional), `timestamp`.
  - `raw_v1` (optional): the untouched packets of a real Kinect v1, when the
    backend reads one over USB; the encoder passes them through.
- `MotionCapabilities`: has depth / colour / bodies / tilt motor, max bodies,
  native resolutions.
- `MotionDevice` (interface every backend implements): `Open()`, `Close()`,
  `capabilities()`, `status()` (not found / driver missing / running, with a
  human-readable reason), `GetNewestFrame(SensorFrame&)`; optional
  `SetTilt(degrees)` and `GetTilt()`.
- `MotionCaptureSystem`: owned by `hid::InputSystem`, the way the Skylanders
  portal (`hid/portal`) already is, so kernel code reaches it through
  `emulator()->input_system()`.
  The cvar `motion_device = none | auto | kinect_v1 | kinect_v2 | webcam`
  (default `none`, so nothing changes for anyone who does not opt in) picks a
  backend; it opens on the title's first camera request and closes at
  shutdown.

### 4.2 The virtual Kinect v1 (`virtual_kinect_v1.*`)

The one piece that makes every device look like the sensor the title expects.

- **Depth**: reproject each depth pixel (through its lens) into camera space,
  move it into a virtual v1 depth camera, project through the v1 lens at
  640x480, nearest point wins. The title converts each pixel through the
  table it gets from request code 24 and treats the result as millimetres
  ([kinect-pscam.md](kinect-pscam.md) §11). So the virtual v1 answers code
  24 with an identity table and writes millimetres directly (0 = no reading,
  big-endian, sensor order, not mirrored); no disparity encoding is needed.
  Only a real v1 read over raw USB keeps its own disparity and table.
- **Colour**: map to the v1 colour camera's 640x480 view (using the depth
  registration for alignment), then mosaic to Bayer as the v1 sends it.
- **Calibration**: the registration blocks (request codes 11/14, which only
  affect depth-to-colour mapping, not the tracker's depth) and the identity
  depth table (code 24) of a nominal v1, generated from fixed nominal
  parameters, so title, encoder and reprojection agree by construction.
- **Tilt**: a virtual motor and accelerometer. A v2 and a webcam have no
  motor; the virtual one answers the title (angle, moving, gravity) and
  rotates the virtual camera instead.
- A real v1 read over USB is passed through untouched (its own calibration).

### 4.3 Backends

| Backend | Platform | Gives | Notes |
| --- | --- | --- | --- |
| Kinect v1, raw USB (libusb; Xenia already has `third_party/libusb`) | Windows first | raw packets + the sensor's calibration | Best fidelity: exactly what the console sees. Needs UsbDk or WinUSB bound to the camera, which conflicts with the Kinect SDK 1.8 driver. |
| Kinect v1, Kinect for Windows SDK 1.8 (`Kinect10.dll`, loaded at runtime) | Windows | mm depth, colour, skeletons | For people who already have the SDK driver installed. Goes through the encoder. |
| Kinect v2, Kinect for Windows SDK 2.0 (`Kinect20.dll`, loaded at runtime) | Windows | 512x424 mm depth, 1920x1080 colour, coordinate mapper, 6 skeletons | Reprojected into the virtual v1 (wider lens, other position). |
| Kinect v1/v2 via libfreenect / libfreenect2 | Linux, macOS (later) | depth, colour, factory calibration | No skeletons; not needed on path A. |
| Webcam + pose model (ONNX Runtime; MediaPipe pose or RTMPose) | all | colour, one skeleton | Has no depth. A synthetic depth image is drawn from the skeleton (a body of capsules along the joints, in front of an empty room with a floor). **Experimental**: whether the console's tracker accepts a drawn body is unknown until tried. Last in the plan. |

## 5. What comes from Free Riders Recompiled

Free Riders Recompiled already has most device code; it was written for path
B (it gives one title skeletons and a depth-with-player-index image), so the
**device side** moves over and the **title side** stays.

### Moves (generic)

Rows marked **rewrite** are knuckleslee's code in Free Riders: Xenia gets a
new implementation of the same job (see §6), not a port.

| Free Riders file(s) | Becomes | Changes when ported |
| --- | --- | --- |
| `kinect_v2_win32.cpp` (device part) | Kinect v2 SDK 2.0 backend, **rewrite** | Output a `SensorFrame` (mm depth + lens + colour + mapper) instead of the game image |
| `kinect_v2_depth.{h,cpp}` (`KinectDepthView`, `kinect_v2_synthesize_depth`) | virtual v1 reprojection, **rewrite** | Keep the reprojection; output millimetres at 640x480 instead of 320x240 depth-with-player-index (the title computes player index itself) |
| `kinect_v2_image.cpp` | colour mapping to the v1 view, **rewrite** | as is, minus preview sizes |
| `kinect_sensor_win32.cpp` (device part) | Kinect v1 SDK 1.8 backend, **rewrite** | `SensorFrame` output |
| `kinect_sensor_freenect.cpp`, `kinect_sensor_freenect2.cpp` | libfreenect backends (later) | `SensorFrame` output; the pose-on-colour part is not needed on path A |
| `kinect_record_raw.cpp` | recording/replay of device frames for tests, **rewrite** (own format) | file format kept |
| `camera_capture.h`, `camera_capture_win32.cpp`, `_v4l2.cpp`, `_apple.mm`, `_android.cpp` | webcam capture | `_v4l2`, `_apple` port as is; `camera_capture.cpp`, `_win32`, `_android` are about half knuckleslee's lines: **rewrite** those |
| `pose_estimator*`, `pose_mediapipe*`, `pose_skeleton*`, `pose_nui_joints.h`, `pose_smoothing*` | webcam pose -> 20-joint skeleton | as is |
| `motion_input.{h,cpp}`, `motion_input_device.cpp` | ideas for `MotionCaptureSystem` (device choice, capabilities) | rewritten in Xenia style |

### Stays in Free Riders (title-specific)

`nui_hooks.cpp`, `nui_race*.cpp`, `nui_smooth_hooks.cpp`,
`nui_transform_smooth*`, `nui_menu_progress.h`, `nui_player_routing.h`,
`nui_speech.cpp`, `nui_analysis_*`, `nui_device_status.cpp` (title
addresses), `camera_race_motion.h`, `camera_arm_activity.h`,
`camera_leg_activity.h`, `camera_overthrow.h`, `race_scripted_motion*`,
`kinect_race_record*`, `kinect_stability*` and `pose_stability*` (menu-hand
tuning for that title), `kinect_depth_features*`, `kinect_depth_compare*`,
the tuner and preview windows (launcher UI; Xenia would get its own small
ImGui preview later), avatar files.

`nui_skeleton.cpp` (writing a guest `NUI_SKELETON_FRAME`) is generic
knowledge but only needed for path B; not ported now.

## 6. Licensing — needs your decision

Xenia Canary is BSD-3-Clause; Free Riders Recompiled is GPL-3.0. Code can
only move into this fork under BSD if **everyone who wrote it** agrees.
`git blame` on the files above:

| Free Riders files | Lines by YuutaTsubasa | Lines by knuckleslee |
| --- | --- | --- |
| `kinect_v2_depth.*`, `kinect_v2_image.cpp`, `kinect_v2_resynth.cpp`, `kinect_record_raw.cpp` | 0 | all (about 1,600) |
| `kinect_v2_win32.cpp` | 21 | 1,230 |
| `kinect_sensor_win32.cpp`, `kinect_sensor.h` | 22 | 710 |
| `camera_capture.cpp`, `camera_capture_win32.cpp`, `camera_capture_android.cpp` | 410 | 426 |
| `kinect_sensor_freenect*.cpp`, `kinect_depth_body.cpp`, `camera_capture_v4l2.cpp`, `camera_capture_apple.mm`, all `pose_*`, `motion_input*` | all | 0 |

So the Windows Kinect v1/v2 backends and the v2 reprojection — the heart of
the Kinect v2 work — are **knuckleslee's code**. Options:

1. Ask knuckleslee to license those files to this fork under BSD-3-Clause
   (they already contribute to Xenia under BSD, and their own Xenia v1 work
   is BSD).
2. Write those parts anew for Xenia from the SDK documentation, without
   copying. Slower, and having read the GPL code makes "anew" debatable.
3. Keep the fork GPL-3.0 (allowed: BSD code can be combined into a GPL
   work), giving up any chance of upstreaming.

The user's own files (webcam, pose, freenect, motion input) can be relicensed
by the user alone.

**Decision (2026-11-17, YuutaTsubasa): option 2, rewrite.** The parts
knuckleslee wrote in Free Riders Recompiled are written anew here, and are
not ported. While writing them:

- the sources are public documentation and permissively licensed code: the
  Kinect for Windows SDK 1.8 / 2.0 reference (function names, struct
  layouts, nominal camera constants such as the 285.63 px depth focal length
  are interface facts), libfreenect / libfreenect2 (Apache-2.0, credited
  where their math is followed) and the OpenKinect protocol notes;
- knuckleslee's Free Riders files are not opened, and nothing is copied from
  them (their design ideas described in this document, such as "reproject v2
  depth into a virtual v1 camera", are ideas, not code);
- each rewritten file says in its header which public sources it follows.

The user's own files are ported under BSD-3-Clause by the user's decision to
bring them here.

### 6.1 The guest camera driver (PsCam) is rewritten too

**Decision (2026-11-17, YuutaTsubasa):** `PsCamDeviceRequest` is not taken
from knuckleslee's `kinect-v1-stage1` branch (BSD) either; it is written
here from our own tracing. The request codes and reply formats are worked
out from how a title's NUI library calls the driver (the guest code, read
locally and never committed), and written down in
[kinect-pscam.md](kinect-pscam.md) with where each fact was seen. The public
description of xenia-canary#1274 names some codes; it is used only as a
pointer for where to look, and every fact in our notes is checked against
the guest code. The branch's code is not opened.

Third-party pieces:

| Piece | License | Fit with BSD | Note |
| --- | --- | --- | --- |
| libusb | LGPL-2.1 | already in Xenia (`third_party/libusb`) | nothing new |
| libfreenect, libfreenect2 | dual Apache-2.0 / GPL-2.0 | yes, taking Apache-2.0 | keep NOTICE; only for the later Linux/macOS backends |
| ONNX Runtime | MIT | yes | prebuilt binaries are large; fetch at build time like Free Riders does, do not commit |
| MediaPipe pose / person detector (opencv_zoo ONNX) | Apache-2.0 | yes | model files fetched, not committed |
| RTMPose (OpenMMLab) | Apache-2.0 (code and released weights) | yes | check the weight's model card again before shipping |
| Kinect for Windows SDK 1.8 / 2.0 | Microsoft proprietary | not redistributed | loaded at runtime (`LoadLibrary`); the user installs the SDK. Free Riders declares the few structs it uses by hand. |
| UsbDk | Apache-2.0 | not bundled | the user installs it for raw USB |
| knuckleslee's `kinect-v1-stage1` (raw USB v1, PsCam request codes) | BSD-3-Clause (Xenia headers) | yes | not used: rewritten (§6.1) |

## 7. Things the tracker needs that are not about devices

knuckleslee's testing showed the console's tracker needs GPU behaviour Xenia
lacks by default: `readback_resolve = "full"`, `readback_memexport = true`,
texture fetches in the shader interpreter (#1279, closed), resolves without
vertices (#1280, open), and some kernel/XAM pieces (LDI decompression,
superseded by #1355; `XamContentOpenFile` on guest devices, #1276). These are
separate from the motion-capture API and are left to upstream; this branch
will document which ones a test needs rather than carry them.

## 8. Commit plan

Each step builds, keeps `motion_device = none` behaving exactly as today,
and is one commit (or a few) with a plain message.

1. Design doc and log (this).
2. `hid/motion`: `SensorFrame`, `CameraIntrinsics`, `MotionCapabilities`,
   `MotionDevice`, a `NullMotionDevice`, the `motion_device` cvar. No caller.
3. `MotionCaptureSystem` owned by the input system; logs which device it
   would open.
4. Virtual v1: nominal calibration replies, the depth frame encoding
   (millimetres, big-endian, sensor order), Bayer mosaic, with unit tests.
5. Virtual v1: reprojection from another lens (written anew in
   `depth_reprojection.*`), with unit tests on drawn scenes.
6. `PsCamDeviceRequest` guest side fed by the virtual v1 (depends on the
   decision about knuckleslee's request-code work).
7. Kinect v2 backend (SDK 2.0). Done 2026-11-18, depth only, not yet run
   on hardware; see §8.1.
8. Virtual tilt (`DetroitDeviceRequest`, `XamNuiCameraTilt*`), device status
   from the motion system instead of the console-settings flag.
   Done 2026-11-18 through XAM only (the traced title does not import
   `DetroitDeviceRequest`); see §8.2.
9. Kinect v1 backends (SDK 1.8; raw USB).
10. Webcam capture and pose model; synthetic depth (experimental).
    Done 2026-11-18, not yet run with a real camera or a title; see §8.3.

### 8.1 The Kinect v2 backend as written

- `kinect_v2_sdk.h` declares the six SDK interfaces it calls, in the SDK
  2.0 header's vtable order (checked by script against a public copy of
  `Kinect.h`), so building needs no SDK. `Kinect20.dll` comes from the free
  Kinect for Windows Runtime 2.0 and is loaded only when chosen.
- A capture thread owns every runtime object (COM, multithreaded). It
  waits for the sensor and its depth-to-camera table, then polls the depth
  reader and publishes 512x424 millimetres plus per-pixel rays.
- Directions come from the table, not a lens model, so mirroring of the
  SDK's images does not matter. The table is in SDK camera space (+X to the
  sensor's left, +Y up); lens space negates both (`kinect_v2_frames.cc`,
  with tests). This relies on the v1 picture being unmirrored (docs/
  kinect-pscam.md §11), which is inferred from tracing, not seen on a real
  console.
- The v2 is treated as sitting exactly where the v1 would (identity
  pose). The small offset between the two sensors' depth cameras is not
  modelled; `CameraPose` is there if a real test shows it matters.
- `Open()` runs on the first camera query, which comes from a guest
  thread, and can block it up to 5 seconds while the sensor starts.
- Still to do: colour, bodies or floor plane (not needed by the title's
  own tracker), and a first run with a real Kinect v2.

### 8.2 Virtual tilt and device status as written

From the fourth trace (kinect-pscam.md §13):
- `hid/motion/virtual_tilt.*`: the motor arrives at once (a title resets
  its tracker while the moving bits are set), clamps to -27..27 degrees,
  and reports gravity whose atan2(z, y) is the angle, which is how the
  title checks it arrived. The camera picture does not tilt; a later step
  could turn the reprojection (`CameraPose`) by the virtual angle so the
  title's floor search sees the view move.
- XAM answers from it only when `motion_device` is not none:
  `XamNuiGetDeviceStatus` status_bits 0x3 while the device runs,
  `XamNuiCameraTiltGetStatus`, `ElevationGetAngle`/`SetAngle`,
  `StopMovement`, `RememberFloor`, `TiltReportStatus` (logged).
- Not done: invoking the tilt callback (it carries system tilt requests,
  e.g. from a tuner UI, which Xenia does not have), a real v1 motor, and
  `DetroitDeviceRequest` for titles that import it.

### 8.3 The webcam backend as written

`motion_device = webcam` (or `auto`, after a Kinect v2 that does not
start) runs this on a "Webcam" thread for each picture:

1. `webcam_capture_*`: an RGBA picture, about 640x480. Windows: Media
   Foundation's source reader, which converts and decodes for us; written
   anew from Microsoft's documentation. Its functions are looked up at run
   time so Windows N still starts. Linux: V4L2, ported from the user's
   Free Riders code (syntax-checked only). Setting: `webcam_device` (part
   of the name; empty for the first).
2. `pose_estimator`: MediaPipe's person detector and pose landmarker
   (OpenCV Zoo's ONNX conversions, Apache-2.0) run by ONNX Runtime 1.30 or
   newer. The runtime and both models are **not shipped**: they are looked
   for in `motion_pose_folder` (default `pose` beside the executable). The
   headers are in `third_party/onnxruntime`; the library is loaded at run
   time. Windows 11 carries an ONNX Runtime 1.17 in System32, which is
   reported as too old rather than used.
3. `pose_smoothing`: the 1 Euro filter.
4. `pose_skeleton`: 17 points to 20 joints; every player at the same size
   (shoulders 0.36 m) and distance (2.5 m). Setting: `webcam_mirrored`.
5. `body_placement`: sideways and up/down from where the hips are in the
   picture (setting: `webcam_fov`, default 60 degrees); the usual
   hip-to-feet drop stood on the floor.
6. `synthetic_depth`: the body drawn as capsules in a bare room (floor
   0.9 m below the sensor, wall at 3.8 m) through the virtual v1's lens.
   A lost player stays 0.3 s, then the room is empty.

The frame also carries the picture (with a pinhole lens as wide as
`webcam_fov`), the skeleton and the drawn floor.

Steps 1-4 come from Free Riders Recompiled files the user wrote alone
(checked with `git blame`); 5 and 6 and the Windows capture are new. Not
ported: Free Riders' older RTMPose path and its menu-hand stability
filter (knuckleslee's).

Checked so far: every step's unit tests; the real models load and run
under ONNX Runtime 1.30. Not yet: a real webcam (the test opens one only
when `XENIA_TEST_WEBCAM` is set), a title's tracker reading the drawn
body, the virtual tilt turning the drawn view (`SyntheticRoom` can, but
the device does not hear the angle yet), and Linux.

## 9. Open questions

- ~~Licensing (§6)~~: decided 2026-11-17, rewrite.
- ~~Whether to build the guest side on knuckleslee's `kinect-v1-stage1`~~:
  decided 2026-11-17, rewrite (§6.1). Still open: whether to talk to
  knuckleslee and ryantimjohn first so the efforts don't collide upstream.
- Whether the console's tracker accepts a webcam's drawn depth body
  (§8.3), and whether its body sizes and room need tuning for it.
- How the player gets ONNX Runtime and the two models (§8.3): a script
  like Free Riders' `fetch_pose_model.py` (that one is partly
  knuckleslee's, so it would be written anew), or instructions.
- The v2 axis handling (§8.1) and whether the title tracks from
  reprojected v2 depth at all: needs a real Kinect v2 (the PC this was
  written on has none).
- Whether reporting a tilt the camera did not make confuses the title's
  floor search when it sweeps the angle (§8.2); needs a game run.
- Test titles: which ones the person in Free-Riders-Recompiled#421 will try
  first, and what logs to ask for.
