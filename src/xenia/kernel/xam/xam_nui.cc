/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2025 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <atomic>
#include <cstring>
#include "xenia/base/logging.h"
#include "xenia/emulator.h"
#include "xenia/kernel/kernel_flags.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"

#include "xenia/kernel/kinect_camera.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xam {
// https://web.cs.ucdavis.edu/~okreylos/ResDev/Kinect/MainPage.html

struct X_NUI_DEVICE_STATUS {
  /* Notes:
     - for one side func of XamNuiGetDeviceStatus
       - if some data address is less than zero then tilt_status = it
       - else another func is called and its return can set
     tilt_status = c0051200 or some value involving DetroitDeviceRequest
       - next PsCamDeviceRequest is called and if its return is less than zero
     then camera_status = return of PsCamDeviceRequest
       - else it equals an unknown local_1c
       - finally McaDeviceRequest is called and if its return is less than zero
     then mic_status = return of McaDeviceRequest
       - else it equals an unknown local_14
     - status_bits can be set to status_bits | 0x44 or | 0x40
  */
  xe::be<uint32_t> camera_status;
  xe::be<uint32_t> tilt_status;
  xe::be<uint32_t> mic_status;
  xe::be<uint32_t> status_bits;
};
static_assert_size(X_NUI_DEVICE_STATUS, 0x10);

// Get
dword_result_t XamNuiGetDeviceStatus_entry(
    pointer_t<X_NUI_DEVICE_STATUS> status_ptr) {
  /* Notes:
     - it does return a value that is not always used
     - returns values are X_ERROR_SUCCESS, 0xC0050006, and others
     - 1) On func start *status_ptr = 0, status_ptr->tilt_status = 0,
     status_ptr->mic_status = 0, and status_ptr->status_bits = 0
     - 2) calls XamXStudioRequest(6,&var <- = 0);
     - if return is greater than -1 && var & 0x80000000 != 0 then set
     status_ptr->tilt_status = 0xC000009D, status_ptr->mic_status = 0xC000009D,
     and status_ptr->status_bits = 0x20
     - lots of branching functions after
  */

  status_ptr.Zero();

  auto camera = kernel_state()->kinect_camera();
  if (camera->DeviceChosen()) {
    // Titles read only status_bits, whatever the result (docs/
    // kinect-pscam.md section 13): bit 0 gates their Kinect play, and bits 0
    // and 1 together are what they keep as "usable".
    const bool present = camera->CameraPresent();
    // Logged the first time and whenever it changes, so a tester's log shows
    // what the title was told.
    static std::atomic<int> last_logged = -1;
    if (last_logged.exchange(int(present)) != int(present)) {
      XELOGI("XamNuiGetDeviceStatus: told the title the Kinect is {}",
             present ? "connected and ready" : "not connected");
    }
    if (!present) {
      status_ptr->camera_status = 0xC000009D;  // STATUS_DEVICE_NOT_CONNECTED
      status_ptr->tilt_status = 0xC000009D;
      return 0xC0050006;
    }
    status_ptr->status_bits = 0x3;
    return X_ERROR_SUCCESS;
  }

  const bool kinect_initialized = kernel_state()->nui()->NuiInitialized();

  status_ptr->status_bits = kinect_initialized;
  return kinect_initialized ? X_ERROR_SUCCESS : 0xC0050006;
}
DECLARE_XAM_EXPORT1(XamNuiGetDeviceStatus, kNone, kStub);

dword_result_t XamUserNuiGetUserIndex_entry(unknown_t unk, lpdword_t index) {
  return X_E_NO_SUCH_USER;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndex, kNone, kStub);

dword_result_t XamUserNuiGetUserIndexForSignin_entry(lpdword_t user_index) {
  if (!user_index) {
    return X_E_INVALIDARG;
  }

  for (uint32_t i = 0; i < XUserMaxUserCount; i++) {
    auto profile = kernel_state()->xam_state()->GetUserProfile(i);
    if (profile) {
      *user_index = i;
      return X_E_SUCCESS;
    }
  }

  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForSignin, kNone, kImplemented);

dword_result_t XamUserNuiGetUserIndexForBind_entry(lpdword_t user_index) {
  if (!user_index) {
    return X_E_INVALIDARG;
  }

  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForBind, kNone, kStub);

struct X_NUI_DEPTH_CALIBRATION {
  uint8_t data[0x30];
};
static_assert_size(X_NUI_DEPTH_CALIBRATION, 0x30);

dword_result_t XamNuiGetDepthCalibration_entry(
    pointer_t<X_NUI_DEPTH_CALIBRATION> depth_data) {
  // always cleared before new data written
  std::memset(depth_data, 0, sizeof(X_NUI_DEPTH_CALIBRATION));
  return X_STATUS_NO_SUCH_FILE | X_FACILITY_NT_BIT;
}
DECLARE_XAM_EXPORT1(XamNuiGetDepthCalibration, kNone, kStub);

// Skeleton
qword_result_t XamNuiSkeletonGetBestSkeletonIndex_entry(int_t unk) {
  return 0xffffffffffffffff;
}
DECLARE_XAM_EXPORT1(XamNuiSkeletonGetBestSkeletonIndex, kNone, kStub);

/* XamNuiCamera Notes
   - most require message calls to xam in 0x0002Bxxx area
*/
dword_result_t XamNuiCameraTiltSetCallback_entry(dword_t callback) {
  kernel_state()->nui()->SetTiltCallback(callback);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltSetCallback, kNone, kStub);

dword_result_t XamNuiCameraTiltGetStatus_entry(
    pointer_t<X_NUI_TILT_STATUS> tilt_status) {
  /* Notes:
     - Used by XamNuiCameraElevationGetAngle, and XamNuiCameraSetFlags
     - if it returns anything greater than -1 then both above functions continue
     - Both funcs request 0x50 bytes with XtSs header
  */
  if (!tilt_status || tilt_status->buffer_size < sizeof(X_NUI_TILT_STATUS) ||
      tilt_status->signature != make_fourcc("XtSs")) {
    return X_E_FAIL;
  }
  auto camera = kernel_state()->kinect_camera();
  if (camera->DeviceChosen()) {
    if (!camera->CameraPresent()) {
      return X_E_DEVICE_NOT_CONNECTED;
    }
    // The virtual motor: never moving (a title resets its tracker while it
    // moves), gravity matching its angle. +0x08..+0x1F are left as they are.
    auto& tilt = camera->tilt();
    auto gravity = tilt.gravity();
    auto floor_plane = tilt.floor_plane();
    auto floor_gravity = tilt.floor_gravity();
    for (size_t i = 0; i < 3; ++i) {
      tilt_status->gravity_long_avg[i] = gravity[i];
      tilt_status->floor_gravity[i] = floor_gravity[i];
    }
    for (size_t i = 0; i < 4; ++i) {
      tilt_status->floor_plane[i] = floor_plane[i];
    }
    tilt_status->flags = X_TILT_STATUS_FLAGS(0);
    tilt_status->lock_flags = 0;
    static std::atomic<bool> logged = false;
    if (!logged.exchange(true)) {
      XELOGI("XamNuiCameraTiltGetStatus: first answer, motor at {} degrees",
             camera->tilt().angle());
    }
    return X_ERROR_SUCCESS;
  }
  // TODO(knuckleslee): Report the Kinect accelerometer and motor status.
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltGetStatus, kNone, kStub);

dword_result_t XamNuiCameraElevationGetAngle_entry(lpdword_t elevation_angle,
                                                   lpdword_t camera_moving) {
  auto camera = kernel_state()->kinect_camera();
  if (camera->DeviceChosen()) {
    if (!camera->CameraPresent()) {
      return X_E_DEVICE_NOT_CONNECTED;
    }
    if (elevation_angle) {
      *elevation_angle = uint32_t(camera->tilt().angle());
    }
    if (camera_moving) {
      *camera_moving = 0;
    }
    return X_ERROR_SUCCESS;
  }
  X_NUI_TILT_STATUS tilt_status = {};
  tilt_status.buffer_size = sizeof(tilt_status);
  tilt_status.signature = make_fourcc("XtSs");  // (XtSs)?
  X_STATUS result = XamNuiCameraTiltGetStatus_entry(&tilt_status);
  if (XSUCCEEDED(result)) {
    // TODO(knuckleslee): Calculate the angle in degrees from
    // gravity_long_avg[2] and gravity_long_avg[1].
    *elevation_angle = 0;
    *camera_moving =
        tilt_status.flags & (X_TILT_STATUS_FLAGS::TILT_STATUS_UNK1 |
                             X_TILT_STATUS_FLAGS::TILT_STATUS_UNK2);  // moving
    result = X_E_FAIL;  // until implementation improves
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationGetAngle, kNone, kStub);

// The tilt calls below work only with a motion-capture device chosen; without
// one there is no Kinect to move. A title ignores their results (docs/
// kinect-pscam.md section 13).
dword_result_t XamNuiCameraElevationSetAngle_entry(int_t degrees) {
  auto camera = kernel_state()->kinect_camera();
  if (!camera->CameraPresent()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  if (degrees < hid::motion::kV1TiltMinDegrees ||
      degrees > hid::motion::kV1TiltMaxDegrees) {
    return X_E_INVALIDARG;
  }
  XELOGI("XamNuiCameraElevationSetAngle: title turned the motor to {} degrees",
         int32_t(degrees));
  camera->tilt().SetAngle(degrees);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationSetAngle, kNone, kImplemented);

dword_result_t XamNuiCameraElevationStopMovement_entry() {
  // The virtual motor is never moving.
  return kernel_state()->kinect_camera()->CameraPresent()
             ? X_ERROR_SUCCESS
             : X_E_DEVICE_NOT_CONNECTED;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationStopMovement, kNone, kImplemented);

// The floor plane comes by value as two 64-bit halves: (nx, ny) and (nz, d),
// each float in the high word first. `first` is set on the first call after
// the library's own aiming move.
dword_result_t XamNuiCameraRememberFloor_entry(qword_t plane_xy,
                                               qword_t plane_zd,
                                               dword_t first) {
  auto camera = kernel_state()->kinect_camera();
  if (!camera->CameraPresent()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  auto to_float = [](uint32_t bits) {
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  };
  uint64_t xy = plane_xy, zd = plane_zd;
  XELOGI("XamNuiCameraRememberFloor: title remembered a floor (first: {})",
         uint32_t(first));
  camera->tilt().RememberFloor(
      {to_float(uint32_t(xy >> 32)), to_float(uint32_t(xy)),
       to_float(uint32_t(zd >> 32)), to_float(uint32_t(zd))});
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraRememberFloor, kNone, kImplemented);

// How a title's tilt request went, for the system's tilt UI; nothing here
// shows one.
dword_result_t XamNuiCameraTiltReportStatus_entry(dword_t code,
                                                  lpvoid_t record) {
  XELOGD("XamNuiCameraTiltReportStatus({:08X})", uint32_t(code));
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltReportStatus, kNone, kStub);

dword_result_t XamNuiCameraGetTiltControllerType_entry() {
  /* Notes:
     - Earlier decompile notes: DetroitDeviceRequest and
       (LZCOUNT(result) << 0x20) >> 0x25.
  */
  auto camera = kernel_state()->kinect_camera();
  if (camera->DeviceChosen()) {
    return camera->CameraPresent() ? 1 : 0;  // 1: a Kinect v1 motor
  }
  // TODO(boma): Get the controller type from connected device.
  // Currently only reports no controller or type 1.
  const bool kinect_initialized =
      kernel_state()->xconfig()->ReadSetting<uint32_t>(
          X_CONFIG_CATEGORY::XCONFIG_USER_CATEGORY, XCONFIG_USER_RETAIL_FLAGS) &
      X_RETAIL_FLAGS::KinectInitialized;
  return kinect_initialized;
}
DECLARE_XAM_EXPORT1(XamNuiCameraGetTiltControllerType, kNone, kStub);

dword_result_t XamNuiCameraSetFlags_entry(qword_t unk1, dword_t unk2) {
  X_STATUS result = X_E_DEVICE_NOT_CONNECTED;
  int controller_type = XamNuiCameraGetTiltControllerType_entry();

  if (controller_type == 1) {
    X_NUI_TILT_STATUS tilt_status = {};
    tilt_status.buffer_size = sizeof(tilt_status);
    tilt_status.signature = make_fourcc("XtSs");  // (XtSs)?
    result = XamNuiCameraTiltGetStatus_entry(&tilt_status);
    if (!XSUCCEEDED(result)) {
      return result;
    }
    // TODO(knuckleslee): Apply the flags once their meaning is known.
    return X_E_FAIL;  // until implementation improves
  }
  if (controller_type == 2) {
    return X_ERROR_SUCCESS;
  }
  if (controller_type >= 3) {
    return X_E_FAIL;  // until implementation improves
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamNuiCameraSetFlags, kNone, kStub);

dword_result_t XamIsNuiUIActive_entry() {
  return kernel_state()->xam_state()->IsUIActive();
}
DECLARE_XAM_EXPORT1(XamIsNuiUIActive, kNone, kImplemented);

dword_result_t XamNuiIsDeviceReady_entry() {
  /* device_state Notes:
   - used with XNotifyBroadcast(kXNotificationSystemNUIHardwareStatusChanged,
   device_state)
   - known values:
     - 0x0001
     - 0x0004
     - 0x0040
  */

  return kernel_state()->nui()->NuiInitialized();
}
DECLARE_XAM_EXPORT1(XamNuiIsDeviceReady, kNone, kImplemented);

dword_result_t XamIsNuiAutomationEnabled_entry(unknown_t unk1, unknown_t unk2) {
  /* Notes:
     - XamIsNuiAutomationEnabled = XamIsNatalPlaybackEnabled
     - Always returns X_E_SUCCESS? Maybe check later versions
     - Recieves param but never interacts with them
     - No Operations
  */
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT2(XamIsNuiAutomationEnabled, kNone, kStub, kHighFrequency);

dword_result_t XamIsNatalPlaybackEnabled_entry(unknown_t unk1, unknown_t unk2) {
  /* Notes:
     - XamIsNuiAutomationEnabled = XamIsNatalPlaybackEnabled
     - Always returns X_E_SUCCESS? Maybe check later versions
     - Recieves param but never interacts with them
     - No Operations
  */
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT2(XamIsNatalPlaybackEnabled, kNone, kStub, kHighFrequency);

dword_result_t XamNuiIsChatMicEnabled_entry() {
  /* Notes:
     - call ExGetXConfigSetting(7,9,&xconfig_nui,0x1c,local_40);
     - check xconfig_nui.flag & 0x8
     - return True or false based on result
  */
  return false;
}
DECLARE_XAM_EXPORT1(XamNuiIsChatMicEnabled, kNone, kImplemented);

/* HUD Notes:
   - XamNuiHudGetEngagedTrackingID, XamNuiHudIsEnabled,
   XamNuiHudSetEngagedTrackingID, XamNuiHudInterpretFrame, and
   XamNuiHudGetEngagedEnrollmentIndex all utilize the same data address
*/
uint32_t nui_unknown_1 = 0;
uint32_t engaged_tracking_id = 0;
char nui_unknown_2 = '\0';

dword_result_t XamNuiHudSetEngagedTrackingID_entry(dword_t id) {
  auto nui_ = kernel_state()->nui();
  if (id) {
    if (!nui_->GetNUIDataPtr()) {
      return X_E_FAIL;
    }
    nui_->SetEngagedTrackingId(id);
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudSetEngagedTrackingID, kNone, kImplemented);

qword_result_t XamNuiHudGetEngagedTrackingID_entry() {
  auto nui_ = kernel_state()->nui();
  if (nui_->GetNUIDataPtr()) {
    return nui_->GetEngagedTrackingId();
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudGetEngagedTrackingID, kNone, kImplemented);

dword_result_t XamNuiHudIsEnabled_entry() {
  auto nui_ = kernel_state()->nui();
  bool result = XamNuiIsDeviceReady_entry();
  if (nui_->GetNUIDataPtr() && nui_->GetUnknown2() && result) {
    return true;
  }
  return false;
}
DECLARE_XAM_EXPORT1(XamNuiHudIsEnabled, kNone, kImplemented);

dword_result_t XamNuiHudGetInitializeFlags_entry() {
  return kernel_state()->nui()->GetInitFlags();
}
DECLARE_XAM_EXPORT1(XamNuiHudGetInitializeFlags, kNone, kImplemented);

void XamNuiHudGetVersions_entry(lpqword_t xam_version_ptr,
                                lpqword_t xtl_version_ptr) {
  auto nui_ = kernel_state()->nui();
  if (xam_version_ptr) {
    *xam_version_ptr = nui_->GetNUIVersion(0);
  }
  if (xtl_version_ptr) {
    *xtl_version_ptr = nui_->GetNUIVersion(1);
  }
}
DECLARE_XAM_EXPORT1(XamNuiHudGetVersions, kNone, kImplemented);

/* XamNuiIdentity Notes:
   - most require message calls to xam in 0x0002Cxxx area
*/
qword_result_t XamNuiIdentityGetSessionId_entry() {
  auto nui_ = kernel_state()->nui();
  if (nui_->GetSessionId() == 0) {
    // xboxkrnl::XeCryptRandom_entry(NUI_Session_Id, 8);
    nui_->SetSessionId(0xDEADF00DDEADF00D);
  }
  return nui_->GetSessionId();
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetSessionId, kNone, kImplemented);

dword_result_t XamNuiIdentityEnrollForSignIn_entry(dword_t unk1, qword_t unk2,
                                                   qword_t unk3, dword_t unk4) {
  /* Notes:
     - Decompiler issues so double check
  */
  if (XamNuiHudIsEnabled_entry() == false) {
    return X_E_FAIL;
  }
  // buffer [2]
  // buffer[0] = unk
  // var = unk4
  // return func(0xfe,0x2c010,buffer,0xc,unk3);
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityEnrollForSignIn, kNone, kStub);

dword_result_t XamNuiIdentityAbort_entry(dword_t unk) {
  if (XamNuiHudIsEnabled_entry() == false) {
    return X_E_FAIL;
  }
  // buffer [4]
  // buffer[0] = unk
  // return func(0xfe,0x2c00e,buffer,4,0)
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityAbort, kNone, kStub);

// Other
dword_result_t XamUserNuiEnableBiometric_entry(dword_t user_index,
                                               int_t enable) {
  return X_E_INVALIDARG;
}
DECLARE_XAM_EXPORT1(XamUserNuiEnableBiometric, kNone, kStub);

struct X_NUI_PLAYER_ENGAGEMENT_UPDATE {
  uint8_t data[0x1C];
};
static_assert_size(X_NUI_PLAYER_ENGAGEMENT_UPDATE, 0x1C);

void XamNuiPlayerEngagementUpdate_entry(
    lpunknown_t unk1, unknown_t unk2,
    pointer_t<X_NUI_PLAYER_ENGAGEMENT_UPDATE> engagement_data) {
  // always cleared before new data written
  std::memset(engagement_data, 0, sizeof(X_NUI_PLAYER_ENGAGEMENT_UPDATE));
}
DECLARE_XAM_EXPORT1(XamNuiPlayerEngagementUpdate, kNone, kStub);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(NUI);
