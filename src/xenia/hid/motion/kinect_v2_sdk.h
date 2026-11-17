/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_HID_MOTION_KINECT_V2_SDK_H_
#define XENIA_HID_MOTION_KINECT_V2_SDK_H_

#include <cstdint>

#include "xenia/base/platform_win.h"

// The few parts of the Kinect for Windows SDK 2.0's COM interface the
// Kinect v2 backend uses, declared here so that building Xenia needs no SDK.
// The runtime (Kinect20.dll, from the free Kinect for Windows Runtime 2.0)
// is loaded when a player picks the device.
//
// A COM interface is a table of functions in a fixed order, so each
// interface below lists its methods in the SDK's order, up to the last one
// used. Methods we never call keep their place with simplified parameter
// types (another interface becomes IUnknown) that pass the same way. The
// order and types follow the SDK 2.0 header, Kinect.h; the names are the
// SDK's, so its documentation applies.

namespace xe {
namespace hid {
namespace motion {
namespace kinect20 {

using WaitableHandle = INT_PTR;  // WAITABLE_HANDLE
using TimeSpan = int64_t;        // TIMESPAN: 100-nanosecond units

struct PointF {
  float x, y;
};
struct CameraSpacePoint {
  float x, y, z;
};
struct DepthSpacePoint {
  float x, y;
};
struct ColorSpacePoint {
  float x, y;
};

struct IFrameDescription : public IUnknown {
  virtual HRESULT STDMETHODCALLTYPE get_Width(int* width) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Height(int* height) = 0;
};

struct IDepthFrame : public IUnknown {
  virtual HRESULT STDMETHODCALLTYPE CopyFrameDataToArray(UINT capacity,
                                                         UINT16* data) = 0;
  virtual HRESULT STDMETHODCALLTYPE AccessUnderlyingBuffer(UINT* capacity,
                                                           UINT16** buffer) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_FrameDescription(IFrameDescription** description) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_RelativeTime(TimeSpan* time) = 0;
};

struct IDepthFrameReader : public IUnknown {
  virtual HRESULT STDMETHODCALLTYPE
  SubscribeFrameArrived(WaitableHandle* handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  UnsubscribeFrameArrived(WaitableHandle handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  GetFrameArrivedEventData(WaitableHandle handle, IUnknown** event_data) = 0;
  // Fails (E_PENDING) when no frame has come since the last one.
  virtual HRESULT STDMETHODCALLTYPE AcquireLatestFrame(IDepthFrame** frame) = 0;
};

struct IDepthFrameSource : public IUnknown {
  virtual HRESULT STDMETHODCALLTYPE
  SubscribeFrameCaptured(WaitableHandle* handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  UnsubscribeFrameCaptured(WaitableHandle handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  GetFrameCapturedEventData(WaitableHandle handle, IUnknown** event_data) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_IsActive(BOOLEAN* active) = 0;
  virtual HRESULT STDMETHODCALLTYPE OpenReader(IDepthFrameReader** reader) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_DepthMinReliableDistance(UINT16* millimetres) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_DepthMaxReliableDistance(UINT16* millimetres) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_FrameDescription(IFrameDescription** description) = 0;
};

struct ICoordinateMapper : public IUnknown {
  virtual HRESULT STDMETHODCALLTYPE
  SubscribeCoordinateMappingChanged(WaitableHandle* handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  UnsubscribeCoordinateMappingChanged(WaitableHandle handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE GetCoordinateMappingChangedEventData(
      WaitableHandle handle, IUnknown** event_data) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapCameraPointToDepthSpace(CameraSpacePoint point, DepthSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapCameraPointToColorSpace(CameraSpacePoint point, ColorSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE MapDepthPointToCameraSpace(
      DepthSpacePoint point, UINT16 depth, CameraSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE MapDepthPointToColorSpace(
      DepthSpacePoint point, UINT16 depth, ColorSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapCameraPointsToDepthSpace(UINT count, const CameraSpacePoint* points,
                              UINT out_count, DepthSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapCameraPointsToColorSpace(UINT count, const CameraSpacePoint* points,
                              UINT out_count, ColorSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE MapDepthPointsToCameraSpace(
      UINT count, const DepthSpacePoint* points, UINT depth_count,
      const UINT16* depths, UINT out_count, CameraSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE MapDepthPointsToColorSpace(
      UINT count, const DepthSpacePoint* points, UINT depth_count,
      const UINT16* depths, UINT out_count, ColorSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapDepthFrameToCameraSpace(UINT count, const UINT16* depths, UINT out_count,
                             CameraSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapDepthFrameToColorSpace(UINT count, const UINT16* depths, UINT out_count,
                            ColorSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapColorFrameToDepthSpace(UINT count, const UINT16* depths, UINT out_count,
                            DepthSpacePoint* out) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  MapColorFrameToCameraSpace(UINT count, const UINT16* depths, UINT out_count,
                             CameraSpacePoint* out) = 0;
  // Allocates the table with CoTaskMemAlloc; free it with CoTaskMemFree.
  virtual HRESULT STDMETHODCALLTYPE
  GetDepthFrameToCameraSpaceTable(UINT32* entry_count, PointF** entries) = 0;
};

struct IKinectSensor : public IUnknown {
  virtual HRESULT STDMETHODCALLTYPE
  SubscribeIsAvailableChanged(WaitableHandle* handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  UnsubscribeIsAvailableChanged(WaitableHandle handle) = 0;
  virtual HRESULT STDMETHODCALLTYPE GetIsAvailableChangedEventData(
      WaitableHandle handle, IUnknown** event_data) = 0;
  virtual HRESULT STDMETHODCALLTYPE Open() = 0;
  virtual HRESULT STDMETHODCALLTYPE Close() = 0;
  virtual HRESULT STDMETHODCALLTYPE get_IsOpen(BOOLEAN* open) = 0;
  // Whether a sensor is plugged in and working; it takes a few seconds
  // after Open.
  virtual HRESULT STDMETHODCALLTYPE get_IsAvailable(BOOLEAN* available) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ColorFrameSource(IUnknown** source) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_DepthFrameSource(IDepthFrameSource** source) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_BodyFrameSource(IUnknown** source) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_BodyIndexFrameSource(IUnknown** source) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_InfraredFrameSource(IUnknown** source) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_LongExposureInfraredFrameSource(IUnknown** source) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_AudioSource(IUnknown** source) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  OpenMultiSourceFrameReader(DWORD frame_source_types, IUnknown** reader) = 0;
  virtual HRESULT STDMETHODCALLTYPE
  get_CoordinateMapper(ICoordinateMapper** mapper) = 0;
};

// Kinect20.dll's one entry point we need: the sensor object, whether or not
// a sensor is plugged in.
using GetDefaultKinectSensorFunction = HRESULT(WINAPI*)(IKinectSensor** sensor);

}  // namespace kinect20
}  // namespace motion
}  // namespace hid
}  // namespace xe

#endif  // XENIA_HID_MOTION_KINECT_V2_SDK_H_
