add_library(xenia-hid-motion STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_types.h
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_device.h
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_device.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_capture.h
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_capture.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/depth_reprojection.h
  ${CMAKE_CURRENT_SOURCE_DIR}/depth_reprojection.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/virtual_kinect_v1.h
  ${CMAKE_CURRENT_SOURCE_DIR}/virtual_kinect_v1.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/virtual_tilt.h
  ${CMAKE_CURRENT_SOURCE_DIR}/virtual_tilt.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_frames.h
  ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_frames.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_landmarks.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_skeleton.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_skeleton.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_smoothing.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_smoothing.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_mediapipe.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_mediapipe.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/synthetic_depth.h
  ${CMAKE_CURRENT_SOURCE_DIR}/synthetic_depth.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_estimator.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pose_estimator.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/webcam_capture.h
  ${CMAKE_CURRENT_SOURCE_DIR}/webcam_pixels.cc
)
if(WIN32)
  target_sources(xenia-hid-motion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_sdk.h
    ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_device.h
    ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_device.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/webcam_capture_win.cc
  )
  # Media Foundation's GUIDs (a static library; its functions are looked up
  # at run time, see webcam_capture_win.cc).
  target_link_libraries(xenia-hid-motion PRIVATE mfuuid)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_sources(xenia-hid-motion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/webcam_capture_v4l2.cc
  )
else()
  target_sources(xenia-hid-motion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/webcam_capture_none.cc
  )
endif()
target_link_libraries(xenia-hid-motion PUBLIC xenia-base)
if(NOT WIN32)
  # dlopen, for ONNX Runtime (pose_estimator.cc).
  target_link_libraries(xenia-hid-motion PRIVATE ${CMAKE_DL_LIBS})
endif()
xe_target_defaults(xenia-hid-motion)

if(XENIA_BUILD_TESTS)
  set(CMAKE_FOLDER "tests")
  add_subdirectory(testing)
endif()
