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
)
if(WIN32)
  target_sources(xenia-hid-motion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_sdk.h
    ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_device.h
    ${CMAKE_CURRENT_SOURCE_DIR}/kinect_v2_device.cc
  )
endif()
target_link_libraries(xenia-hid-motion PUBLIC xenia-base)
xe_target_defaults(xenia-hid-motion)

if(XENIA_BUILD_TESTS)
  set(CMAKE_FOLDER "tests")
  add_subdirectory(testing)
endif()
