add_library(xenia-hid-motion STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_types.h
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_device.h
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_device.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_capture.h
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_capture.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/virtual_kinect_v1.h
  ${CMAKE_CURRENT_SOURCE_DIR}/virtual_kinect_v1.cc
)
target_link_libraries(xenia-hid-motion PUBLIC xenia-base)
xe_target_defaults(xenia-hid-motion)

if(XENIA_BUILD_TESTS)
  set(CMAKE_FOLDER "tests")
  add_subdirectory(testing)
endif()
