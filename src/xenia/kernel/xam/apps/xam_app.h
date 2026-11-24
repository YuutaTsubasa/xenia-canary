/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2020 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_APPS_XAM_APP_H_
#define XENIA_KERNEL_XAM_APPS_XAM_APP_H_

#include <atomic>
#include <mutex>

#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/xam/app_manager.h"

namespace xe {
namespace kernel {
namespace xam {
namespace apps {

struct X_NUI_TITLE_INITIALIZE_DATA {
  xe::be<uint64_t> xam_version;
  xe::be<uint64_t> xtl_version;
  xe::be<uint32_t> flags;
  // 0x04 padding
};
static_assert_size(X_NUI_TITLE_INITIALIZE_DATA, 0x18);

class XamApp : public App {
 public:
  explicit XamApp(KernelState* kernel_state);

  X_HRESULT DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                uint32_t buffer_length) override;

 private:
  // Kinect identity (docs/kinect-pscam.md section 14): tells the title's
  // identity callback that the skeleton in this 0x2B002 frame is a
  // signed-in player.
  void AnnounceIdentity(const uint8_t* frame);
  // Logs whether the title's skeleton tracker sees anyone in 0x2B002 frames.
  void ReportTracking(const uint8_t* frame);

  // The callback the title registered with 0x2C009, the guest block handed
  // to it, and the tracking id of the skeleton last announced.
  std::atomic<uint32_t> identity_callback_ = 0;
  uint32_t identity_block_ = 0;
  uint32_t identified_tracking_id_ = 0;

  // ReportTracking's counts.
  std::mutex tracking_mutex_;
  bool tracking_seen_ = false;
  uint32_t tracking_frames_total_ = 0;
  uint32_t tracking_frames_ = 0;
  uint32_t tracking_frames_with_someone_ = 0;
  uint32_t tracking_reports_ = 0;
};

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_APPS_XAM_APP_H_
