/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2021 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/xam/apps/xam_app.h"

#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/cpu/processor.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/nui.h"
#include "xenia/kernel/xam/xam_content_device.h"
#include "xenia/kernel/xenumerator.h"
#include "xenia/kernel/xthread.h"

#include <map>
#include <mutex>

/* Notes:
   - Messages ids that start with 0x00021xxx are UI calls
   - Messages ids that start with 0x00023xxx are used for the user profile
   - Messages ids that start with 0x0002Bxxx are used by the Kinect device
   usually for camera related functions
   - Messages ids that start with 0x0002Cxxx are used by the XamNuiIdentity
   functions
*/

DECLARE_bool(motion_sign_in_first_user);

namespace xe {
namespace kernel {
namespace xam {
namespace apps {

XamApp::XamApp(KernelState* kernel_state) : App(kernel_state, 0xFE) {}

X_HRESULT XamApp::DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                      uint32_t buffer_length) {
  // NOTE: buffer_length may be zero or valid.
  auto buffer = memory_->TranslateVirtual(buffer_ptr);
  switch (message) {
    case 0x0002000E: {
      X_ENUMERATE_PARAM* data_ptr =
          reinterpret_cast<X_ENUMERATE_PARAM*>(buffer);

      XELOGD(
          "XEnumerateCrossTitle({:04X}, {:04X}, {:04X}, {:04X}, {}, {}, "
          "{:04X})",
          data_ptr->user_index.get(), data_ptr->flags.get(),
          data_ptr->private_enum_structure_ptr.get(),
          data_ptr->buffer_ptr.get(), data_ptr->buffer_size.get(),
          data_ptr->items_requested.get(), data_ptr->items_returned_ptr.get());

      if (!data_ptr->buffer_ptr || !data_ptr->private_enum_structure_ptr) {
        return X_E_INVALIDARG;
      }

      auto enum_struct =
          memory_->TranslateVirtual<X_KENUMERATOR_CONTENT_AGGREGATE*>(
              data_ptr->private_enum_structure_ptr);

      auto e = kernel_state_->object_table()->LookupObject<XEnumerator>(
          enum_struct->handle);

      if (!e) {
        return X_E_INVALIDARG;
      }

      assert_true(enum_struct->magic == kXObjSignature);
      assert_true(data_ptr->buffer_size == sizeof(XCONTENT_DATA_INTERNAL));

      XCONTENT_DATA_INTERNAL* content_data_ptr =
          memory_->TranslateVirtual<XCONTENT_DATA_INTERNAL*>(
              data_ptr->buffer_ptr);

      uint32_t item_count = 0;
      std::memset(content_data_ptr, 0, data_ptr->buffer_size);

      X_RESULT result =
          e->WriteItems(reinterpret_cast<uint8_t*>(content_data_ptr),
                        data_ptr->buffer_size, &item_count);

      result = X_HRESULT_FROM_WIN32(result);

      xe::be<uint32_t>* items_returned_ptr =
          memory_->TranslateVirtual<xe::be<uint32_t>*>(
              data_ptr->items_returned_ptr);

      *items_returned_ptr = item_count;

      return result;
    }
    case 0x00020021: {
      struct XContentQueryVolumeDeviceType {
        char root_name[64];
        xe::be<uint32_t> is_title_process;
        xe::be<DeviceType> device_type_ptr;
        xe::be<uint32_t> overlapped_ptr;
      }* data = reinterpret_cast<XContentQueryVolumeDeviceType*>(buffer);
      assert_true(buffer_length == sizeof(XContentQueryVolumeDeviceType));

      std::string target;
      if (!kernel_state_->file_system()->FindSymbolicLink(
              std::string(data->root_name) + ':', target)) {
        return X_E_INVALIDARG;
      }

      // Only apply this check to XContent packages
      if (!target.starts_with("\\Device\\Package_")) {
        return X_E_INVALIDARG;
      }

      xe::be<DeviceType>* device_type_ptr =
          memory_->TranslateVirtual<xe::be<DeviceType>*>(
              static_cast<uint32_t>(data->device_type_ptr.get()));

      switch (kernel_state_->deployment_type_) {
        case XDeploymentType::kDownload:
        case XDeploymentType::kInstalledToHDD: {
          *device_type_ptr = DeviceType::HDD;
        } break;
        case XDeploymentType::kOpticalDisc: {
          *device_type_ptr = DeviceType::ODD;
        } break;
        default: {
          *device_type_ptr = DeviceType::Invalid;
        } break;
      }

      XELOGD("XContentQueryVolumeDeviceType('{}', {:08X}, {:08X}, {:08X})",
             data->root_name,
             static_cast<uint32_t>(data->is_title_process.get()),
             static_cast<uint32_t>(data->device_type_ptr.get()),
             static_cast<uint32_t>(data->overlapped_ptr.get()));

      return X_E_SUCCESS;
    }
    case 0x00021012: {
      uint32_t enabled = xe::load_and_swap<uint32_t>(buffer);
      XELOGD("XEnableGuestSignin: {}", enabled ? "true" : "false");
      return X_E_SUCCESS;
    }
    case 0x00022005: {
      struct XTITLE_GET_DEPLOYMENT_TYPE {
        xe::be<uint32_t> deployment_type_ptr;
        xe::be<uint32_t> overlapped_ptr;
      }* data = reinterpret_cast<XTITLE_GET_DEPLOYMENT_TYPE*>(buffer);
      assert_true(!buffer_length ||
                  buffer_length == sizeof(XTITLE_GET_DEPLOYMENT_TYPE));
      auto deployment_type =
          memory_->TranslateVirtual<uint32_t*>(data->deployment_type_ptr);
      *deployment_type = static_cast<uint32_t>(kernel_state_->deployment_type_);
      XELOGD("XTitleGetDeploymentType({:08X}, {:08X}",
             data->deployment_type_ptr.get(), data->overlapped_ptr.get());
      return X_E_SUCCESS;
    }
    case 0x0002B001: {
      // Games used in:
      // 42560819, 373307D9
      uint32_t ms_delay = xe::load_and_swap<uint32_t>(buffer);

      XELOGD("XNuiDelayUI unimplemented({:08X})", ms_delay);
      return X_E_SUCCESS;
    }
    case 0x0002B002: {
      // Sent every frame by the Kinect library in titles such as Sonic Free
      // Riders, Game Party: In Motion and EA SPORTS Active 2. Its argument
      // holds two small numbers (0 and 5 in Free Riders), a mask (0x2DFF)
      // and four floats that look like the floor's plane: a normal close to
      // (0, 1, 0) and the camera's height. It looks like the title telling
      // the system what it sees, so take it and say it worked; the title got
      // a failure on every frame before. Free Riders' hand cursor still comes
      // up with this answer.
      static bool logged = false;
      if (!logged && buffer_ptr) {
        logged = true;
        auto words = reinterpret_cast<const xe::be<uint32_t>*>(buffer);
        auto floats = reinterpret_cast<const xe::be<float>*>(buffer);
        XELOGI(
            "XamApp: Kinect message 0x2B002 (first of many): {:08X} {:08X} "
            "{:08X} {:08X}, plane {} {} {} {}",
            uint32_t(words[0]), uint32_t(words[1]), uint32_t(words[2]),
            uint32_t(words[3]), float(floats[4]), float(floats[5]),
            float(floats[6]), float(floats[7]));
      }
      if (buffer_ptr) {
        ReportTracking(buffer);
        AnnounceIdentity(buffer);
      }
      return X_E_SUCCESS;
    }
    case 0x0002B003: {
      // Games used in:
      // 4D5309C9
      // It only receives buffer
      X_NUI_TITLE_INITIALIZE_DATA* args =
          reinterpret_cast<X_NUI_TITLE_INITIALIZE_DATA*>(buffer);
      XELOGD("XNuiTitleInitialize({:016X}, {:016X}, flags: 0x{:08X}), Stubbed",
             args->xam_version.get(), args->xtl_version.get(),
             args->flags.get());
      X_HRESULT result = X_E_FAIL;

      if (args->xam_version == 5) {
        kernel_state_->nui()->SetNUIVersion(uint64_t(args->xam_version), 0);
        kernel_state_->nui()->SetNUIVersion(uint64_t(args->xtl_version), 1);
        kernel_state_->nui()->SetInitFlags(uint32_t(args->flags));
        if ((args->flags & 0x80000000) == 0) {
          XELOGD("Todo: write data to two pointers");
          result = X_E_SUCCESS;
        }
      }
      if ((args->flags & 0x40000000) == 0) {
        XELOGD(
            "Todo: call XamNuiGetDeviceStatus and if status does not have flag "
            "1 set but has flags 4 and 0x40 call XamShowNuiTroubleshooterUI");
      }

      return result;
    }
    case 0x0002B004: {
      // Games used in:
      // Emerald Dashboard
      // Called with no buffer

      kernel_state_->nui()->SetNUIVersion(0, 0);
      kernel_state_->nui()->SetNUIVersion(0, 1);
      kernel_state_->nui()->SetInitFlags(0);
      kernel_state_->nui()->SetTiltCallback(0);

      XELOGD("XNuiTitleShutdown stubbed");
      return X_E_SUCCESS;
    }
    case 0x0002C009: {
      // Registers (non-zero) or removes (zero) a Kinect identity callback.
      // TODO(knuckleslee): Invoke the callback for identity notifications.
      if (!buffer_ptr) {
        return X_E_INVALIDARG;
      }
      {
        const uint32_t callback = xe::load_and_swap<uint32_t>(buffer);
        identity_callback_ = callback;
        XELOGI("XamApp: Kinect identity callback {:08X}{}", callback,
               cvars::motion_sign_in_first_user
                   ? ""
                   : " (never called back: motion_sign_in_first_user is off)");
      }
      return X_E_SUCCESS;
    }
    // Causes dashboard to correctly process language/region change. It does not
    // contain any buffer.
    case 0x8000000D: {
      const bool is_pc_enabled =
          (kernel_state_->xconfig()->ReadSetting<uint8_t>(
               XCONFIG_USER_CATEGORY, XCONFIG_USER_PC_FLAGS) &
           X_PC_FLAGS::PCEnabled) != 0;

      return is_pc_enabled ? X_E_ACCESS_DENIED : X_E_SUCCESS;
    }
  }
  // A title may send the same unknown message every frame. Say what it looked
  // like the first few times, with the words it points at, any names those
  // words point at, and where in the title it came from, so a log tells us
  // what the message is for; then stay quiet instead of filling the log.
  static std::mutex unknown_mutex;
  static std::map<uint32_t, uint32_t> unknown_counts;
  uint32_t seen;
  {
    std::lock_guard<std::mutex> lock(unknown_mutex);
    seen = ++unknown_counts[message];
  }
  if (seen <= 3) {
    std::string words, names;
    // True when every page from address to address + 48 is committed and
    // readable.
    auto readable = [this](uint32_t address) {
      auto heap = memory_->LookupHeap(address);
      return heap && heap->QueryRangeAccess(address, address + 48) !=
                         xe::memory::PageAccess::kNoAccess;
    };
    if (buffer_ptr) {
      for (uint32_t i = 0; i < 8; ++i) {
        uint32_t word = xe::load_and_swap<uint32_t>(buffer + i * 4);
        words += fmt::format("{}{:08X}", i ? " " : "", word);
        // A word in the title's image may be a name, such as the resource or
        // task the message is about. Only a word whose pages are mapped can
        // be read: Fruit Ninja Kinect sends a word in that range that isn't,
        // and reading it crashed Xenia.
        if (word >= 0x82000000 && word < 0x90000000 && readable(word)) {
          auto text = memory_->TranslateVirtual<const char*>(word);
          std::string name;
          while (name.size() < 48 && text[name.size()] >= 0x20 &&
                 text[name.size()] < 0x7F) {
            name += text[name.size()];
          }
          if (name.size() >= 3 && !text[name.size()]) {
            names += fmt::format(", word {} names \"{}\"", i, name);
          }
        }
      }
    }
    uint32_t caller = 0;
    if (XThread::IsInThread()) {
      caller = static_cast<uint32_t>(
          XThread::GetCurrentThread()->thread_state()->context()->lr);
    }
    XELOGE(
        "Unimplemented XAM message app={:08X}, msg={:08X}, arg1={:08X}, "
        "arg2={:08X}, called from {:08X}, words at arg1: {}{}{}",
        app_id(), message, buffer_ptr, buffer_length, caller,
        words.empty() ? "none" : words, names,
        seen == 3 ? " (not logged again)" : "");
  }
  return X_E_FAIL;
}

// Says in the log whether the title's own skeleton tracker sees anyone, so a
// tester's log shows why nobody was announced: the first frame with a
// tracked skeleton, and every 300 frames (about 10 s) how many had one, the
// first 12 times.
void XamApp::ReportTracking(const uint8_t* frame) {
  constexpr uint32_t kSkeletons = 6;
  constexpr uint32_t kSkeletonBytes = 448;
  constexpr uint32_t kSkeletonTracked = 2;
  constexpr uint32_t kFramesPerReport = 300;
  constexpr uint32_t kReports = 12;
  uint32_t tracked = 0;
  for (uint32_t i = 0; i < kSkeletons; ++i) {
    if (xe::load_and_swap<uint32_t>(frame + 48 + i * kSkeletonBytes) ==
        kSkeletonTracked) {
      ++tracked;
    }
  }
  std::lock_guard<std::mutex> lock(tracking_mutex_);
  if (tracked && !tracking_seen_) {
    tracking_seen_ = true;
    XELOGI("XamApp: the title's skeleton tracker found {} player(s), {} "
           "frames in",
           tracked, tracking_frames_total_);
  }
  ++tracking_frames_total_;
  ++tracking_frames_;
  tracking_frames_with_someone_ += tracked ? 1 : 0;
  if (tracking_frames_ == kFramesPerReport) {
    if (tracking_reports_ < kReports) {
      ++tracking_reports_;
      XELOGI(
          "XamApp: skeleton report {}/{}: the title tracked someone in {} of "
          "the last {} frames",
          tracking_reports_, kReports, tracking_frames_with_someone_,
          kFramesPerReport);
    }
    tracking_frames_ = 0;
    tracking_frames_with_someone_ = 0;
  }
}

// With motion_sign_in_first_user, says that the first tracked skeleton in a
// 0x2B002 frame is a signed-in player, as a console's XAM does once it has
// recognised someone's face (docs/kinect-pscam.md section 14). The frame
// holds the title's 6 skeletons at +48, 448 bytes each: tracking state at
// +0 (2 = tracked) and tracking id at +4. The title's callback takes 0x2C006
// with {tracking id, identity entry, flags}; with flags 0 it picks a free
// entry itself. Each new tracking id is announced once.
void XamApp::AnnounceIdentity(const uint8_t* frame) {
  const uint32_t callback = identity_callback_;
  if (!cvars::motion_sign_in_first_user || !callback ||
      !kernel_state_->xam_state()->GetUserProfile(uint32_t(0))) {
    return;
  }
  constexpr uint32_t kSkeletons = 6;
  constexpr uint32_t kSkeletonBytes = 448;
  constexpr uint32_t kSkeletonTracked = 2;
  uint32_t tracking_id = 0;
  bool still_there = false;
  for (uint32_t i = 0; i < kSkeletons; ++i) {
    const uint8_t* skeleton = frame + 48 + i * kSkeletonBytes;
    if (xe::load_and_swap<uint32_t>(skeleton) != kSkeletonTracked) {
      continue;
    }
    const uint32_t id = xe::load_and_swap<uint32_t>(skeleton + 4);
    if (id == identified_tracking_id_) {
      still_there = true;
    } else if (!tracking_id) {
      tracking_id = id;
    }
  }
  if (still_there || !tracking_id) {
    if (!still_there) {
      identified_tracking_id_ = 0;  // they left; announce whoever comes next
    }
    return;
  }
  identified_tracking_id_ = tracking_id;
  if (!identity_block_) {
    identity_block_ = memory_->SystemHeapAlloc(16);
  }
  const uint32_t block = identity_block_;
  auto* words = memory_->TranslateVirtual<xe::be<uint32_t>*>(block);
  words[0] = tracking_id;
  words[1] = 0;  // identity entry: unused with flags 0
  words[2] = 0;  // flags
  XELOGI(
      "XamApp: told the title's Kinect identity callback that skeleton {:08X} "
      "is the signed-in player",
      tracking_id);
  // On the kernel's dispatch thread, a guest thread of its own: the title
  // sends 0x2B002 from its identity worker, which may hold the locks the
  // callback takes.
  auto kernel_state = kernel_state_;
  kernel_state_->Dispatch([kernel_state, callback, block]() {
    auto thread = XThread::GetCurrentThread();
    if (!thread) {
      return;
    }
    uint64_t args[] = {0x0002C006, block};
    uint64_t result = kernel_state->processor()->Execute(
        thread->thread_state(), callback, args, xe::countof(args));
    XELOGI("XamApp: the identity callback answered {:08X}", uint32_t(result));
  });
}

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace xe
