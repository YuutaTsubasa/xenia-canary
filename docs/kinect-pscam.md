# PsCamDeviceRequest: how a title's NUI library uses the Kinect camera driver

Our own tracing notes for the guest-facing camera driver (design:
[motion-capture.md](motion-capture.md) §6.1). Nothing here comes from other
Xenia Kinect branches; every fact was read from a title's code.

**Scope and caveats**

- Traced in **one title so far: Sonic Free Riders (5345084D)**, from its
  static recompilation (read locally; game code is never committed). Other
  titles may link another XDK version of the NUI library: re-check codes and
  layouts in a second title (Kinect Adventures is the natural next one)
  before relying on details.
- Addresses (`sub_8276xxxx`, `G = 0x82B5D2B0`, ...) are this title's, given
  so each fact can be found again; the driver must not depend on them.
- `[seen]` = read directly from the instructions; `[inf]` = inferred from how
  values are used; `[?]` = a guess.

Source: `Free-Riders-Recompiled\out\recomp\ppc\ppc_recomp.74.cpp` / `.75.cpp` (static recompilation), read only.
Confidence tags: **[seen]** = read directly from the instruction stream; **[inf]** = inferred from how the values are used; **[?]** = guess.

All offsets are byte offsets, all fields big-endian u32 unless stated otherwise.

## 0. How the driver is reached

* `sub_82769508` is **not called directly by anything**. Its address is stored into a global function pointer **`DevReq` at 0x82AE8C7C** by `sub_8276B938`, but only when `XamXStudioRequest(1, &out)` succeeds (Kinect Studio present). [seen]
  * The static initial value of 0x82AE8C7C is in .data and not visible in the recomp. Since every request goes through this pointer, it must default to the `PsCamDeviceRequest` import thunk 0x82ACC27C. [inf]
  * `sub_82769508(block)`: calls `XamXStudioRequest(4099, {block, out})`. If that returns >= 0, it returns `out` (the second word) as the NTSTATUS. Otherwise it calls `PsCamDeviceRequest(block)` and returns its result. [seen]
* **Signature: `NTSTATUS PsCamDeviceRequest(void* block)`. One argument; the return value is an NTSTATUS.** [seen]
  * `0x103` (STATUS_PENDING) means the request was queued, and completion comes later through a callback stored in the block. [seen: every async caller compares r3 with 259]
  * `>= 0` with any other value means it completed synchronously. `< 0` means failure. [seen]
* There is a tail-call trampoline `sub_82769F88` (`jump *DevReq`) used by the camera-settings code in `.75.cpp`. [seen]
* `sub_82769F98` is a **synchronous wrapper** for async-capable codes [seen]:
  * It writes `block+16 = &ctx` and `block+20 = 0x82769F70`. Here `ctx` is a 16-byte KEVENT (NotificationEvent, unsignalled) followed by one u32 status slot.
  * It calls `DevReq`. If the result is 259, it calls `KeWaitForSingleObject(&ctx.event, UserRequest, KernelMode, FALSE, NULL)` and returns `ctx.status`. Otherwise it returns the immediate result.
  * Callback `0x82769F70(block, NTSTATUS status)` loads `ctx = block+16`, stores `status` to `ctx+16`, then calls `KeSetEvent(ctx, 1, FALSE)`. [seen]
* **Completion callback convention:** `callback(block, NTSTATUS status, u32 bytesTransferred)`, with r3 = block, r4 = status, r5 = byte count (only the frame callbacks read r5). The context the callback wants is `*(block+16)`. [seen in D490/D838/A4D8/F70]
  * The kernel/driver must call it at an IRQL where the callback can take a spinlock: it uses `KfAcquireSpinLock` and `KeSetEvent`. [inf]
* Global NUI state struct **G = 0x82B5D2B0** (`lis -32074, addi -11600`) is used below as `G+n`. [seen]

## 1. Generic request block header

| off | size | meaning | evidence |
|---|---|---|---|
| +0 | u32 | **request code** | every caller [seen] |
| +4 | u32 | 1st parameter or reply word (code-specific) | codes 0, 13, 24, 30 [seen] |
| +8..+15 | | code-specific | |
| +16 | u32 | completion **context** pointer (async codes) | 5, 15, 35; sync wrapper [seen] |
| +20 | u32 | completion **callback** function pointer (async codes) | 5, 15, 35; sync wrapper [seen] |
| +24.. | | code-specific payload | |

Notes:
* +16 and +20 have this meaning only for codes that complete asynchronously. Code 30 uses +16 as an ordinary pointer parameter. [seen]
* Callers do not zero unused fields of most blocks (codes 27, 33, 2 leave stack garbage). The driver should only read the fields listed below. [seen]

## 2. Request-code table

| code | purpose (guess) | sync/async | block contents written by caller | reply consumed | seen at |
|---|---|---|---|---|---|
| **0** | get device state | sync | +0=0, +4=0 | +4 = state enum (see §3) | D160, D3F8, D838 (error path) |
| **2** | stop streaming | via sync wrapper (cb/ctx at +16/+20) | +0=2 | status only (ignored) | D3F8 |
| **5** | read one frame (depth or colour), queued | **async, returns 259** | §4 | callback(block, status, bytes) | slots built in E628; queued in A018/A100 (depth), A2E0/A3C8 (colour) |
| **11** | read calibration, old format | sync | +0=11 (block ~4.2 KB on stack) | §6 | C788 |
| **13** | get firmware/protocol version | sync | +0=13 (block 32 bytes) | u8 at +4 = major, u8 at +5 = minor; if >= 5.1, use code 14, else code 11 | CFA8 |
| **14** | read calibration, new format | sync | +0=14 (block >= 4272 bytes) | §6 | C9B0 |
| **15** | set frame timing / phase (sync of camera to console clock) | async, cb 0x8276A4D8 | §5 | callback status | A570 (called from D160 and the A650 PLL) |
| **24** | read 8192-entry u16 lookup table (raw depth to depth unit) | sync | +0=24, +4=1, +8=ptr to 16384-byte buffer (G+8760), +12=8192 (entry count) | the buffer; on failure the library fills an identity table 0..8191 | CFA8 |
| **25** | get timing info | sync | +0=25, +4..+59 zeroed (60-byte block) | 14 u32 at +4..+56 = microsecond values; each is multiplied by 50 (to 50 MHz timebase ticks) and stored to G+1160..G+1212. `G+1192` (= reply +36) must be non-zero or timing sync is skipped. | E58 (called by D160) |
| **27** | start streaming | via sync wrapper | +0=27, +28 = `1 | (initFlags & 0x40 ? 2 : 0)` | status < 0: fail init | D160 (only when state == 4) |
| **30** | unpack 11-bit packed depth in place | sync | +0=30, +4=src ptr (frame buffer), +8=422400 (0x67200), +12=dst ptr (same buffer), +16=ptr to u32 dst length, value 614400 on input | must return >= 0 and leave `*len == 614400`; otherwise the frame is dropped and requeued | D6C8 |
| **33** | unknown enable/config | sync, result ignored | +0=33, +28=1 | none | CFA8 |
| **35** | set colour-camera settings (exposure, white balance etc.) | async, cb 0x827788F0, block lives at G+2484 | +0=35, +20=cb, +28..+119 = 92-byte settings struct, then +40 (settings+12) negated in place | callback reads **block+24** (a sequence/generation number written by the driver [inf]) and stores it to G+2612/G+2616 and a table at G+2080. Failure 0xC00000A3 or 0xC000009D means give up; other failures are retried. | 75: 827782A0, 82778B48, 82778CA0 (after code 27), 827788F0 |
| **36** | get colour-camera settings | via sync wrapper | +0=36 (block >= 96 bytes, frame 224) | +28 low nibble (flags), +76/+80/+84/+88 (u32 x4), +92 (u32) | 75: 827783B8, 827784E8 |

`XamXStudioRequest` codes for reference [seen]: 1 = probe and install hook (B938), 769 = NUI init done (F640), 770 = NUI shutdown (B998), 4099 = forward device request, 4100 = per-frame flags query inside the frame callbacks, 5121/5122/5378/6 = elsewhere (not examined). On real hardware without Studio, these fail and the library ignores them.

## 3. Device state (code 0 reply at +4)

Taken from the `bdz` switch in D160 (`NuiInitialize` path and the worker thread) and the error path in D838 [seen]. The meanings are guesses.

| state | D160 (arg = "allow start") | D838 error path | guess |
|---|---|---|---|
| 0, 1 | not present, return 0 | nothing | no device |
| 2 | present + streaming → return 1 | present + streaming | **running** |
| 3, 5 | present, return 0 | present, not streaming | busy / transitioning |
| 4 | present. If arg≠0, run the **start sequence** (§7) and return 1 on success | present, not streaming | **idle, ready to start** |
| 6, 7 | control goes to 0x8276D3F8 (stop routine) [? looks like a switch case landing on the next function's entry; uncertain] | clear "streaming" flag | error / needs reset |

* D160 writes `G+88` = present and `G+92` = streaming. [seen]
* The stop routine D3F8 queries code 0 again and sends **code 2** when state is 2, 3, 6 or 7. [seen]

## 4. Frame read (code 5) — the hot path

### Slots

* There are 4 depth slots at `G+128 + i*128` and 4 colour slots at `G+648 + i*128`, built once by `sub_8276E628`. [seen]
* The request block sits at **slot+8**, so `block = slot+8` and `slot+N = block+(N-8)`.

| block off | slot off | depth value | colour value |
|---|---|---|---|
| +0 | +8 | 5 | 5 |
| +4 | +12 | not set (0 after memset) — maybe driver status [?] | same |
| +16 | +24 | context = slot base | same |
| +20 | +28 | callback **0x8276D490** | callback **0x8276D838** |
| +24 | +32 | **4** (stream selector: depth) | **2** (stream selector: colour) |
| +28 | +36 | buffer ptr = depthPool + i*0x96000 | buffer ptr = colourPool + i*size |
| +32 | +40 | buffer length **614400 (0x96000)** | buffer length `size` = `bpp * 307200` |
| +40 | +48 | ptr to frame-info output = slot+72 (= block+64) | same |
| +44 (u8) | +52 | – | colour: settings generation byte (read by D838; the driver must write it [inf]) |
| +64 | +72 | **frame number** (u32), written by driver via the +40 pointer [inf] | same |
| +92 | +100 | buffer ptr (library copy) | same |
| +96 | +104 | buffer offset inside pool | same |

* Depth pool: one allocation of 0x258000 = 4 × 614400 at `G+104`. [seen]
* Colour pool: allocated at `G+100`, 4 × `size`. [seen]
  * `size` is stored at 0x82B5D294 and equals `bpp × 0x4B000`.
  * `bpp` is stored at 0x82B5D298: 2 if init flag 0x40 is set, else 1.
  * Width 640 is at 0x82B5D2A0 and height 480 at 0x82B5D29C. [seen]
  * So colour is **640x480, 1 byte/pixel (307200 = Bayer/mono?) or 2 bytes/pixel (614400 = YUV422?)**. The format is not visible in this layer. [inf]
* Slot +0 is the library's own flags word, not part of the block [seen]:
  * 1 = in flight
  * 2 = held by app
  * 4 = completed
  * 0x20 = recalibrate requested
  * 0x40 = Studio flag
  * 0x80 = depth arrived packed

### Queuing

* `sub_8276A100` queues the 4 depth slots once. `sub_8276A3C8` queues the 4 colour slots, but only if a colour pool exists. Both run on the worker thread after the device reaches "streaming". [seen]
* Each request: set slot+0 = 1, atomically increment the in-flight counter (`G+112` depth, `G+640` colour), call `DevReq(slot+8)`. [seen]
  * 259 → OK, pending.
  * 0 → also accepted.
  * < 0 → counter decremented. When the counter reaches 0, it signals event `G+1560`.
* After the app has consumed a frame, the slot is requeued one at a time by `sub_8276A018` (depth) / `sub_8276A2E0` (colour), with the same block unchanged. So **the driver must not modify +0/+16/+20/+24/+28/+32/+40**. [seen/inf]
* Shutdown flag `G+24` makes requeue fail with 0xC000004B. [seen]

### Depth completion `sub_8276D490(block, status, bytes)` [seen]

1. `rec = *(block+16)`. Write `rec+4 = status`.
2. Check the byte count, if status >= 0:
   * `bytes == 614400` → normal.
   * `bytes == 422400` (0x67200, 640x480 × 11 bits) → set flag 0x80 (the consumer later issues **code 30** to unpack in place).
   * anything else → `rec+4 = 0xC0000206`.
3. `rec+88 = rec+72` (frame no.), `rec+92 = mftb`, `rec+96 = G+1216`.
4. If OK:
   * Call XStudio 4100 (ignored on hardware).
   * Mark the record current (`G+124`) and count dropped frames from frame-number gaps (`G+8952`, `G+8956`).
   * **`KeSetEvent(G+36)`**, which wakes the worker thread.
5. If the status failed: run D160 (state re-check) and requeue.
6. Decrement `G+112`. When it reaches 0, signal `G+1560`.

### Colour completion `sub_8276D838(block, status, bytes)` [seen]

* Same idea: the byte count must equal `size` at 0x82B5D294, else 0xC0000206.
* Then it matches the byte at block+44 (settings generation) against the code-35 history to tag the frame with the 12-byte settings that were active (rec+108..+120).
* The frame becomes current at `G+644`. No event is set; the worker pairs it with the next depth frame (`sub_8276E0F8` → `sub_8276A200`).
* On failure: code 0 state check (§3), then requeue.
* Decrement `G+640`.

### Consumer side [seen]

* Worker thread `sub_8276EF90` is created by `ExCreateThread` in F640, with start address `0x8276EF90`.
* It waits with `KeWaitForMultipleObjects(3, {G+52, G+36, G+68}, WaitAny)`.
* On `G+36` (depth arrived), `sub_8276EDD8` runs:
  1. Takes the current depth record (D6C8, which also issues code 30 if the 0x80 flag is set).
  2. Rebases the frame number.
  3. If flag 0x20 is set, re-runs calibration (CFA8).
  4. Hands `rec+100` (the buffer) plus the code-24 table `G+8760` to the tracker `sub_8278E5A0`.
* Colour is paired from `G+644`.

## 5. Code 15 timing block (`sub_8276A570(a, b, c, d, e, ctx)`) [seen]

* The block is embedded at ctx+356, where ctx = G+1160. The caller clears 44 bytes first.
* A busy flag at ctx+292 rejects overlapping calls with 0xC0000001.

| off | value |
|---|---|
| +0 | 15 |
| +16 | ctx |
| +20 | 0x8276A4D8 |
| +24 (u16) | a — `(streamMask-1)`; D160 passes 2 [inf: 0 = depth, 1 = colour, 2 = both] |
| +28 (u32) | b converted from 50 MHz ticks to µs (`b*1000000/50000000`) |
| +32 (u16) | c |
| +36 (u32) | d, ticks → µs |
| +40 (u16) | e |

* In D160: a=2, b=G+1160, c=0, d=G+1180, e=0. These values come from the code 25 reply words +4 and +24, which look like the depth and colour frame periods. [inf]
* The library's nominal period is 1666666 ticks = 1/30 s. [seen constant 0x196E6A]
* `sub_8276A650` is a phase-locked loop that later sends periodic code 15 adjustments. [seen]
* Callback A4D8: if status >= 0, it copies two pending 52-byte state blocks to their active slots. It always clears the busy flag. [seen]
* **An emulator can complete code 15 immediately with STATUS_SUCCESS, calling the callback with status 0.** If it returns >= 0 without calling the callback, the busy flag (cleared only by the callback or by a < 0 return) stays set forever, and later adjustments are refused. **Call the callback.** [inf]

## 6. Calibration outputs

### Code 13 → version check
u8 at +4 = major, u8 at +5 = minor. If major >= 5 and minor >= 1, the library uses code 14; otherwise it uses code 11. [seen, CFA8]

### Code 11 (old format)
Block at sp+112, frame 4352. [seen, C788]

* +8..+95: 12 doubles, converted to floats.
  * The first 6 go to G+8584..G+8604; the second 6 go to G+8624..G+8644. Two 6-parameter sets (two cameras' intrinsics?) [inf].
  * G+8568 and G+8608 (40 bytes each) are zeroed first.
* +104: double, multiplied by a constant and converted to a 64-bit fixed-point scale.
* +112..+4207: u16[2048]. Each non-zero entry < 8192 is a knot. The library linearly interpolates between knots to fill the 8192-entry u16 table at G+8648.
* +4212: u16 used as an offset in that interpolation.
* Afterwards it builds texture G+8652 from G+8648.

### Code 14 (new format)
Block at sp+304, frame 4704. [seen, C9B0]

* +12, +16, +20, +24: s32.
* +28: 19-bit signed (extracted with shift-13).
* +32, +36, +40, +44: s32.
* +48: 19-bit signed.
* +76, +80, +84, +88: 21-bit signed (shift-11).
* +92, +96, +100, +104, +112, +120: 27-bit signed (shift-5).
* +4232, +4240, +4256, +4264: doubles.
* +4248, +4250: u16. +4248 is divided by 640.

These feed two calls of a 3x3/affine solver (`sub_82769CC8`). The results land as floats in G+8568..G+8644 (two 10-float sets). The same doubles generate the 8192-entry G+8648 table analytically.

* Then G+8704[i] = G+8648[G+8760[i]] if 0 < G+8760[i] < 4000, else 0x8000. Textures G+8652/8708/8764 are rebuilt. [seen]
* **The packed fixed-point registration fields look like the Kinect "registration info" set.** An emulator needs real values or plausible defaults here. Failing code 13 (major < 5) selects the simpler code 11 path. [inf]

### Code 24 → G+8760 (u16[8192])
* The table index is a raw depth sample of up to 13 bits.
* Values < 4000 are treated as valid (mm?). [inf]
* If code 24 fails, the library uses an identity table. In that case the depth buffer should already contain the final unit, < 8192. [inf]

## 7. Init / teardown sequence

`sub_8276FD88` = NuiInitialize (outer: opens content, then calls the inner `sub_8276F640(flags, ...)`).

1. **F640** [seen]:
   1. Zero G (10464 bytes).
   2. G+0 = init flags, G+4 = arg.
   3. Two default code-35 settings structs (`sub_82778160`) at G+1804 / G+2192.
   4. **B938**: XStudio probe; if present, swap `DevReq`.
   5. Create events, then **E628** (allocate pools, build the 4+4 code-5 slots).
   6. `ExCreateThread(worker EF90, suspended)`.
   7. Allocate 3 × 16 KB tables (G+8648, G+8704, G+8760) and fill G+8760 with identity.
   8. **`D160(0)`** = **code 0** state query only (no start).
   9. Resume thread, then XStudio 769.
   10. NuiInitialize succeeds even if no device is present; state is stored in G+88/G+92.
2. **Worker EF90** loop [seen]:
   1. While not streaming (`G+92==0`): call **`D160(1)`**. If that returns 0, wait 30 ms on G+20 (`NtWaitForSingleObjectEx`, timeout −300000).
   2. `D160(1)` with state 4 runs the start sequence:
      1. **code 27** (start; +28 stream mask)
      2. `sub_82778CA0` → **code 35** (async colour settings, idx = G+1680)
      3. **CFA8**:
         * **code 24** (depth LUT)
         * **code 33** (+28=1)
         * **code 13** (version)
         * **code 14 or code 11** (calibration)
      4. take spinlock G+1456
      5. **code 25** (timing). If OK and the reply's +36 word ≠ 0, set up the PLL constants, then **code 15** (initial timing).
      6. Set G+88 = 1, G+92 = 1.
   3. Then **A100**: queue 4 × code 5 depth. **A3C8**: queue 4 × code 5 colour.
   4. Main wait loop on {G+52, G+36, G+68}.
3. **Shutdown** `sub_8276B998` [seen]:
   1. XStudio 770.
   2. Set G+24 = 1, NtSetEvent(G+20), KeSetEvent(G+52), and wait for the thread.
   3. The worker exits its loop and calls **D3F8**: **code 0**, and if the state is 2/3/6/7, **code 2** (stop).
   4. It then waits on G+1560 until **both in-flight counters (G+112, G+640) reach 0**. **The driver must complete (or fail) every outstanding code-5 request after code 2, or shutdown hangs.** [inf from the wait loop at 0x8276F5BC]

## 8. Detroit (tilt) / Mca (mic)

* `DetroitDeviceRequest`, `McaDeviceRequest` and `UsbdGetNatal*` are **not imported** by this title (not in `ppc_func_mapping.cpp`). [seen]
* Tilt is done through `XamNuiCameraElevation*`, `XamNuiCameraTilt*` and `XamNuiCameraRememberFloor`.
* Mic is done through `XamVoiceGetMicArrayAudioEx`.
* These are XAM-level and are not examined here.

## 9. Minimal emulation recipe (derived)

1. **code 0** → write +4 = 4 the first time (idle), then 2 after code 27. Return 0.
2. **27, 33, 2** → return 0. For 27/2 (sync-wrapper codes), either return 0 directly, or return 259 and later call `cb(block, 0, 0)`.
3. **13** → +4 = 4, +5 = 0. This forces the code-11 path; supply a plausible code-11 blob, or fail code 11 (then calibration floats stay zero, which is risky) [?].
4. **24** → fill the table, or return failure to get the identity table.
5. **25** → fill 14 µs words. A zero at +36 skips timing sync, so code 15 is never sent from D160. The safest route is to fill +4 = +24 = 33333 and +36 = 0. [inf]
6. **5** → keep the block. On every new frame, write the frame number to `*(block+40)` (and for colour, the byte at block+44). Copy pixels to `*(block+28)` (length `*(block+32)`). Call `(*(block+20))(block, 0, length)` from a guest thread. Return 259 at submit time.
7. **30** → only needed if 422400-byte packed depth is delivered. Avoid it by delivering 614400.
8. **35** → return 259, then call `cb(block, 0, 0)` after writing a generation byte to block+24. **36** → fill per §2.

## 10. Open questions

* ~~The exact meaning of the code-5 depth pixel values~~: answered in §11.
* Does the driver write anything at block+4 (code 5) or block+24 (code 35), and how big is the frame-info struct at `*(block+40)`? Only +0 (frame number) is read for depth.
* The colour pixel format for bpp 1 vs 2. Which init flag corresponds to 0x40 in the public NUI flags?
* The full code 25 reply semantics (only +4, +24, +36, +44, +52, +56 are used directly).
* What code 33 is, and what the other fields of codes 27 and 33 mean; callers leave them uninitialised.
* The D160 state 6/7 switch target 0x8276D3F8 coincides with a function entry. Is it a recompiler boundary artefact?
* The 92-byte code-35 settings struct layout (defaults: +4 = 1/2, +24 = 1, +32/+36 = 0x1999, +40/+44 = 0xCCCC, +48/+52/+56 = 0x10000, +68 = XGetVideoMode-derived), and the meaning of the negated field at +12.
* The initial value of `DevReq` (0x82AE8C7C) in .data, assumed to be 0x82ACC27C.

## 11. Depth pixel format (what the tracker reads)

Second tracing pass (2026-11-18): follows the depth buffer from the worker
into the skeleton tracker to find what the driver must write. Same title,
same rules as above.

Source: `Free-Riders-Recompiled/out/recomp/ppc/ppc_recomp.{74,77}.cpp`. G = 0x82B5D2B0.
Tags: [seen] = read directly in the code, [inf] = inferred, [?] = not determined.
Note: G+8648 / G+8704 / G+8760 are *pointer fields* to the 16 KB u16[8192] tables (`lwz r4,8760(r30)` in EDD8). Below, `T24` = `*(G+8760)` (code-24 table), `T11` = `*(G+8648)`.

### Call chain (depth path)

| step | function | what it does |
|---|---|---|
| 1 | `sub_8276EDD8` | passes r3 = depth rec+100 (640x480 u16 buffer), r4 = T24, r5 = sp+80 (two u32 "tracked skeleton IDs" copied from G+10164/10172), r6 = buffer of record at worker+280, r7 = output area `[G+8816]+112 + worker+284`, r8 = worker, r9 = worker+32 [seen] |
| 2 | `sub_8278E5A0` | null-checks the buffer and the tracker object `*(0x82B60E98)`, then forwards to F5B0 [seen] |
| 3 | `sub_8278F5B0` | stores the depth pointer at obj+0x10000+1692; optional debug hook object `*(0x82B60E9C)` gets (buf, 640, 480, T24) via vtable[0]; then calls `sub_8278EC88` [seen] |
| 4 | `sub_8278EC88` | skeleton-ID bookkeeping, then calls `sub_8279ACD8(…, depth=r15, table=r14, …)` [seen] |
| 5 | `sub_8279ACD8` | per-frame pipeline; calls `sub_82793808(obj, depth, T24)` (0x8279B26C area) [seen] |
| 6 | **`sub_82793808`** | **the only place the raw depth pixels are read** in this path: 8x8 downsample + T24 lookup + 3D conversion [seen] |

The colour record (worker+276) is *not* passed into this chain (see Q6).

### 1. How each 16-bit pixel is interpreted (`sub_82793808`)

**Phase 1 — downsample 640x480 → 80x60, VMX** [seen]
* Reads the buffer as **big-endian u16** with `lvx`; per 64-pixel (128-byte) segment, splits even/odd halfwords (`vperm` with constant at 0x820C0FF0 and `vpkuwum`) and takes `vmaxuh` of each horizontal pair, then `vmaxuh` with the same pair in the next row → **max of each 2x2 block**. [seen; the vperm constant itself is not in the C++, so "even halfwords" is inf — it is the only choice that pairs with `vpkuwum`'s odd halfwords]
* Over 4 row-pairs (8 rows), keeps a running **`vminuh` of the 2x2 maxima, skipping any 2x2 max that equals 0** (`vcmpequh` with zero vector, `vsel` keeps the old value). Accumulator starts at the splat constant at 0x820C1000 (presumably 0xFFFF or 0x7FFF; either is ≥ 8192 → "invalid" later). [seen / constant ?]
* Then reduces horizontally with `vminuh` (vpermwi/vsldoi) so each 8x8 block becomes one value = **min over its 16 non-zero 2x2 maxima**. No masking (`& 0x7FF`), no shift, no byte swap is applied to pixels. [seen]
* Row loop: outer counter 59 + one tail row = 60 output rows, input pointer advances 8 rows (10240 bytes) per output row. [seen]
* Output u16[80*60] at obj+0x2EFF0 (row stride 160 bytes). Within a row, segment stores go to **decreasing addresses** (r31 −= 16, then +320 per row) and pass through a final `vperm` (constant at 0x820C1020) → **horizontally mirrored** 80-wide row. [seen store order; reversal within the 8-lane vector inf]

**Phase 2 — per cell, 60 rows x 80 columns** [seen, loop at 0x82793D5C..0x82793FB4]
For each downsampled value `v` (read and rewritten in place):
* `v == 32767` → stays 32767 (invalid).
* `v >= 8192` → 32767 (invalid).
* else `v = T24[v]`; if that is **0 → 32767** (invalid); if **> 3999 → 8000** (clamped "far"); otherwise kept.
* Then converts to a float 3D point stored in a 16-byte cell record (`obj+0x2C4F0`-ish base `addis 2,-15376`): X = (col*8+4 − C1)·k·z, Y = (C2 − (row*8+4))·k·z, Z = z (constants at 0x820C0FD0, 0x820BC16C, 0x820C0FE0; values not visible) [seen formula / constants ?]. Compares |ΔZ| with previous frame against 10 and 100, compares z against a per-cell background value scaled by 64226/65536 and 64881/65536, and sets flag bits at +12/+14. [seen]
* After the loop, if state field obj+0x31974 == 15, background cells that are 0 are set to 32767. Counter of "near" cells written to obj+0x33F28. [seen]

**Units:** the post-T24 value is compared against 3999/8000 and used directly as Z with motion thresholds 10 and 100 → **millimetres** [inf, strong].

### 2. Which table is raw → mm

* **T24 (code 24, `*(G+8760)`) is the raw → mm table.** It is indexed by the (downsampled) raw pixel value 0..8191, and its output is treated as mm with 0 meaning "no depth". [seen in 82793808]
* **T11 (`*(G+8648)`) is mm → registration shift**, not raw → mm:
  * `sub_82766FB8` (looks like NuiImageGetColorPixelCoordinatesFromDepthPixel; checks resolution arg == 2) indexes T11 with `(packedDepth >> 3) & 0xFFF` — i.e. the public packed format `mm << 3 | playerIndex` — and uses `T11[mm] >> 4` (arith.) as a horizontal pixel offset; it also mirrors x: `639 − 2*depthX` (depth coords at half resolution). [seen]
  * Code 11 path (`sub_8276C788`): for k = 0..2047, entry `e_k` (u16, 0 < e_k < 8192) is a depth value; T11 is filled as a step function over indices [prev_e, e_k) with value `((k << 20 … ) − offset<<22) * scale >> 40` — a value computed from k (the disparity index). So the code-11 blob is a **2048-entry disparity → depth** table, and T11 is its inverse turned into a shift. [seen; exact fixed-point semantics inf]
* G+8704 = T11[T24[raw]] (raw → shift) for 0 < T24 < 4000, else 0x8000 [from notes, CFA8]. Used only for registration textures (G+8708) [inf]; not used by the tracker path above.
* **If code 24 fails (identity T24)**, the pixel values must themselves be **mm**, with 0 = no reading; valid 1..3999; 4000..8191 become "far" (8000). [inf from the code above]

### 3. "No depth" value

* **0**: the cleanest "no reading". Zero pixels lose every 2x2 max; an all-zero 2x2 block is skipped entirely in the 8x8 min. An all-zero 8x8 block → accumulator constant → ≥ 8192 → 32767. [seen]
* **Any value ≥ 8192** (e.g. 0xFFFF, 0x7FFF): wins the 2x2 max, but loses the 8x8 min against any valid block; if a whole 8x8 block is ≥ 8192 → invalid. Acts like "infinitely far". [seen]
* **2047 / 0x7FF**: *not special in the code*. It is a normal table index; it is invalid only if `T24[2047] == 0`. With raw Kinect disparity, 2047 is the hardware "no reading" and T24[2047] is presumably 0 [inf]. With an identity T24 it would read as 2047 mm — wrong. [seen/inf]
* Since 2x2 uses **max** and 8x8 uses **min** (ignoring 0), values must **increase with distance** (true for both raw disparity-code and mm). [inf]
* No `& 0x7FF` / `& 0x1FFF` mask is applied; bits above 12 are not stripped, so a player index in the low 3 bits (`mm<<3|p`) is NOT expected here — the driver frame is **not** in the public packed format. [seen: no mask; inf: format]

### 4. Downsampling / orientation

* Tracker sees **80x60**, each cell = 8x8 source pixels, rule "min over non-zero 2x2 maxima". Not 320x240. [seen]
* Rows top → bottom in source order (no vertical flip); Y in 3D is computed as (C2 − y) i.e. up-positive. [seen]
* Columns are **mirrored** on output (decreasing store addresses + final vperm). [seen/inf] So the driver's 640x480 frame is expected in **sensor (unmirrored) order**, and the library mirrors it. Consistent with `sub_82766FB8` using `639 − 2x`. [inf]
* After tracking, `sub_8276EDD8` flushes/finishes 19200 bytes (= 80*60*4) at the output area, and 614400 bytes at the depth slot (`sub_82506EF8(start,end,1)`). [seen]

### 5. Code 30 bit order

Not visible: the library only issues code 30 (src, 422400, dst, &len=614400) and checks `*len == 614400`; it never touches the packed bits. [seen] Delivering 614400-byte unpacked frames avoids it. Expected unpacked result: one BE u16 per pixel holding the 11-bit value (0..2047) in the low bits [inf, from the 8192-index table and no masking].

### 6. Colour buffer

* The colour record goes to worker+276 (`sub_8276A200` pops it from G+644 and sets a flag via rlwimi) [seen]. It is not passed to `sub_8278E5A0`. [seen]
* `sub_8276E0F8` passes `depthSlotOffset >> 3` and `colourSlotOffset >> 2` (from rec+104) plus worker+284>>2 into `sub_8278E970` → `sub_82792970`, which only writes them into a command descriptor at `*(obj+112)` (obj = `*(0x82B60EA0)`) and flushes it with `sub_82506EF8`. The consumer of that descriptor (another thread or a GPU shader) was not found. [seen / consumer ?]
* No CPU demosaic (2x2 Bayer) or YUV unpack was found in the paths traced. The `>> 2` (4-byte units) for colour vs `>> 3` for depth hints the colour offset is used as a 32-bit-element index [?]. **Bayer vs YUV422 remains open.**

### Implications for an emulated driver

1. Code 5 depth: write **640x480 big-endian u16**, row-major, top row first, **not mirrored** (sensor order), 614400 bytes, so code 30 is never needed.
2. Pick one consistent encoding:
   * **Easiest:** fail code 24 (identity T24) and write **depth in mm** directly, 0 = no reading, clamp ≥ 4000 to something like 4000..8191 (treated as far) — or write 0 for "no reading". Keep values < 8192, no player bits, no shift.
   * **Hardware-like:** answer code 24 with a disparity→mm table (T24[2047] = 0) and write 11-bit raw disparity (2047 = no reading). Only worth it if real raw data is available.
3. Whatever is written must be monotonic increasing with distance; 0 must mean "no data".
4. Calibration (codes 11/14) only affects registration (depth↔colour mapping, T11/T8704 textures), not the tracker's depth values. A plausible code-11 blob (2048 knots, increasing) is enough; its effect on skeleton tracking was not seen in this path.
5. Useful range: 1..3999 mm (≈ 0.8–4 m typical); the tracker's own background logic compares against previous frames, so a stable background helps.

### Open questions

* Exact float constants k, C1, C2 (0x820C0FD0/0FE0, 0x820BC16C) and the accumulator init at 0x820C1000 / perms at 0x820C0FF0, 0x820C1020 — need the XEX data section (would confirm the mirror and focal length).
* The consumer of the `sub_82792970` descriptor (colour + depth offsets) — likely where colour (and maybe a full-res depth) reaches the GPU; decides Bayer vs YUV.
* What worker+280's record holds (an 80x60 u16 player-label map is processed from it in `sub_82796170`, neighbour offsets ±2 and ±160 bytes) [?].
* Whether the game itself (outside NUI) reads the 640x480 depth (e.g. for a silhouette display) and expects packed `mm<<3|player`; not traced.
