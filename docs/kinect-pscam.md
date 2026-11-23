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
  * `sub_82766FB8` (looks like NuiImageGetColorPixelCoordinatesFromDepthPixel; checks resolution arg == 2) indexes T11 with `(packedDepth >> 3) & 0x1FFF` (corrected in §12; first read as 0xFFF) — i.e. the public packed format `mm << 3 | playerIndex` — and uses `T11[mm] >> 4` (arith.) as a horizontal pixel offset; it also mirrors x: `639 − 2*depthX` (depth coords at half resolution). [seen]
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

## 12. Calibration replies (codes 13, 11, 14)

Third tracing pass (2026-11-18): what happens when calibration fails, how
the code 11 fields are used, and what a driver can safely answer. Same
title, same rules as above.

Title 5345084D, source `Free-Riders-Recompiled/out/recomp/ppc/` only. G = 0x82B5D2B0.
Tags: [seen] read in the code, [inf] inferred, [?] not determined.
`T11` = u16[8192] at `*(G+8648)`, `T24` = `*(G+8760)`, `T8704` = `*(G+8704)`.
`PolyX` = 10 floats at G+8568..G+8604, `PolyY` = 10 floats at G+8608..G+8644.

### Q1. Failure handling

### Inside `sub_8276CFA8` (calibration) [seen]
1. Code 24 (block sp+96). `< 0` → fills T24 with identity 0..8191. Result not propagated. Then the T24 texture (G+8764) is rebuilt (`sub_824F3CF0` / `sub_824F22A0`).
2. Code 33 (block sp+160, +28 = 1). Result ignored.
3. Code 13 (block sp+128, 32 bytes). Code 14 is chosen only if **status >= 0 AND u8 +4 >= 5 AND u8 +5 >= 1** (0x8276D098..0x8276D0AC). Anything else, **including a failed code 13**, goes to code 11. A failed 13 is harmless. (Quirk: version 6.0 also selects code 11, because the minor check is independent.)
4. `r30 = sub_8276C9B0()` or `sub_8276C788()`. This is the **return value of CFA8**.
5. Regardless of the result: it zeroes 10 words G+10420..G+10456 (this forces the GPU registration constants to be re-uploaded; see Q4). Then, depending on G+2624 / G+0 bit 31 / G+10160, it calls `sub_8278E558` → `sub_8278F498` on the tracker object `*(0x82B60E98)`. That function only resets tracker state (zeroes fields and resets 3 player slots). It reads no calibration.

### `sub_8276C788` (code 11) [seen]
* It calls DevReq. **If the status is < 0, it returns the status at once** (0x8276C7BC → 0x8276C994).
  * Nothing is zeroed. PolyX/PolyY keep whatever they held (zero after F640's memset of G). T11 is not memset.
* On success it always returns 0. No field is validated, so nothing in the blob can make it fail.

### `sub_8276C9B0` (code 14) [seen]
* Same shape: `< 0` → return the status, touching nothing. Success → return 0.

### Up through `sub_8276D160` (state 4, arg ≠ 0) [seen]
* The order is code 27 (via `sub_82769F98`), then `sub_82778CA0` (code 35), then `if (code27 < 0) fail`, then `CFA8`.
* **`CFA8 < 0` → jumps to 0x8276D3D8 with r30 = 0, r28 = 1.**
  * G+88 (present) = 1, G+92 (streaming) = 0, return 0.
  * Code 25 and code 15 are skipped, and the spinlock section is not entered.
  * **No code 2 is sent**, even though code 27 already started the stream.
* Worker `sub_8276EF90`: while G+92 == 0, it calls `D160(1)` and waits 30 ms. So the start is **retried every 30 ms**. What happens next depends on the code 0 state the driver reports:
  * **The driver still reports 4:** the full start sequence runs again (27, 35, 24, 33, 13, 11...) every 30 ms. If the failure is permanent, **streaming never starts** (`NuiInitialize` itself has already succeeded).
  * **The driver reports 2 after the earlier code 27:** D160 takes the state-2 branch (0x8276D1D4) and sets streaming = 1 **without ever having completed calibration, code 25 or code 15**. That works by accident: PolyX/PolyY = 0 and T11 = uninitialised allocation contents.
* The recalibration path in `sub_8276EDD8` (slot flag 0x20 → `bl 0x8276CFA8` at 0x8276EE74) **ignores the return value** [seen].
* **Conclusion:** answer code 11 with success. Failing it is either a 30 ms retry storm or an accidental, half-initialised start.

### Q2. Code 11 fields

The block is at sp+112 in a 4352-byte frame [seen].

**Size limit:** C788 saves r30, r31 and LR at old_sp−24/−16/−8, which is block+4216, +4224 and +4232. **The driver must not write past block+4215.** A 4216-byte (0x1078) reply is the maximum safe size [seen/inf].

| block off | type | use |
|---|---|---|
| +0 | u32 | 11 (written by the caller) |
| +4 | – | not read |
| +8..+48 | 6 × f64 `d0..d5` | `frsp` → G+8584, 8588, 8592, 8596, 8600, 8604 = PolyX c4..c9 |
| +56..+96 | 6 × f64 `d6..d11` | → G+8624..8644 = PolyY c4..c9 |
| +104 | f64 `ds` | `S = (s64) trunc(ds × K)`, where K is the f64 at **0x820991B0** (`lis -32246; lfd -28240`; value not in the C++) [seen/?] |
| +112..+4207 | u16[2048] `e[k]` | knots (depth value at disparity index k) |
| +4208..+4211 | – | not read |
| +4212 | u16 `off` | disparity offset |

Before the copy, G+8568 and G+8608 (40 bytes each) are zeroed. So **PolyX/PolyY c0..c3 (the cubic terms) are always 0 on the code-11 path** [seen].

### T11 construction (loop 0x8276C8C4..0x8276C928) [seen]
1. `memset(T11, 0, 16384)`.
2. Set `prev = 0` and `val = 0`.
3. For k = 0..2047, take `e = e[k]`. **If e == 0 or e >= 8192, skip it** (k still advances). Otherwise:
   * if `(u16)prev < e`: `T11[prev .. e-1] = val`
   * `prev = e`
   * `val = (s16)( ((((u64)k << 20) & ~0x3FFFFF) − ((u64)off << 22)) × S >> 40 )`
     * The mask makes `(k << 20) & ~0x3FFFFF` = `floor(k/4) << 22`.
     * So `val = (s16) floor((floor(k/4) − off) × S / 2^18)`.
4. Tail: if `(u16)prev < 2048`, set `T11[prev .. 2047] = val`. **The tail fill stops at 2048, not 8192.** Entries above the last knot stay 0 when the last knot is ≥ 2048.

Consequences:
* Knots must be **strictly increasing** to fill anything. A decreasing or repeated knot only updates `val`.
* `T11[0 .. e_first−1] = 0`, and `T11[e_k .. e_{k+1}−1] = val(k)`.
* **There is no division anywhere in C788.** `fctidz` of 0 is 0. `fctidz` of NaN/inf saturates without trapping. A zero blob is therefore arithmetically safe [seen].
* T11 values are horizontal shifts in **1/16 pixel** (used as `T11 >> 4`, below).
* Last step: the texture header G+8652 (8192×1, created over the T11 memory in F640 via `sub_825E0D38`/`sub_825E0E78`) is re-locked and unlocked. **T8704 is NOT rebuilt on the code-11 path.** Only C9B0 builds it. With code 11 it keeps its allocation contents (presumably zero under Xenia) [seen/inf].

### Polynomial `sub_82769C20(poly, x, y)` [seen]
* `u = x − Cx`, where Cx is the f32 at 0x820BC16C. **This is the same constant as C1 in the tracker's 3D conversion (§11).**
* `v = y − Cy`, where Cy is the f32 at 0x8200F808. Values not visible; probably 320 / 240 [?].
* `P = c0·u³ + c1·v³ + c2·u²v + c3·uv² + c4·u² + c5·v² + c6·uv + c7·u + c8·v + c9`
* No division.

### Who reads it
1. **`sub_82766FB8`** (CPU depth → colour pixel; checks resolution arg == 2; returns 0x80070015 / 0x8007048F if the device is not streaming / not present) [seen]:
   * Inputs: depth (dx, dy) at 320×240 (mirrored) and the packed depth word.
   * `x' = 639 − 2dx`, `y' = 2dy` (sensor coordinates at 640×480).
   * `shift = (s16)T11[(packed >> 3) & 0x1FFF] >> 4`. Note: **0x1FFF (13 bits)**, not 0xFFF as §11 says. The rlwinm mask covers bits 18..30.
   * `colourX = 2dx − shift − PolyX(x', y')`, `colourY = 2dy + PolyY(x', y')`.
   * Equivalently, in sensor coordinates: `colourSensorX = x' + shift + PolyX`, `colourY = y' + PolyY`.
   * Optional view-area (zoom) adjustment: `2·(cx − 160) − va[1]`, `2·(cy − 120) − va[2]`.
   * **Its only caller is `sub_827743E8`** (0x82775 area, the colour-camera settings module); see Q4. No `lis/addi` reference to 0x82766FB8 exists elsewhere [seen by grep].
2. **GPU registration pass:** `sub_8276DFC8` → `sub_827695D0` → `sub_8278E8C8` → `sub_827918B0` (on object `*(0x82B60EA0)`) → `sub_827911F8` [seen].
   * `sub_827911F8` copies the 10 floats of PolyX and PolyY plus the descriptors of the T11 texture (G+8652), the T8704 texture (G+8708) and the T24 texture (G+8764) into a 236-byte constant block, and sends it with `sub_824F8B30`.
   * `sub_827918B0` then issues D3D draw state. It is the same object that receives the colour/depth slot offsets (§11 Q6).
   * DFC8 re-uploads only when the 40-byte stream configuration differs from G+10420..G+10459 (which CFA8 zeroes).
3. **Tracker path: not used.**
   * There are no reads of G+8568..G+8648 / T11 in `sub_82793808`, `sub_8279ACD8`, `sub_8278EC88` or `sub_8278F5B0`.
   * A grep of ppc_recomp.73–79 for those G offsets finds only C788, C9B0, 82766FB8, 827695D0, F640 and B998 [seen].
   * The tracker reads only T24 [seen, §11]. CFA8's tracker call (`sub_8278F498`) is a reset that takes no calibration data [seen].

### Q3. Proposed code-11 reply

### A. Minimal: everything zero (recommended)
* Reply: block +4..+4215 all zero (12 doubles = 0.0, ds = 0.0, e[] = 0, off = 0). Return STATUS_SUCCESS.
* Result:
  * PolyX = PolyY = 0.
  * T11 is all zero: the memset clears it, every knot is skipped because e = 0, and the tail fill (`prev = 0 < 2048`) writes `T11[0..2047] = val = 0` [seen].
* Mapping: `colourX = 2dx`, `colourY = 2dy`. Colour and depth are pixel-aligned at 2× scale, with zero parallax.
* Safety:
  * No division in C788 or in `sub_82769C20`/`82766FB8`.
  * `S = trunc(0 × K) = 0`.
  * **No NaN, no inf, no trap.**
* This is exactly a "both cameras on the same optical axis, same intrinsics" pinhole model. The emulated driver produces depth and colour from the same source (or from a webcam plus a depth estimate), so this is also the physically correct answer for it.

### B. Kinect-like ~2.5 cm baseline (optional)
* Physics: shift(z) = f·b/z, with f ≈ 525 px (640-wide colour) and b ≈ 25 mm, so f·b = 13125 px·mm.
* The library model is `shift16 = (floor(k/4) − off)·S/2^18`, i.e. linear in a disparity index k. This works because raw Kinect disparity is linear in 1/z.
* Use a synthetic disparity with 1/z linear in k, with z from z_min = 400 mm to z_max = 8000 mm (< 8192):
  * `e[k] = round( 1 / (1/400 − k·(1/400 − 1/8000)/2047) )`. This gives e[0] = 400 and e[2047] = 8000, and is increasing.
  * Wanted shift (px, relative to z_min) = f·b·(1/400 − 1/z_k) = k · 13125 · 0.002375 / 2047 = **0.015228·k px** = **0.24364·k in 1/16 px**.
  * With off = 0: `floor(k/4)·S/2^18 ≈ (k/4)·S/2^18` → S/2^18 = 0.97457 → **S ≈ 255 476**.
  * So **ds = 255 476 / K**, where K is the double at guest 0x820991B0. K is unknown here, but the emulator can read it from guest memory at request time (title-specific).
  * PolyX c9 (d5) = −f·b/z_min = **−32.8 px**, so the total is `x_c = x_d − f·b/z` (≈ 0 at infinity, −32.8 px at 0.4 m). The sign depends on which side the RGB camera sits [?].
* Caveats:
  * Rounding makes many low-k knots equal (≈ 0.19 mm per step near 400 mm). Equal knots are harmless (no fill, val updated).
  * T11[8000..8191] stays 0 because the tail fill stops at 2048.
  * T11 below 400 mm is 0.
* **Not worth it unless the emulated colour image really comes from an offset camera.**

### Code 14 is not recommended
* C9B0 computes `f0 = K26 / ((u16@+4248 / 640) · u16@+4250 · d@+4240)` (0x8276CA60..0x8276CA70). Then per entry: `(i·f0 − c) / (i·f0)`.
* With zero or small fields (+4248 < 640 → integer quotient 0) you get **division by zero → inf/NaN**, plus the unseen solver `sub_82769CC8`.
* It also needs real Kinect registration fixed-point data. Avoid it by making code 13 select code 11.

### Q4. Does registration matter for gameplay?
* **Skeleton tracking: no** [seen]. The tracker uses only T24 and raw depth (§11). The calibration call into the tracker is a reset.
* **CPU mapping `sub_82766FB8`:** its only caller is `sub_827743E8` (colour-settings module, 0x8277xxxx).
  * It takes a skeleton-derived 3D point (the mean of three joint vectors × a constant at 0x820101B0), projects it to depth coordinates (`sub_82772210`), maps it to colour, and stores the offset from the image centre `2·(x−320)`, `2·(y−240)`, clamped to ±320/±240, at obj+116/+120 with obj+112 = 1.
  * This looks like a **metering / region-of-interest point for the colour camera's auto exposure / white balance** (code 35) [inf].
  * The game does not call `sub_82766FB8` directly [seen by grep].
* **GPU pass (`sub_827918B0`):** produces the registered colour/depth images (player video / cut-out) [inf].
* So registration affects only **how the player's camera picture lines up** (and where colour metering is aimed). Gameplay input does not depend on it [inf, strong].

### Recommended driver answers
| code | answer |
|---|---|
| 24 | fail (identity T24), as §11 says; depth in mm |
| 33 | return 0 |
| 13 | return 0 with +4 = 4, +5 = 0 (or just fail it). Never report ≥ 5.1 |
| 11 | return 0. Write **zeros** at block+4..+4215 (or leave the caller's block unwritten; but the caller does not clear it, so the stack contains garbage, so **write the zeros**). Never write at or beyond block+4216 |
| 14 | not reached; if implemented, return failure (it is only reached when 13 says ≥ 5.1) |

* Never fail code 11 permanently. That causes a 30 ms start retry loop, or a half-initialised start if state 2 is reported.
* **Important:** the caller does not memset the code-11 block. With a "return 0 without writing" stub, the 12 doubles, ds, knots and off are stack garbage. That can give NaN polynomials and junk T11, so the driver must fill the block.

### Open questions
* Values of K (0x820991B0), Cx (0x820BC16C) and Cy (0x8200F808). They need the XEX .rdata. Cx is shared with the tracker's C1.
* Why floor(k/4)? Old firmware's disparity knots may be in 1/4 or 1/8 units. Exact Kinect semantics are not determined.
* The shader that consumes the 236-byte constant block (T11/T8704/T24 textures + polys), and whether T8704 = 0 on the code-11 path changes the picture (real old-firmware Kinects take the same path, so presumably fine).
* The identity of `sub_827743E8`'s three input vectors (which joints) and whether obj+112..+120 really feeds code 35.
* G+2624 / G+10156 / G+10160 conditions for the tracker reset in CFA8.

## 13. Tilt motor and device status (XAM)

Fourth tracing pass (2026-11-18): how the same title uses the XAM tilt and
device-status imports (section 8 left them out). Same title, same rules
as above.

Source: `Free-Riders-Recompiled\out\recomp\ppc\ppc_recomp.74.cpp` (NUI library, 0x82764938..0x82766BC8), plus `.3/.39/.48/.92/.93.cpp` for the game-side device status calls. Read only.
Tags follow kinect-pscam.md: **[seen]** = read from the instructions; **[inf]** = inferred from how values are used; **[?]** = guess.
The recomp has no .rdata, so float constants are known only by address. They are listed in §12.

### 0. Layout of the library's tilt state

* `T` = 0x82B5D060 (`lis -32074, addi -12192`). `XamNuiCameraTilt*` init does a 512-byte memset of it. It sits just before `G` = 0x82B5D2B0 = T+0x250. [seen]
* **The XtSs block `B` = T+336 (0x82B5D1B0)**. It is persistent: the library writes the header before every GetStatus call and never clears the body. [seen]
* Other T fields used below [seen]:
  * T+36 = spinlock.
  * T+44 = auto-tilt-started latch.
  * T+48 = BOOL passed to RememberFloor.
  * T+60 = "floor search done" flag.
  * T+80 = operation state 0..8.
  * T+84 = frame countdown.
  * T+88 = retry counter.
  * T+92 = analysis counter.
  * T+96 / T+112 = two report records.
  * T+100 = target angle (s32).
  * T+380 = **cached copy of XtSs flags** (it is just B+0x2C).
  * T+416 = kind of system request (0 none, 1/2/3).
  * T+420 / T+424 = tick stamps. They are written but never read, so there is no time-based timeout.
  * T+436..463 = 28-byte XConfig blob.
  * T+464 = request flags.
  * T+468 = embedded request block.
  * T+496 = pointer to the pending request block.

### 1. XamNuiCameraTiltSetCallback

* **Called once**, from `sub_827658D8` (tilt init), which runs inside NuiInitialize-inner `sub_8276F640` (call at 0x8276FCF8, after the device init). [seen]
* One argument: r3 = **0x82765538** (`sub_82765538`). The return value is ignored. [seen]
* **What the callback is**: `HRESULT cb(u32 code, void* data)`. It is the channel through which **the system asks the running title to tilt** (for example a Kinect Tuner or Guide). It is not a "move finished" notification. [seen: switch on code; inf: purpose]
  * `code - 0x2B005` is used as a switch index 0..6. [seen]
    * **0x2B005**: `data` = `float*`. T+440 = T+104 = *data. Sets T+416 = 2 and starts an operation (`sub_82765028`). [seen]
    * **0x2B009**: no data. Sets T+80 = 7, which is a cancel: the next frame completes the operation with 995 (ERROR_OPERATION_ABORTED). Returns 0. [seen]
    * **0x2B00A**: `data` = `{u32 flags; float a; float b}`. Reads the XConfig, then sets T+416 = 1 and calls `sub_82765290(flags, tbl[flags&3]*C1, a, b, &T+128, &T+468)`, the full auto-tilt request. `tbl` = 4 floats at 0x82AE8C28. [seen]
    * **0x2B00B**: `data` = `s32*`. T+444 = T+100 = *data, the angle in degrees. Sets T+416 = 3 and starts an operation. [seen]
    * **0x2B006..0x2B008 and anything else**: returns 0x8000FFFF (E_UNEXPECTED). [seen]
  * If an operation is already running (T+80 != 0) for codes 0x2B005/0x2B00A/0x2B00B, it reports status 6 (busy) through ReportStatus (see §6) and returns 0x80004005. Otherwise it returns 0. [seen]
  * The callback zeroes T+468..T+496 before use and takes no lock. [seen]
* **Emulator consequence**: Xenia never needs to invoke the callback unless it emulates the system tilt UI. Store the pointer and ignore it. [inf]

### 2. XamNuiCameraTiltGetStatus

* Signature as used: `HRESULT/NTSTATUS GetStatus(void* blk)`, one argument. Before each call the caller writes blk+0 = **0x50** and blk+4 = **0x58745373 'XtSs'**. blk is always `B` (T+336). [seen]
* Call sites:
  * **`sub_82766230`**: the per-frame tilt state machine. It is called by the per-depth-frame worker `sub_8276EDD8` (at 0x8276EF00) whenever G+8 != 0, so **once per processed depth frame**. [seen]
  * **`sub_82765848`** ("get normal to gravity"). It is called:
    * per skeleton-frame build in `sub_8276C020` (0x8276C35C), for vNormalToGravity;
    * in states 3 and 6 of the state machine. [seen]
  * So roughly 2-3 calls per frame. [inf]
* Return value:
  * `sub_82765848`: if < 0, the output vector is left at its default (0, c@0x820015C0, 0, 0), most likely (0, 1, 0, 0) [?]. The return value is passed to the caller, but `sub_8276C020` ignores it. [seen]
  * `sub_82766230`:
    * While an operation runs (T+80 not 0 and not 6), `result < 0` **or flags & 0x8** fails the operation with Win32 31 (ERROR_GEN_FAILURE). [seen]
    * States 2, 4 and 6 additionally fail with 21 (ERROR_NOT_READY) on `result < 0`. [seen]
    * When idle, failure is harmless. [seen]
* **XtSs fields read** (only these; +0x08..+0x1F are never touched) [seen]:

| off | type | use |
|---|---|---|
| +0x00 | u32 | written by the title = 0x50 |
| +0x04 | u32 | written by the title = 'XtSs' |
| +0x20,+0x24,+0x28 | 3 × s32 gx, gy, gz | current gravity. See below. |
| +0x2C | u32 flags | 0x1 and 0x2 = moving (tested as `flags & 3`); 0x2 alone has an extra meaning; 0x4 = stalled; 0x8 = error; 0x10 = "force re-aim" (see below) |
| +0x30..+0x3C | float4 | **remembered floor plane** (nx, ny, nz, d). +0x3C (d) != 0.0 is the library's test for "a floor is remembered". |
| +0x40 | u32 | bit 0 set means the tilt is locked: the 0x2B00A request returns 5 (ERROR_ACCESS_DENIED) [seen]; meaning [inf] |
| +0x44,+0x48,+0x4C | 3 × s32 | gravity at the time the floor was remembered |

* **Gravity to vector** (`sub_82764F08`) [seen]:
  * len = sqrt(gx² + gy² + gz²). If len == c@0x8200EDB0 (≈0), the result is the zero vector.
  * Otherwise s = c@0x8200EDC0 / len (≈1/len), and the vector is **(-gx·s, gy·s, gz·s, c@0x8200092C)**, with w ≈ 0.
  * The scale of the s32 values is irrelevant, since only ratios are used. [inf]
* **Gravity to integer camera pitch** (start of `sub_82766230`) [seen]:
  * `a = atan2(gz, gy)` (`sub_82A54518(f1 = +0x28, f2 = +0x24)`).
  * `deg = trunc(floor(a · c@0x82041714 + c@0x82001C78))`. The constants are presumably 180/π and 0.5 [?].
  * This `deg` (r26) is compared with the target angle: when |target − deg| > c@0x8200EDE8 (double), SetAngle(target) is sent again. **So the accelerometer must agree with the commanded elevation, or the library keeps nudging** (a bounded number of times per state; see §5). [seen/inf]
  * A level camera evidently reads gx = gz = 0, gy > 0 (the default normal is (0, 1, 0)) [inf]. The sign of gz for an upward tilt is not determinable without the constants [?].
* Validation: the title checks only the HRESULT sign and the flag bits. It does not check the returned size or the signature. [seen]

### 3. vNormalToGravity / floor in the skeleton frame (Q8)

* `sub_8276C020` (skeleton frame build, frame at r27+32). The frame layout matches NUI_SKELETON_FRAME: +0 timestamp, +8 frame number, +12 flags, +16 vFloorClipPlane, +32 vNormalToGravity. [inf]
  * **+32 vNormalToGravity = `sub_82765848()`** = normalised (-gx, gy, gz, 0) from XtSs. [seen]
  * If the tracker found no floor (w == 0) **and G+0 & 0x800**, the floor is taken from **`sub_82765A80`**, and frame flags |= 2. [seen]
    * `sub_82765A80` takes the remembered plane B+0x30. Using the axis/angle between gravity-then (B+0x44) and gravity-now (B+0x20) (acos of the dot product, a cross product, and a 3x3 matrix from `sub_82765928`), it rotates the plane into the current camera pose and keeps d = B+0x3C. It returns 0 if the plane is zero. [seen]
  * The floor plane's d (frame+28) is then multiplied by c@0x82051624. The same constant scales floor height in ReportStatus, which suggests mm → m [?].
* NuiInitialize is called as `sub_8276FD88(27, 2)` from the game (ppc_recomp.0.cpp ~7590). If G+0 = 27, bits 0x800 and 0x04000000 are clear, so **the remembered-floor substitution is not used by this title, and the auto-tilt (§5) is enabled** [inf; G+0 = flags per kinect-pscam §7].
* The main skeleton tracker `sub_8278E5A0` receives no gravity argument [seen in EDD8].
  * Gravity reaches only the tilt helper `sub_8278E708` (player framing during auto-tilt). Its args are: depth buffer, &B+0x30, f1 = atan2 of the current gravity normal, f2 = atan2(B+0x4C, B+0x48) (`sub_82764938`), and a counter. [seen]
  * `sub_82764AA8` also calls it with a null buffer on completion. [seen]
* **While the motor moves (cached flags & 3), `sub_8276BCD0` resets the skeleton tracker every frame** (via `sub_8276BC58` → `sub_8278E558`). It is called from EDD8 each frame. **The emulator must not leave the moving bits set**, or tracking never starts. [seen]

### 4. XamNuiCameraElevationSetAngle

* One argument: r3 = **s32 angle in degrees**. The library wrapper `sub_82764C38` rejects |angle| > 27 with 0x80070057 without calling. [seen]
* Call sites:
  * via the wrapper: 2 calls in `sub_82765290` and 8 in `sub_82766230`;
  * direct: `sub_82765028` calls **SetAngle(−25)**. [seen]
* **The return value is ignored everywhere** (the wrapper returns it, and no caller reads r3). [seen]
* All callers are inside the NUI library: **the game itself never tilts**. Tilting happens only in the library's one-shot auto-tilt and in system requests. [seen]
* Before each call, T+420 = tick count (never read). [seen]
* Completion is detected by polling `XtSs` flags once per frame, not by the callback and not by time [seen]:
  * `flags & 3` → keep waiting (no limit);
  * `flags & 4` → fail with 25. The completion path maps 25 to report status 4 and calls **StopMovement** (see §6). In state 6 with request flag 0x20, it calls StopMovement and continues instead;
  * `flags & 8` → fail with 31.
* After motion stops: wait 15 frames (state 5), then analyse a frame, then possibly one corrective SetAngle (state 3), then wait again (state 6). [seen]

### 5. Auto-tilt flow (library-initiated, once)

* Trigger, in `sub_82766230` [seen]: G+8 != 0, not moving, and T+44 == 0. Under the lock:
  * if G+0 & 0x04000000, the latch is set and nothing happens;
  * else if T+80 == 0, it latches, prepares request block T+468, calls `sub_82765028` and reads the XConfig (`sub_82764E68`).
* `sub_82765028` sets state 2 and a 60-frame countdown. It calls **SetAngle(−25) and sets T+48 = 1** when flags & 0x10 is set, or when no floor is remembered (B+0x3C == 0) and T+60 == 0. [seen]
* `sub_82764E68` calls `ExGetXConfigSetting(7, 9, &T+436, 28, &sz)`. On failure the defaults are used. T+440 (float) is clamped from below; T+444 (s32) is used as an angle offset. [seen] (not a requested import, noted for completeness)
* State machine on T+80 [seen]:

| state | does |
|---|---|
| 2 | Wait while moving. 0x4 → 25. If a floor is remembered: wait 14 frames, compute the target (`sub_82764CC8`, ±27 clamp), re-SetAngle if the accelerometer is off, then go to state 4. If no floor: count down, then, while retries T+88 > 0 and current deg < 0, SetAngle(23 − 10·n) and wait 60 frames (a sweep). Then set T+60 = 1. |
| 4 | Wait while moving; 0x4 → 25; then go to 5 with 15 frames. |
| 5 | Count down, then go to 3. |
| 3 | Needs a depth frame. Gets gravity, runs `sub_8278E708`, possibly a corrective SetAngle(deg − delta, ±27), then goes to 6. |
| 6 | Wait while moving; stalled → 25 or StopMovement; final analysis; complete with T+432 (0/1). |
| 7 | Complete with 995 (cancelled by 0x2B009). |
| 8 | Set by `sub_82764BC0` at worker exit, which completes with 1223. |

* Completion `sub_82764AA8(code)` [seen]:
  * takes the pending block;
  * logs `sub_8278E708(0, ...)`;
  * stores code to block+0, SetEvents block+12, calls block+16 (`cb(code, 0, block)`);
  * then `sub_82764988(code)` → ReportStatus.
* The auto-tilt runs once per NuiInitialize. Failure is not fatal: no retry, and the game is unaffected apart from the report. [seen/inf]

### 6. XamNuiCameraTiltReportStatus, ElevationStopMovement

* **ReportStatus(u32 code, void* rec)**: 2 arguments, return value ignored. [seen]
  * code **0x2B006** with rec = T+96 = {+0 u32 status, +4 s32 target angle (T+100), +8 float (T+104), +12 float floorHeight = B+0x3C · c@0x82051624}.
  * code **0x2B00C** with rec = T+112 = {+0 u32 status, +4..+12 unused (0), +16 float = B+0x3C · c@0x82051624}.
  * Record sizes 16 and 20 bytes [inf].
* Which code is used depends on T+416 [seen]:
  * 1 (the 0x2B00A request) → 0x2B00C;
  * 2/3 → 0x2B006;
  * 0 (the library's own auto-tilt) → report only when status == 3, using 0x2B006.
* Status mapping in `sub_82764988` [seen]:

| completion code | reported status |
|---|---|
| 0 | 0 |
| 5 | 8 |
| 21 | 2 |
| 25 | 4 (plus StopMovement) |
| 4 | 4 (plus StopMovement) |
| 31 | 5 |
| 995 | 7 |
| other | passed through (for example 1223) |

  * Status **6** = busy (from the callback). Status **3** = movement while flags & 2 (in `sub_827650B8`: target reset to 0, T+60 = 0, T+68 = 1) [seen]. Meaning of flag 0x2 "moved externally" [?].
* **ElevationStopMovement()**: no meaningful arguments (r3 holds leftovers), return value ignored. Called from `sub_82764988` (codes 4/25) and from state 6 on a stall. [seen]
* Emulator: ReportStatus can be a no-op that returns 0. [inf]

### 7. XamNuiCameraElevationGetAngle

* **One call**, in `sub_82765290` (the 0x2B00A system request only): `GetAngle(s32* angle, ?* second)` = (&sp+80, &sp+84). [seen]
* The return value is ignored. Only *angle is read: it is compared with the target T+100 (re-SetAngle if the difference > c@0x8200EDE8). The second output is never read. [seen]
* Not used in the normal game flow. [seen]

### 8. XamNuiCameraRememberFloor

* Called only from `sub_827650B8` (per frame, from state machine entry, with r30 = worker+32 = the tracker's floor plane), and only when not moving (cached flags & 3 == 0) and the plane is valid. [seen]
  * "Valid" means w != 0 and dot(plane.xyz, gravity unit vector) >= c@0x82098F18 (`sub_82764FB8`). [seen]
* **Arguments: the float4 plane passed by value in GPRs**: r3 = 64-bit load of (nx, ny), r4 = 64-bit load of (nz, d) (high word first), r5 = BOOL T+48. [seen]
  * T+48 = 1 only for the first remember after the library's own SetAngle(−25). It is reset to 0 after each call. [seen]
* When it is called [seen]:
  * immediately if state == 2 (an operation just started);
  * otherwise after the same plane has been seen unchanged for 60 consecutive frames, and then on every further unchanged frame, because the counter is not reset after the call.
* The return value is ignored. [seen]
* Expected system behaviour: store the plane into XtSs +0x30..+0x3C and current gravity into +0x44..+0x4C. That is where the library later reads them back, and B+0x3C != 0 changes the auto-tilt path. [inf]

### 9. XamNuiGetDeviceStatus (game side, not the NUI library)

Signature: `GetDeviceStatus(16-byte buf)`. Only **+0xC status_bits** is ever read; +0, +4 and +8 are never read. [seen]

* **`sub_824D0D70`**: calls it with a stack buffer and **returns +0xC, ignoring the call's return value**. If the import does not write +0xC, the result is stack garbage, so always write it. [seen]
  * `sub_82232D48` (game manager init, once): byte [mgr+32547] = bits & 1, byte [mgr+32548] = (bits>>1) & 1. [seen]
  * `sub_82438930`: if (bits & 3) == 3 → `sub_8290B100` [seen] (purpose [?], runs under a critical section in 0x82B76690 code).
  * `sub_8290E228`: feeds bits as the param of a synthetic **0x00060019** (kXNotificationSystemNUIHardwareStatusChanged) to a listener `sub_8290E138` for the initial state. [seen]
* The live update comes from XNotify 0x60019 in `sub_82233040` [seen]:
  * param bit 0 → +32547;
  * bit 1 → +32548. On a 0→1 change with extra conditions, this calls `sub_8290B100`;
  * a bit 0 change to 0 → pause/menu handling: the virtual calls and state 1140/1144 changes. [seen]
* **+32547 (bit 0) gates Kinect gameplay processing** [seen]:
  * `sub_82222908`, `sub_82233470`, `sub_82233980`, `sub_82236FC8` and `sub_82495860` skip their work when it is 0.
  * So **bit 0 = "Kinect connected/ready"** [inf]. Bit 1 is a second "ready" condition [?].
* **`sub_8290F188`** (stats/telemetry object 0x82B76690 path, from `sub_8290F9F0`): at most once per 60 000 ms. If the call returns == 0, obj byte0 = (bits >> 3) & 1 (**bit 0x8**). Otherwise nothing. [seen] Meaning [?].
* No "connect Kinect" UI call was found next to these sites. The device-lost path goes through the 0x60019 notification handler. [seen/inf]

### 10. XamNuiGetDeviceSerialNumber

* `sub_8290F0C0(obj)`, once (latched by obj+32). It is skipped if `XamVoiceGetMicArrayStatus()` == 2.
* Otherwise it calls `GetDeviceSerialNumber(obj, 17)`, i.e. (char* buf, u32 size = 17). If the result is < 0, the 17 bytes are zero-filled. Not fatal; it is telemetry. [seen]

### 11. XamXStudioRequest

* No call in the tilt or device-status code. [seen]
* The codes found are 1, 769, 770, 4099 and 4100 (kinect-pscam §0), plus 5122 (`sub_8276FF90`, a skeleton-frame record), 5121 (`sub_82770428`) and 6 / 5378 (`sub_82777C88`). None relate to tilt. [seen]

### 12. Constants by address (values not visible in recomp)

| address | use | guess |
|---|---|---|
| 0x82051624 | floor d scale and report float | 0.001 |
| 0x8200092C | the "zero" compare value everywhere | 0.0 |
| 0x820015C0 | default normal y | 1.0 |
| 0x82041714 | rad→deg | 57.2958 |
| 0x82001C78 (double) | rounding | 0.5 |
| 0x8200EDE8 (double) | angle tolerance for re-SetAngle | unknown |
| 0x8200EDB0 / 0x8200EDC0 (double) | normalisation constants | 0 / 1 |
| 0x82098F18 | floor-normal · gravity threshold | unknown |
| 0x82AE8C28 | float[4] for 0x2B00A flags & 3 | unknown |
| 0x82062074 / 0x82098F14 | XConfig height clamp and default | unknown |

### 13. Minimal emulation (derived)

* TiltSetCallback: store the pointer, return 0.
* TiltGetStatus: accept the 0x50/'XtSs' block and return 0.
  * Fill gravity with gy > 0 and gz chosen so that round(atan2(gz, gy)·180/π) equals the current elevation. The sign is to be confirmed by watching whether the library re-sends SetAngle.
  * Set flags = 0 except for a short "moving" window after SetAngle.
  * Store RememberFloor's plane and the gravity at that moment into +0x30/+0x44.
* SetAngle: clamp to ±27, update the elevation, return 0. StopMovement / ReportStatus: return 0. GetAngle: write *angle, return 0.
* GetDeviceStatus: return 0 and **always write +0xC**. Bits 0 and 1 set means Kinect usable; also broadcast 0x60019 with the same bits on changes.

### Open questions

1. Sign convention of gz (and of gx, which the library negates) relative to positive SetAngle. Needs the constants or a run-time check.
2. Values of the tolerance 0x8200EDE8 and the floor-dot threshold 0x82098F18.
3. Exact meaning of XtSs flag 0x2 (vs 0x1), 0x10 and +0x40 bit 0; and whether the real system fills +0x30..+0x4C only after RememberFloor.
4. status_bits bit 1 and bit 3 meanings; what `sub_8290B100` does.
5. Whether game code (outside the NUI library) reads vNormalToGravity / vFloorClipPlane from the skeleton frame. Not traced.
6. G+0 really equals 27 (confirm that F640 does not OR in 0x800 or 0x04000000).

## 14. Kinect identity (XAM messages 0x2Axxx / 0x2Cxxx)

Third tracing pass (2026-11-21/22), same title and rules. It follows why EA SPORTS Active 2, UFC Personal Trainer and Kinectimals stop on their sign-in screens. These titles link the same NUI library, and the identity code is part of it. G = the NUI global block (`-11600(-32074)`, as above); `I` = the identity block at `*(0x82B5D2B0-844)` (another global).

### 1. `GetIdentityData` and error 80070015 [seen]

* `sub_8276B278` (the library's GetIdentityData) sets a request (G+8548 = 1, G+8544 = request) and waits on 2 objects (its event G+8524 and the caller's) for about 0.67 s. On a timeout (258) it returns **0x80070015** ("not ready"), and the title prints `[NuiIdentity] Unexpected Error - GetIdentityData hr = 80070015`.
* The request is answered by the per-frame worker `sub_8276C020`. It answers only when the frame has a colour record (worker+276) whose +120 is 1, and whose 12 bytes at +108 equal G+2200. These are the colour settings that were in force for that frame (section 4, colour completion), so the frame has to be one taken with the identity settings.
* So the driver has to accept code 35 and deliver colour frames that carry the settings number in the byte at +44. Commits 46 and 47 in motion-capture-log.md do this. With test build 8, the error is gone from the testers' logs.

### 2. Messages the library sends to XAM [seen]

All go to app 0xFE, with a 24-byte argument block. Each wrapper fills the block:

| msg | wrapper | block | how it is used |
|---|---|---|---|
| 0x2A004 | `sub_82A70678` | name, qword, buffer, size | write a stored blob (inferred) |
| 0x2A005 | `sub_82A70608` | name ("NuiIdentity"), qword* id, buffer, u32* size | read a stored blob: first with no buffer, to get id and size. A failure makes `sub_8277F628` return E_FAIL. Size 0, id 0 or a different id makes it return 1 (nothing stored) |
| 0x2A006 | `sub_82A705B8` | name | unknown (open or close) |
| 0x2A007 | `sub_82A70550` | xuid, qword, buffer, size | write biometric data for a user (inferred) |
| 0x2A008 | `sub_82A704E0` | xuid, qword* id, buffer, u32* size | read biometric data for a user; `id == 0x80000000` means none. Only asked when `XamUserNuiGetEnrollmentIndex` fails for that user (`sub_82773E28`) |
| 0x2A009 | `sub_82A70490` | qword | unknown |
| 0x2C009 | `sub_82A703C0` | callback pointer | registers the identity callback (below); XMsgInProcessCall |
| 0x2C00C | `sub_82A703F8` | mode, qword, buffer, size, out, u32* outsize | a XAM-side transform of a buffer: mode 2 with a name such as "NuiV1FaceHiRes" while loading face models; mode 0/1 to unpack stored blobs |

Every message here is async (XMsgStartIORequest), except 0x2C009 and 0x2C00C. All of them fail in Xenia, and the library treats that as "nothing stored". That is not what blocks the titles.

### 3. The identity callback (`sub_82774720` in Free Riders) [seen]

`callback(msg, data)`, where `msg - 0x2C002` (0..17) selects a case. This is how XAM tells the title about identity:

| msg | case | what the title does |
|---|---|---|
| 0x2C002 | 0 | nothing, returns 0 |
| 0x2C006 | 4 | `sub_82773888(2, data[0], data[1], data[2] \| 0x80000001)`: "skeleton data[0] is identity entry data[1]" |
| 0x2C007 | 5 | `sub_82773888(1, data[0], 0, 0x80000000)` |
| 0x2C008 | 6 | `sub_82773888(6, data[0], 0, 0x80000000)` |
| 0x2C00A | 8 | returns a global in data[0] |
| 0x2C00B | 9 | sets a global flag to 1 |
| 0x2C00E | 12 | `sub_82773AF0(data[0])` |
| 0x2C00F | 13 | `sub_827745F0()` |
| 0x2C011 | 15 | copies identity data out: needs data[0] >= 24 and writes 20 bytes to data[1..5] |
| 0x2C012 | 16 | `sub_827728F0(data[1], 1)` (needs data[0] >= 8) |
| 0x2C013 | 17 | `sub_82764728(data[1], ...)`, writes 2 words (needs data[0] >= 16) |
| others | | DbgPrint and 0x8000FFFF |

`sub_82773888(type, trackingId, entry, flags)`:
* Looks up `trackingId` among the 6 skeleton slots of `I`. Slot i is at I+192+i*336: tracking id at +324, state at +328 (2 = tracked), identity entry at +320. If there is no tracked slot with that id, it fails with 0x83010008.
* It then fills a 88-byte request record and signals the identity event.
* Type 2 goes through `sub_82773568`, which uses the 8 identity entries of 24 bytes at I+0 (state 3 = in use) and returns the entry number used.

### 4. What 0x2B002 carries [seen]

`sub_827795D0` → `sub_827794E8` builds a 2784-byte block each frame, and the worker sends it as 0x2B002:
* +0 qword 5
* +8 qword 0x2DFF
* +16 and +32 two vectors: the floor clip plane and the normal to gravity
* +48 2688 bytes: the 6 skeletons (6 x 448: state, tracking id, enrollment index, user index, position, 20 joints...), copied from the skeleton frame
* +2736 the timestamp, and +2748..+2760 four words

So XAM sees every tracked skeleton and its tracking id. That is how a console's XAM would know whom to identify and which `trackingId` to put in 0x2C006.

### 5. What 0x2C006 starts, and what comes back (0x2C003) [seen]

Test build 9 logs (Free-Riders-Recompiled#421) show every title answering the 0x2C006 announcement with a message **to** XAM, 0x2C003, a few frames later. Tracing it in Sonic Free Riders:

* The callback's job kinds are not what the names suggested. 0x2C006 (kind 2, flags always get bit 0) is **enroll**: learn the face of this skeleton into an identity entry. 0x2C007 (kind 1) is **identify**: compare this skeleton's face against the entries already learned. 0x2C008 is kind 6.
* The worker `sub_82777988` runs each job on every frame. It needs the skeleton still tracked, then finds the head in the colour picture (`sub_82774070`), and when the job's +104 is 2 it analyses the face (`sub_8277E5A0`) and keeps the sample (`sub_82772718`). Kind 1 then calls the matcher `sub_82776410`, kind 2 with bit 0 the enroller `sub_82776268`.
* The enroller finishes only once enough face samples are in (`sub_8277EA60`): it then stores them into the entry, marks the entry as holding data, and the result's entry (+24) is that entry. The matcher scores the face against each entry in state 1 that holds data.
* `sub_82775348` reports the result to XAM: kind 2 as **0x2C003**, kind 1 as 0x2C004, kind 6 as 0x2C005. The 24 bytes are {hr, tracking id, entry, flag, 64-bit id}. It also writes the entry into the skeleton slot (+320): an entry 0..7 on success, -1 if the skeleton was lost (hr 0x83010008), -5 otherwise. -4 means "being worked on".

What the logs show, decoded:

| Game | 0x2C003 | Meaning |
| --- | --- | --- |
| EA SPORTS Active 2, Game Party, Kinectimals | 0, entry -5 | the job ended without learning a face |
| UFC Personal Trainer | 0x80004004, entry -5 | the job was abandoned |
| Kinect Sports: Season Two | 0x83010008, entry -1 | the skeleton was no longer tracked when it ended |
| Just Dance 3 | 0x83010008, then 0x80004004 | both; it plays anyway, so it only needs the announcement |

So no title ever gets a learned face, and the skeleton's entry stays -5 or -1. The likely reason: the colour picture is the plain grey of row 47 in the log, so there is no face to find. On a console the face comes from the real colour camera. Getting real colour to the title (Kinect v2's colour stream, or the webcam, converted to the 360's colour format, which is still open: Bayer or YUV, section "Open questions" above) is the next step. Faking the result instead would mean writing into the title's own memory, which differs per game.

### 6. The colour picture the face code reads [seen]

* The head crop (`sub_82774070` → `sub_82779E30` → `sub_82782368`) reads a picture of **4 bytes a pixel** and turns it grey through three 256-entry lookup tables, one for each of bytes 0, 1 and 2 (byte 3 unused): grey = (T[b0] + T[256+b1] + T[512+b2]) >> 8. So by then the library has made full colour out of the camera's frame itself.
* The camera's frame is smaller: every title in the #421 logs (EA SPORTS Active 2, Game Party, Just Dance 3, Kinectimals, Kinect Sports: Season Two, UFC) asks for **307,200 bytes**, 640x480 at one byte a pixel (bpp 1; init flag 0x40 would make it 2). That is the Kinect v1's raw colour: a Bayer mosaic. libfreenect documents its order as GRBG. No CPU demosaic was found, so the library most likely does it on the GPU, in the pass that also gets the colour slot offsets (section 11 Q6).
* Where the head is in the picture comes from the skeleton: a joint goes to depth pixels (`sub_82772210`) and then to colour pixels (`sub_82771480` → `sub_82766FB8`, the registration). With the all-zero registration the camera returns for code 11, colour pixel = depth pixel (in sensor order). So the colour frame should show what the virtual v1's depth camera shows.
* What Xenia hands over (log rows 51 to 56): the device's colour carried with each depth pixel through the reprojection, written as a GRBG mosaic in sensor order, grey where there is no colour. A wrong tile order would only change the colours, not the brightness the face code works on.

### 7. Stored face data and the file cache [seen]

The identity code also asks XAM for what the console already learned of a player, through four exports Xenia did not have (every title in the #421 logs calls the first two):
* `XamReadBiometricData(id, &sequence, buffer, &size, overlapped)` (Free Riders `sub_82775110`, `sub_82776A78`): a Win32 error code, 0 = read. On an error it starts from nothing, then `XamUserNuiEnableBiometric(user, 1)` and `XamWriteBiometricData(id, &sequence, buffer, &size, overlapped)` store what it learned.
* `XamCacheOpenFile(record)` (`sub_82776980`), one record of 584 bytes per cached file: 0 = opened; otherwise it clears the record's handle (+16). Then `XamCacheReset(24)` and `XamCacheCloseFile(record, 0)` for each record still holding a handle.

A missing export returns without setting r3, so the title reads whatever r3 held (its own first argument) as the result. For the cache that looks like failure; for the biometric read it can look like success, and then the title uses a buffer nothing was read into. Fruit Ninja Kinect's identity thread crashed (host access violation) right after these calls (test build 9). Xenia now answers: read → 2 (not found), write → 0 (accepted, not kept), open → 2, reset and close → 0 (log row 57).

### Open questions

* Which callback message the console sends when it recognises a signed-in player (0x2C006 is the likely one), and what `entry` and `flags` should be. Entry looks like the enrollment index (0..7).
* Whether titles then read the user from the skeleton's enrollment/user index fields, or from `XamUserNuiGetEnrollmentIndex` (exported, still not implemented in Xenia), or both.
