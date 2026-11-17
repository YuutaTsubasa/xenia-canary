# Motion capture: commit log

One line per commit on `claude/kinect-one`, in order, in plain words: what
it changes and why. The design is in [motion-capture.md](motion-capture.md).

| # | Commit | What and why |
| --- | --- | --- |
| 1 | docs: motion-capture design | Writes down how the Xbox 360 Kinect works (the title tracks skeletons itself from raw camera frames), what Xenia and others already have, and the plan: one motion-capture API with Kinect v1, Kinect v2 and webcam backends under it, feeding an emulated Kinect v1 camera. No code yet; waiting for review. |
| 2 | docs: record the rewrite decision | The Kinect v1/v2 Windows code and the v2 depth reprojection in Free Riders were written by knuckleslee, so they can't simply move into a BSD project. The user chose to write them anew here from public sources (Kinect SDK reference, libfreenect) without opening those files. The design now marks which parts are rewritten and which are ported. |
