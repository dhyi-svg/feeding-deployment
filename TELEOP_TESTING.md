# Teleop recording — collecting real demos to test algorithms against

Drive the arm by hand with the Xbox controller, record what the robot saw and where it
actually was, then replay autonomous perception against that data offline — same frames,
same intrinsics, same camera pose, with your own end-effector poses as ground truth.

Everything here is **read-only**. Nothing in this file commands the arm.

Script: `scripts/session/record_teleop_demo.py`.
Bring-up for the box itself is `JETSON_SETUP.md`; this file replaces §6 of it for the
duration of a recording session.

---

## 1. Why a recorder is needed at all (and why it does not fight teleop)

Two halves, two different problems.

**The camera half is free.** It subscribes to `realsense2_camera` over ROS 2. A
subscriber cannot affect the arm, so there is nothing to be careful about.

**The arm half cannot go through `arm_server.py`.** Xbox teleop faults the Kortex session
that server holds (`JETSON_SETUP.md` §7: `ERROR_PROTOCOL_SERVER` /
`INVALID_USER_SESSION_ACCESS` on the next call), so anything reading through the arm RPC —
`record_ground_truth.py`, the launch's joint-state bridge, `check_tf_vs_fk.py` — dies the
moment you pick up the controller.

So the recorder opens **its own read-only Kortex session**
(`feeding_deployment/control/robot_controller/kortex_readonly.py`). It calls
`RefreshFeedback()` and nothing else: no `ClearFaults`, no `SetServoingMode`, no actuator
control-mode writes, no `ExecuteAction`, no `/tmp/kinova.lock`. Contrast `KinovaArm`,
which does all of those in its constructor — never construct it during teleop.

> **UNVERIFIED:** whether the arm grants that second (read-only) session while the
> controller holds control. It is read-only and Kinova's own web dashboard shows live
> values during teleop, but that is inference, not a hardware check on this box. Run
> `--check` while teleoping (§3) before trusting a session to it. If it is ever refused,
> `--no-camera` still records nothing at all from the arm — say so in `TESTING_LOG.md`
> rather than working around it silently.

---

## 2. What must and must not be running

| | Why |
|---|---|
| ❌ `arm_server.py` | teleop faults its session; it also holds `/tmp/kinova.lock` |
| ❌ `stub_base_server.py`, `bulldog_bypass.py` | only needed to unlock motion — there is no motion here |
| ✅ `microwave_bringup.launch.py` with `use_joint_state_bridge:=false` | camera + `robot_state_publisher` + the static hand-eye calibration |

The launch's own joint-state bridge is turned off because it reads the (dead) arm RPC.
The recorder publishes `/joint_states` itself, from its read-only Kortex session, so
`robot_state_publisher` still builds the TF tree and every saved frame can carry a real
`arm_base_link ← camera_color_optical_frame` matrix.

---

## 3. Running a session

Session environment first (from `JETSON_SETUP.md` §2):

```bash
cd ~/feeding-deployment
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$HOME/feeding-deployment/src:$PYTHONPATH"
PY=$HOME/feeding-deployment/.venv/bin/python
```

Camera and TF tree (`ros2 launch` needs **system python3**, not the venv — it needs `lark`):

```bash
ros2 launch launch/ros2/microwave_bringup.launch.py use_joint_state_bridge:=false
```

Then, **while driving the arm with the controller**, prove the recorder does not disturb
it. Do this every session, before the demo you care about:

```bash
$PY -u scripts/session/record_teleop_demo.py --check
```

It prints the arm state, one full sample, then samples for 3 s (the numbers must move
while you drive), and finally resolves `arm_base_link → camera_color_optical_frame` at a
real frame's stamp. If teleop hiccups at any point, stop and record that in
`TESTING_LOG.md` — that is the finding, and it retires the assumption this whole approach
rests on.

Record:

```bash
$PY -u scripts/session/record_teleop_demo.py --tag pos_A --note "door opened by hand, 2nd try"
```

While it runs, the terminal is a tagging console:

```
<Enter>          mark this instant           -> label "mark"
grasp<Enter>     mark it with a label        -> any word works
q<Enter>         stop                        (Ctrl-C also stops cleanly)
```

Gripper open/close transitions and arm-state changes tag themselves, so the grasp pose
lands in `ground_truth.json` even if you never touch the keyboard. Every tag also
force-saves the next frame, whatever the arm is doing.

`--tag` is per **appliance position**, not per session — same convention as
`record_ground_truth.py`. Move the microwave, use a new tag. A corpus spanning several
positions and orientations is the point; it is what shows whether a method generalises
instead of fitting one setup.

---

## 4. What gets recorded

```
~/captures/teleop_<timestamp>/
  meta.json               run metadata: args, git SHA, env vars, calibration, camera_info, counts
  state.jsonl             every arm sample (default 30 Hz)
  events.jsonl            tags, with the arm state at that instant
  ground_truth.json       first grasp pose, in record_ground_truth.py's schema
  frames/000000/
    frame_rgb.jpg               colour
    frame_depth.png             uint16 millimetres, ALIGNED to colour
    frame_detection_inputs.json camera_info + arm_base_link <- camera_color_optical_frame
    frame_state.json            arm state at that frame + image/pose sync age
```

**Yes, depth is recorded** — aligned to colour, lossless 16-bit PNG in millimetres, which
is exactly what `pixel2World` and the plane fit consume. The whole handle pipeline
(plane fit → protruding cluster → DBSCAN) works off depth, so a recording without it
would be useless for testing anything but the 2D detector.

### Per arm sample (`state.jsonl`)

| Field | Why it is worth the bytes |
|---|---|
| `joints`, `joint_vel`, `joint_effort` | the arm's configuration; FK reproduces any frame offline |
| `ee_pos`, `ee_quat` | the pose you drove to — the ground truth |
| `gripper` | 0 open … ~1 closed |
| `gripper_current` | **the only signal that says something is actually held.** `gripper_pos` saturates near 1.0 with or without a handle between the fingers (CLAUDE.md); motor current stays elevated while the fingers stall against an object |
| `ee_force`, `ee_torque` | external wrench at the tool: contact detection, and how hard the door resisted |
| `max_joint_vel`, `still` | whether the arm was being driven at that instant |
| `t` | wall clock, for joining against frames |

`state.jsonl` is continuous and cheap — it runs at 30 Hz for the whole session regardless
of what the frame gate below does, so the full driven trajectory is always there even
where no images were saved.

### Per frame

`frame_detection_inputs.json` is byte-for-byte the sidecar the live pipeline writes
(`_log_detection_inputs`), so **the replay tools read teleop recordings unchanged**.
`frame_state.json` adds three things the arm samples do not have:

- `camera_stamp` and `arm_sample_age_s` — how stale the pose label is relative to the
  image (<33 ms at 30 Hz). When a detector disagrees with a label, this says whether sync
  could explain it.
- `idle_for_s` — seconds since the arm was last driven. `0` means the frame was taken
  mid-motion, so expect blur and real sync error in its pose; a settled frame (`still:
  true`, `idle_for_s` near `--settle-sec`) is the one to trust as ground truth.

`meta.json` records the recorder's git SHA. It does **not** record `HANDLE_DEPTH_CORR` and
friends: no detection runs during a recording, so those values cannot influence the data.
They belong to whatever replay you run later, and that is where they should be recorded.

---

## 5. Size, and the knobs that control it

Measured on this rig's 640x480 frames:

Measured on this rig's real 640x480 captures:

| | per frame | at 2 Hz, while driving |
|---|---|---|
| **default** (jpg q95 88 kB + depth png6 51 kB + ~3 kB json) | ~142 kB | **~17 MB/min** |
| `--jpg-quality 90` | ~113 kB | ~14 MB/min |
| `--rgb-format png` (lossless colour) | ~373 kB | ~45 MB/min |
| `--no-camera` (arm state only) | — | ~0.7 MB/min |

Those are rates *while the arm is being driven* — the gate below means a session spent
mostly parked costs far less. A five-minute session with two minutes of actual motion
lands around **35 MB**, so a day of a dozen demos is well under a gigabyte.

`state.jsonl` is ~400 bytes per sample, so 30 Hz costs ~0.7 MB/min and runs the whole
session regardless of the gate.

**Is JPEG a problem?** Only the colour image is lossy, and only slightly at q95 — depth,
the metric channel every geometry stage actually uses, is always lossless 16-bit PNG. The
one real caveat is that a detector run on recorded colour sees a marginally different
image than it would live; if you are ever chasing a sub-percent confidence difference,
re-record that case with `--rgb-format png`.

### Why there is no archive format (HDF5, zarr, tar.zst, bag)

Measured on this repo's real captures, not assumed. The recording is ~95% image bytes,
and those bytes are *already* compressed — so wrapping them in a container that applies
zlib/zstd again buys nothing:

| | per frame |
|---|---|
| colour, JPEG q95 (default) | 88 kB |
| colour, PNG level 6 — lossless | 319 kB |
| colour, raw + zlib — what an HDF5 gzip filter gives | 444 kB |
| **JPEG then zstd -10** | **88 kB — 0% gain** |
| depth, PNG level 6 (default) | 51 kB |
| depth, raw uint16 + zlib / + zstd | 56 / 54 kB — *worse* |
| depth, raw uint16 + lzma | 38 kB, but 360 ms/frame vs 38 ms |

The tempting argument is cross-frame redundancy: consecutive frames look alike, so a
container that compresses them together should win big. It does — but only on data that
does not exist. Ten pixel-identical depth frames go from 503 kB (separate PNGs) to 44 kB
as one `zstd -19 --long=27` stream, an 11× win. Add the ±2 mm of noise a real RealSense
puts on every pixel and the same test gives 1749 → 1708 kB: **2%**. Sensor noise destroys
byte-level matching, which is the only thing a lossless general-purpose compressor can
exploit.

And the case where frames genuinely are near-identical — arm parked, camera static — is
the one the motion gate already handles by not writing them at all. That is the same win,
taken earlier and more cheaply.

The one real lever left is a **lossy video codec** for the colour stream (H.264/HEVC would
be an order of magnitude smaller by modelling motion between frames). It is deliberately
not the default: it breaks the per-frame capture format every replay tool reads, and puts
inter-frame compression artifacts into the images an algorithm is being scored on. If disk
ever becomes the binding constraint, encode a *copy* for archive and keep the frames.

### The motion gate

Frames are saved **while the arm is being driven, and for `--settle-sec` (default 1.5 s)
after it stops** — so you get the motion *and* the sharp, settled frame of wherever you
just drove to. Once it has been parked longer than that, saving stops: a hundred copies of
one stationary view is bytes, not data. Move the arm and it resumes by itself.

Motion is judged at 30 Hz from the arm's own joint velocities, not from the frame loop, so
a quick nudge between two frames still counts. A tag always force-saves the next frame
regardless, and `--all-frames` disables the gate entirely.

Every skipped frame is counted (`frames_skipped_idle` in `meta.json`), so a small
recording is never ambiguous about why.

Other knobs: `--frame-hz` (default 2), `--state-hz` (default 30), `--still-vel` (the speed
below which the arm counts as parked, default 0.02 rad/s ≈ 1.1 °/s), `--min-free-gb`
(stops rather than filling the disk), `--max-minutes`.

---

## 6. Using the recordings

Replay the real detector — GroundingDINO, real plane fit, real DBSCAN, real corrections —
with no arm and no camera. Only the tf2 lookup is replaced, by the matrix recorded at
capture time:

```bash
$PY -u scripts/session/replay_detection.py ~/captures/teleop_*/frames/*
```

Each frame directory is one self-contained capture, so globbing `frames/*` replays them
all in one process (the model loads once).

To score a run against the human's own pose:

```python
import json, numpy as np
gt = json.loads(open("ground_truth.json").read())["ee_pos"]          # where you actually were
# ... run detection on that capture, then:
print(np.linalg.norm(np.asarray(detected) - np.asarray(gt)) * 100, "cm")
```

The convention `record_ground_truth.py` established, and the one this follows: touch the
handle with the gripper **centred left-right on the bar, at the depth you would close**.
`HANDLE_DEPTH_CORR` was fitted so the corrected handle pose equals the EE pose at that
touch, which is why `GRIP_EXT` is 0. Height along a long vertical bar is ambiguous —
touch at a repeatable height (mid-bar) and score depth/lateral error separately from
along-bar error.

### The other thing these recordings are for

A demo recorded **while gripping the door** is a direct measurement of the door's real
kinematics. The open question on this rig is exactly that: `_generate_door_arc_waypoints`
assumes a purely vertical hinge axis, and a user-demonstrated waypoint came out +5.8 cm
in z from the formula's prediction because this microwave's door rises as it opens
(CLAUDE.md). Grip the handle, open the door by hand, tag a few points along the way —
`events.jsonl` then holds real waypoints on the true arc, and `state.jsonl` holds the
whole continuous path at 30 Hz. Fitting the actual axis from that beats guessing a
parametric model, and it is the low-risk option the next-steps note already prefers.

---

## 7. Gotchas

**Frames dropped for no TF.** A frame whose `arm_base_link → camera_color_optical_frame`
cannot be resolved *at its own stamp* is dropped, not saved against the newest available
transform — during teleop the arm is moving, so "newest" is a different pose, and a
mislabelled frame is worse than a missing one. A large `frames_dropped_no_tf` in
`meta.json` means the TF tree was not up (launch not running, or `/joint_states` not being
published), not that the demo was bad. `--check` catches this before you record.

**The microwave moves.** Every door swing nudges it a few cm. That is why ground truth is
tagged per appliance position and re-touched, not stored once and reused.

**The shared ledger is not touched.** `ground_truth.json` is written inside the run
directory only; `~/captures/ground_truth.jsonl` stays whatever `record_ground_truth.py`
put there.

**`--check` needs you to be driving.** Its whole purpose is to observe the arm while
something else controls it. Running it on an idle arm proves much less.
