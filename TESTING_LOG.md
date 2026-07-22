# Real-arm testing log — microwave door opening

Hardware bring-up + autonomous microwave-door-opening on the single-machine rig
(Jetson Orin Nano + Kinova Gen3, no NUC/base/ROS). Companion to `LOCAL_DEPLOYMENT.md`
(env / what-works reference) and the `## CURRENT STATE / NEXT STEP` block in `CLAUDE.md`.
Newest sessions on top.

---

## Setup — talking to the arm (every session)

The RPC arm interface hardcodes the lab NUC hostname; the local edit to
`arm_interface.py` lets an env var override it, and the Kortex SDK lives in the user
site (not `.venv`). So every arm command needs:

```bash
E="PYTHONPATH=$HOME/.local/lib/python3.10/site-packages ARM_RPC_HOST=127.0.0.1"
PY=$HOME/feeding-deployment/.venv/bin/python
```

- `ARM_RPC_HOST=127.0.0.1` → `NUC_HOSTNAME = os.environ.get("ARM_RPC_HOST", <lab NUC>)`
  in `arm_interface.py` (**uncommitted** local edit). Points the RPC client at the
  arm server running on this box instead of the lab NUC.
- Detection also needs `CUDA_VISIBLE_DEVICES=""` (GroundingDINO on the Tegra iGPU hits
  an NVML assert — run it on CPU, ~34 s/frame).

Bring-up order each session (see `LOCAL_DEPLOYMENT.md` for the full table):
1. `arm_server.py` — connects, clears faults, holds position (no motion).
2. `scripts/stub_base_server.py` — no-op base so bulldog's handshake passes.
3. `scripts/bulldog_bypass.py` — flips `bulldog_ready`, heartbeats `is_alive()`.
   **No software e-stop** — physical only. If it dies the arm e-stops in ~1 s.

**After Xbox teleop or any e-stop:** the arm goes to firmware manual mode / the Kortex
session faults. Restart `arm_server.py` (reconnect + fault-clear + hold), then re-run
the bypass (fresh server re-locks motion). Verify with `get_state()` → expect
`SERVOING` and control reclaimed.

---

## 2026-07-22 (later) — Pachirisu: recurring USB controller failures block depth work; handle detection re-confirmed

Follow-on session, same day as the calibration entry below. Goal was to run the repo's
real `AppliancePerception.detect_handle_and_placement` (the plane-fit + DBSCAN
3D handle-localization method, not just `detect_items`) against the microwave, using a
monkeypatched `get_frame_to_frame_transform` that substitutes the prior session's
`cv2.calibrateHandEye()` result for the live tf2 lookup this repo normally expects
(nothing on this box publishes a TF tree — see the plan written for this at
`~/.claude/plans/quiet-herding-puzzle.md`, not yet executed). **Did not get there** —
repeated USB/camera hardware failures ate the session. Recovered a working visual
result (plain detection, no depth) before time ran out.

### `align_depth:=true` is unreliable on this box — but so, eventually, is plain streaming

Attempting to bring up the camera with `align_depth:=true` (needed for the
depth-based handle-localization work — `pixel2World` requires depth aligned to the
color pixel grid) hit the same `xhci_hcd 0000:00:14.0: xHCI host controller not
responding, assume dead` full-controller crash **twice**, each requiring the same
`sudo` PCI unbind/rebind fix from the previous session
(`echo -n "0000:00:14.0" | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind` then
`.../bind`) plus a physical replug to fully recover. A third `align_depth` attempt
showed the same runaway `messenger-libusb.cpp` `control_transfer` warning spike that
preceded both crashes, but was caught early with a SIGINT before it took the
controller down — first evidence the pattern is at least somewhat predictable
in advance (a burst of `control_transfer` errors a few seconds after "Sync Mode: On"
appears in the log), not purely instantaneous.

**This session's new, worse finding: it's not `align_depth`-specific.** A subsequent
plain (non-`align_depth`) launch — the exact mode that streamed rock-solid at 30Hz for
the entire previous session with zero incidents — also crashed the same way
(`usb device disconnected`, full controller death). This downgrades the earlier
"only `align_depth` triggers it" theory to "the camera's USB connection is degrading
with repeated cycling, and `align_depth` (heavier USB traffic / `Sync Mode: On`) just
makes it more likely, not exclusively responsible." Each recovery cycle also seemed to
take a bit more coaxing than the last (one attempt needed unbind/rebind **and** a
physical replug before the device would even enumerate at the USB/SDK level again,
where earlier in the same session unbind/rebind alone had sufficed).

**Not resolved.** Suspect the physical USB cable/port/hub chain itself may be marginal
(the earlier session's dmesg output showed the camera connected via `2-1.1` a
sub-hub port, not directly on a root port) — worth trying a different cable and/or a
direct motherboard port (bypassing any hub) before the next attempt at depth work,
rather than continuing to treat each crash as independent and re-running the same
recovery.

### Recovered: a clean, stable visual result before time ran out

After the last recovery cycle (unbind/rebind + physical replug), plain (non-aligned)
streaming came back solid (30Hz, `rostopic hz` clean) and stayed stable for the rest
of the session. Re-ran the same `detect_items` smoke test as the prior session, with
the actual microwave swapped back into frame: **`microwave: 0.57`** confidence, clean
single bounding box — consistent with both this rig's and the Jetson's previously
documented confidence range. Annotated + raw frames from both this session and the
prior one saved to `~/deployment_ws/pachirisu_detection_trials/` (host-persisted, not
in the repo).

### Not done — carried forward to next session

- The actual goal (`detect_handle_and_placement` with the no-tf2 monkeypatch) was
  never executed — a full plan for it exists at `~/.claude/plans/quiet-herding-puzzle.md`
  (monkeypatch design, two-tier `handle_type` retry — try `"microwave handle"` single-class
  first since every previous test used a 3-class combined prompt, fall back to
  `"microwave"`'s whole-appliance box otherwise, mirroring the Jetson script's proven
  approach) and can likely be executed largely as-written once `align_depth` (or a
  software-alignment substitute) is usable again.
- Camera hardware fragility itself needs attention before more depth work — see above.

### End-of-session state

Stopped cleanly, SIGINT-only throughout (camera → `arm_server.py` → `roscore`),
verified no leftover processes and no stale `/tmp/kinova.lock`. Camera was in a
stable, working state at the moment of shutdown (not mid-failure).

---

## 2026-07-22 — Pachirisu: eye-in-hand calibration done + validated, USB controller failure/recovery

Follow-on session. Goal was the camera→arm-base calibration flagged as the blocker at the end of the
prior session. Got it done and validated to 1.1mm on a real robot move, but a full detect→grasp→open
attempt is **not** ready this session — see gap list at the end. Also: the box's USB host controller
genuinely died mid-session and needed a kernel-level recovery, worth knowing about if cameras/USB
devices vanish again in a future session.

### Calibration approach — plain OpenCV, not easy_handeye

`easy_handeye` (ROS1) isn't a RoboStack binary package and would need an untested-on-this-box catkin
source build, plus it normally drives calibration poses via MoveIt, which this repo doesn't use.
Skipped it entirely in favor of `cv2.calibrateHandEye()` directly: move the arm through a series of
poses (low speed, poll-for-settle, independent re-verify — same discipline as every real-arm move this
session), record `get_state()`'s EE pose (gripper2base) at each one, detect the calibration target and
`solvePnP` its pose (target2cam) at the same pose, feed the paired lists into
`cv2.calibrateHandEye(method=cv2.CALIB_HAND_EYE_TSAI)`. No new packages, no MoveIt integration.

**Target:** a physical board taped in front of the arm — 12 ArUco markers (`cv2.aruco.DICT_5X5_50`,
IDs 0–11) in a 4×3 grid, identified from a captured frame by brute-force trying `cv2.aruco`
dictionaries until one matched all 12. Marker size taken as the user's estimate (2 in / 0.0508 m,
unverified by ruler) — this sets the *absolute* scale of the whole calibration; worth double-checking
if later grasp attempts show a consistent scale-like error.

### First attempt failed validation — single-marker depth noise

Using only one marker (ID 0) per pose gave a calibration that looked plausible but failed a real
check: since the board is physically stationary, computing "target position in base frame" from every
collected pose should agree closely across poses. It didn't — Z (depth) axis had 15cm std / 48cm max
spread across 11 poses, X/Y were fine (~2-3cm). Root cause: a single ~5cm marker viewed from ~50cm has
poor depth/tilt constraint from `solvePnP` — small pixel corner noise blows up along the camera's
viewing axis. This is exactly why the physical target has 12 markers, not 1 — just wasn't using them.

### Fix — full-board multi-marker pose, empirically derived (not assumed) spacing

Rather than assume a uniform grid pitch, derived the board's true local geometry from vision itself:
solve marker 0's pose via `solvePnP`, fit its plane (point=t0, normal=R0's local +Z in camera frame),
then for every other visible marker's 4 corners, ray-cast the pixel through the camera's
inverse-intrinsics and intersect with that plane, express the result in marker-0's local frame. This
needs no spacing measurement beyond the one marker-size number, only planarity (true — it's a flat
printed board). Per-pose target pose is then one combined `solvePnP` over all currently-visible
markers' corners (8-12 of 12, depending on pose) instead of 4 points from one marker.

**Also found: J6 is the dominant "aim" joint here** (not J5, unlike the wrist-tilt finding earlier
this session — the effective joint is pose-dependent). First re-collection attempt (±10-20° J5/J6/J7
combos) tanked marker counts on any pose touching J6 (a lone ±15° J6 move alone: 12 markers → 0-4),
while J5/J7 stayed at 10-12 markers even at ±20° (near-pure roll at this configuration). Refit range:
J5/J7 pushed to ±15-20° for rotational diversity, J6 capped at ±5°. Result across 17 poses (all kept
8-12 markers): **X std 6.2mm/spread 2.4cm, Y std 16.7mm/spread 6.3cm, Z std 5.0mm/spread 1.9cm** —
self-consistent, usable. Saved to (persisted on host, bind-mounted, survives container recreation):
```
~/deployment_ws/pachirisu_wrist_camera_calib/wrist_camera_calib_fullboard.json   # R_cam2gripper, t_cam2gripper (~20cm magnitude, physically un-sanity-checked)
~/deployment_ws/pachirisu_wrist_camera_calib/handeye_raw_samples.json           # raw per-pose samples + derived board geometry, for re-solving without new arm motion
```

Also learned the hard way: **`ARMSTATE_SERVOING_MANUALLY_CONTROLLED` reads as a persistent-looking
state for ~5s after almost every `REACH_JOINT_ANGLES` call on this rig**, then clears to `READY` on
its own — not a fault, just needs patience (poll for ~6s before treating it as real). An earlier, less
patient version of this check caused a false-abort mid-calibration.

### Live validation on the real robot: 1.1mm

Computed a hover target 15cm off the board's center (detected board → transform through the
calibration → base frame), IK'd to it (seeded PyBullet FK from real joints, unconstrained), commanded
a 3-substep approach. First substep landed **1.1mm** from the predicted point — strong direct evidence
the calibration is correct, not just internally consistent.

**Second substep hit a real J4-proximity issue, unresolved.** J4 was already sitting close to its
flagged soft-limit region (-145° to -150°, limit is -152.4°) from an earlier e-stop event nudging the
arm. The unconstrained IK's solution for substep 2 pushed J4 to -149.5°, and the move aborted (arm
read stuck `MANUALLY_CONTROLLED`, didn't clear — possibly the firmware protecting the limit, possibly
something else, couldn't fully distinguish). Tried fixing via null-space IK
(`calculateInverseKinematics` with `lowerLimits`/`upperLimits`/`jointRanges`/`restPoses` tightened on
J2/J4/J6) — **first attempt silently no-op'd**: the URDF has 13 non-fixed joints (7 arm + 6 gripper
fingers) and only 7-length arrays were passed, so pybullet likely ignored the null-space branch on the
size mismatch (same J4=-149.5° result). Fixed the array lengths (13, using the gripper joints' own
real limits + current position as their rest pose) — this genuinely moved J4 away from the limit
(-136°) but **accuracy regressed to 5.9cm tracking error**, i.e. the DLS null-space solver traded off
primary-objective accuracy for the added constraints without enough iterations/tuning to converge
properly. Not resolved — stopped here rather than continuing to fight the IK solver.

### USB host controller died mid-session — not camera-specific, needed a kernel-level fix

While cycling the RealSense through several forceful `pkill -9` restarts (chasing an `align_depth`
relaunch), the *entire xHCI USB host controller* (PCI `0000:00:14.0`) crashed
(`kernel: xHCI host controller not responding, assume dead`) and the kernel force-disconnected every
device on it at once (not just the camera — 8 devices across two bus trees). Confirmed via
`sudo dmesg` (needs sudo — `dmesg_restrict` blocks it otherwise, even for the invoking user, not just
this box's sandboxed shell). Neither a container restart nor several camera replugs (including trying
a different port) fixed it, because the fault was upstream of all of that. Fix:
`echo -n "0000:00:14.0" | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind` then the same with `bind` —
after several attempts, this worked and all USB devices re-enumerated with fresh device numbers.
**Takeaway: if USB devices vanish entirely (not just the camera) and replugging into a different port
doesn't help, suspect the host controller itself before assuming it's a per-device problem** —
`sudo dmesg | tail -30` right after a failed replug is the fastest way to tell (a genuine device-level
failure looks different in the log from a `xHCI host controller ... assume dead` line).

### Also this session: accidental e-stop, clean recovery

A physical e-stop was pressed (accidental). Recovery matched the documented procedure exactly —
`arm_server.py`'s TCP session hit a `BrokenPipeError` (e-stop severs the Kortex connection hard), which
also broke `ArmInterface.close()`'s clean-shutdown path (same broken pipe), leaving a stale
`/tmp/kinova.lock`. No manual cleanup needed: `KinovaArm.__init__` already self-heals stale locks
(checks if the PID in the lock file is still alive via `os.kill(pid, 0)`, removes it if not) — a plain
restart of `arm_server.py` handled it.

### Not done / real gaps before a full detect→grasp→open attempt

- **Handle localization.** `AppliancePerception.detect_items` reliably detects the `microwave` class
  (0.39-0.61 confidence, consistent with the Jetson's documented range) but **never** fired on
  `microwave handle`/`door handle` in any test this session, even with the handle clearly, closely
  visible — a prompt/threshold sensitivity issue at this camera's viewing angles, not a scene problem.
  The Jetson's real pipeline doesn't depend on that prompt working either — it detects `microwave`,
  then finds the handle via depth plane-fitting + DBSCAN "protruding cluster" (see the 2026-07-14 entry
  below). That depth-based step is unbuilt on Pachirisu; `align_depth:=true` was smoke-tested working
  once this session (before the USB controller died) but not exercised against a real
  handle-localization attempt.
- **No empirical corrections tuned for this rig** — the Jetson's `DEPTH_CORR`/`LAT_CORR`/`GRIP_EXT` are
  specific to *that* camera mount and don't transfer; this rig needs its own, which normally takes
  several real grasp attempts compared against ground truth.
- **Only validated at small range (7cm, 1 substep).** The J4-proximity/null-space-IK issue above is
  unresolved for larger moves.
- **The microwave itself needs to physically replace the calibration board** in front of the arm
  before any of this can be attempted for real.

### End-of-session state

Everything stopped cleanly: `bulldog_bypass.py`/`arm_server.py` (SIGINT, lock released),
`roscore`/`realsense2_camera` (SIGINT/SIGKILL as needed), verified no leftover processes and no stale
lock file. Container `feed-noetic` left running but idle (note: it was also found unexpectedly *not*
running at the start of this session despite last session's doc note saying so — the prior session's
own teardown apparently did stop it via `docker stop`, contradicting what got written down; worth a
`docker ps` gut-check at the start of any session rather than trusting this note blindly). Calibration
files persisted to the host at `~/deployment_ws/pachirisu_wrist_camera_calib/`, independent of
container lifecycle.

---

## 2026-07-21 (later) — Pachirisu: first commanded motion + GPU vision pipeline

Follow-on session, same day as the read-only bring-up below. Two goals: (1) the
**first-ever commanded motion** on this box, done as a small/cautious rung-by-rung
ladder, and (2) get the repo's **real vision model** (GroundingDINO, not a stub)
running against the live RealSense feed. Both succeeded. Container churned through
several recreates along the way — details below so the next session doesn't have to
rediscover any of this.

### First motion: J5 wrist tilt, ±5° then 20°

Goal was specifically a wrist pitch-up move, not a base-joint sweep. Reasoned which
joint from the *current* pose rather than assuming: seeded PyBullet FK (same
`resetJointState`-from-real-joints pattern as the Jetson's seeded-IK fix) from
`assets/robot/robot.urdf`, perturbed J5/J6/J7 by ±5° each, and compared each one's
effect on the `tool_frame` local-Z axis's world-Z component (i.e. does the gripper's
pointing direction tilt away from straight-down). Findings:
- **J7 structurally can't tilt the gripper at all, at any configuration** — the fixed
  joint chain `bracelet_link → end_effector_link → tool_frame` only flips/translates
  along `bracelet_link`'s own Z axis, and J7 rotates *about* that same axis (pure
  roll). Confirmed numerically: Δ(approach-Z) = 0.0000 for both directions.
- **J5 (continuous, no joint limit) beat J6 (limited, but with huge margin here)** on
  tilt-per-degree, so J5 was picked. Sanity-checked the FK model first: PyBullet's
  seeded FK vs the real `get_state()` EE quaternion were nearly identical (quat
  distance 0.0084), confirming the URDF is trustworthy for this kind of reasoning.
- Also flagged as a live safety check: J4's *current* reading (-151.8°) is already
  past the URDF's own stated soft limit for that joint (-147.3°) — ruled out J2/J4/J6
  as candidates for this reason alone, independent of the tilt-effectiveness result.

**First attempt: `ACTION_ABORT` / `abort_details: METHOD_FAILED`, on every
`REACH_JOINT_ANGLES` call — even a zero-delta one** (commanding the arm to its own
current position also aborted the same way). `GetArmState().active_state` read
`ARMSTATE_SERVOING_READY` throughout — no fault, not manually-controlled, telemetry
fine. Added a temporary diagnostic (`ArmInterface.get_arm_state()` in
`arm_interface.py`, plus an uncommented notification-event print in `kinova.py`'s
`check_for_end_or_abort` — both left in place, harmless/read-only) to get the actual
abort reason. Never fully root-caused: it started working right after the user
interacted with the arm's web dashboard (`http://192.168.1.10`) — the pose had also
visibly shifted between attempts (consistent with a manual jog), suggesting the
dashboard was holding some kind of implicit control lock or advisory that
`GetArmState()` doesn't surface. **Worth checking the dashboard for an open
manual-control panel before assuming a `METHOD_FAILED` abort is a code bug.**

Once unblocked: J5 -5° (target -20.14°, actual -20.14°, wrong on the first try only),
then -5° again, +5° reverse, then a 20° move — every one landed within ~0.02° of
target, with all 6 other joints exactly 0.000° delta each time. Full rung ladder
(propose → explicit go → execute → poll-for-settle → re-verify) held throughout.

### Vision: real GroundingDINO on the RTX 5070, not a stub

Repo's own entry point is `AppliancePerception.detect_items` (via `GroundedSAM` +
`RealSenseInterface`) in
`src/feeding_deployment/perception/appliance_perception/appliance_perception.py` —
note the directory-vs-file collision: the *module* path is
`feeding_deployment.perception.appliance_perception.appliance_perception`, not the
flat `feeding_deployment.perception.appliance_perception` it looks like at a glance
(there's a package dir and a same-named `.py` file inside it). Also: don't
redundantly `sys.path.insert()` the `msgs_ws` path on top of it already being in
`PYTHONPATH` — the duplicate entry made `feeding_deployment.perception` resolve as a
broken multi-origin namespace package and produced a confusing
`cannot import name ... (unknown location)` error that looks unrelated to the real
cause.

**Container recreated three times** to get here (each time via `docker commit` first
so nothing installed got lost — `ros_env`/`kortex_api`/`msgs_ws` live in the
container's writable layer, not a bind mount):
1. `--privileged` — for `/dev/video*` + USB access (originally missing entirely:
   `Privileged: false`, no `--device`, camera invisible inside the container even
   though present on the host).
2. `--gpus all` added — for the RTX 5070 (nvidia container runtime was available on
   the host, just not attached to this container). First recreate attempt **forgot
   this flag** (copy-paste from the previous recreate) — caught immediately via
   `nvidia-smi` coming back empty inside the container; fixed by recreating again
   from the same snapshot with the flag added, no state lost.
3. `-v /dev:/dev` added — `--privileged` alone does **not** make the container's
   `/dev` track the host live; it's a snapshot taken at container start. A mid-session
   power event (arm's plug pulled, camera also affected) caused the RealSense to
   re-enumerate on the USB bus with a new device number; the container kept serving
   the *old* stale `/dev/bus/usb/002/NNN` node, so the camera process failed with
   `RS2_USB_STATUS_NO_DEVICE` / `acquire_power failed` even after a physical
   replug + fresh process restart. Only fixed by bind-mounting `/dev` live. **This
   is now permanent** — future USB replugs should Just Work without another rebuild.

Final container: `feed-noetic`, from image `feed-noetic-snapshot3`, run with
`--network host --privileged --gpus all -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix
-v ~/deployment_ws:/root/deployment_ws`. Old intermediate containers/images cleaned
up at end of session; kept `feed-noetic-snapshot3` (current) and the original
`osrf/ros:noetic-desktop-full` base as a from-scratch fallback.

**Package installs, into `ros_env` (RoboStack, Python 3.11):**
| package | note |
|---|---|
| `torch` / `torchvision` | plain `pip install`, no special index needed — latest stable (2.13.0+cu130) already supports Blackwell (RTX 5070, sm_120) out of the box. Verified with a real `@` matmul on `cuda`, not just `is_available()` (per `grounded_sam.py`'s own documented gotcha about that lying). |
| `groundingdino-py` | PyPI package (not a cloned repo). Installs as a **pure-Python wheel** — no compiled CUDA extension to fight Blackwell over; the deformable-attention op just runs through standard PyTorch ops on GPU. |
| `transformers` | **must pin `<5`** — latest 5.x removed `get_head_mask`, which GroundingDINO's `BertModelWarper` still calls; install resolved to `4.57.6`. |
| `supervision` | `groundingdino-py` pins `==0.6.0`, but this repo's `appliance_perception.py` unpacks `Detections` iteration as a 6-tuple (`for _, _, confidence, class_id, _, _ in detections`), which only exists from roughly 0.14+ onward (the `data` field). Overrode to `0.21.0` — pip warns about the conflict with `groundingdino-py`'s pin, harmless in practice. |
| `segment_anything` | `pip install git+https://github.com/facebookresearch/segment-anything.git` — only needed for the module-level import in `grounded_sam.py`; SAM itself stays lazy-loaded (appliance/handle path never touches it). |
| `open3d`, `scikit-learn` | plain installs, no issues. |
| `ros-noetic-realsense2-camera` | RoboStack (`robostack-staging`), pulls in `ros-noetic-librealsense2` 2.50.0. |

**Checkpoint:** `groundingdino_swinb_cogcoor.pth` (938 MB, Swin-B) downloaded from
`https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha2/` to
`~/Grounded-Segment-Anything/` inside the container — not in the repo, not cached
anywhere, has to be fetched fresh per box.

**Result:** `GroundedSAM()` loads with `DEVICE: cuda`. Live frame from
`/camera/color/image_raw` (30 Hz, confirmed via `rostopic hz`) fed into
`detect_items()` with the actual microwave placed in view:
`microwave: 0.61`, `microwave handle: 0.56`, `door handle: 0.40` — all landing on the
same correct bounding box (visually confirmed via the annotated overlay), consistent
with the Jetson rig's documented 0.59–0.81 range for the same prompt.

### Not done this session
- **No camera→arm-base calibration on this box.** The Jetson's detect→grasp pipeline
  depends on an `easy_handeye2` eye-in-hand calibration file plus empirically-tuned
  corrections (`DEPTH_CORR=0.16`, `LAT_CORR=0.07`, `GRIP_EXT=0.065`) tuned to *that*
  rig's specific camera mount — none of that exists or has been verified here.
  Explicitly decided **not** to attempt any detect→move-the-arm-toward-it step today
  without it; that's real, separate calibration work, not something to stack onto a
  first-motion + first-vision session.
- Depth (`/camera/depth/image_rect_raw`) was streamed and confirmed at 30 Hz, but
  `RealSenseInterface` (the repo's own consumer class) expects
  `/camera/aligned_depth_to_color/image_raw` specifically — need
  `align_depth:=true` on the `rs_camera.launch` invocation (or `rs_aligned_depth.launch`)
  before `RealSenseInterface`/`detect_handle_and_placement`'s full 3D pipeline will
  work; today's detection test used `detect_items` directly (RGB-only) to sidestep
  this.
- `stub_base_server.py` / real `bulldog` still not exercised on this box.

### End-of-session state
Everything stopped cleanly: `bulldog_bypass.py` + `arm_server.py` (SIGINT, lock file
released, arm re-locked), `roscore` + `realsense2_camera` (SIGINT/SIGKILL as needed).
Container `feed-noetic` left running but idle — next session can skip essentially all
of today's setup (`ros_env`, GPU, live `/dev`, checkpoint, all pins) and go straight
to bring-up.

---

## 2026-07-21 — Pachirisu read-only arm bring-up (RoboStack + kortex_api)

**New host, same arm.** First-ever connection from `Pachirisu` (RTX/24.04 desktop, separate
from the Jetson rig) to the Kinova Gen3 at `192.168.1.10`, using the RoboStack `ros_env`
set up earlier this session (see the native-install-blocker entry below). Goal: read-only
telemetry only — no motion, no gripper, no mode changes beyond what `arm_server.py`'s own
init already does. Treated as an unverified fresh bring-up throughout.

### Network
`enp4s0` already on `192.168.1.11/24` (same subnet, no static-IP change needed); arm
pings in <0.3 ms (direct link) and its web dashboard (`http://192.168.1.10`) answers
`HTTP/1.1 200 OK`. The `feed-noetic` container runs with **`--network host`**, so it has
identical access — confirmed with a raw Python `socket.connect()` to port 10000 (Kortex
Base RPC) and port 80, both succeeding, independent of any `ping` binary.

### Prerequisites installed into `ros_env`
- **`kortex_api-2.8.0.post5-py3-none-any.whl`** — Kinova's Kortex Python SDK, not on
  PyPI. Pulled from their Artifactory (`generic-local-public/kortex/API/2.8.0/`, found via
  the JFrog storage API since the UI is JS-rendered). It's a **pure-Python wheel** (no
  `cpXY`/platform tag), so the "does py3.11 have a matching build" question doesn't
  apply — one wheel, any CPython 3.5+, any platform.
  **Latent conflict, not fixed:** `kortex_api` hard-pins `protobuf==3.20.0`, downgrading
  the env from `5.29.6` and conflicting with the *declared* requirements of
  `google-ai-generativelanguage`/`grpcio-status`/`proto-plus`/`googleapis-common-protos`
  (pulled in by `anthropic`/`openai`/`google-generativeai`). Checked empirically:
  `rospy`, `feeding_deployment`, `anthropic`, `openai`, and `google.generativeai` all
  still import fine at 3.20.0 — benign for the arm-control path, but flagged as a
  pip-warned, unresolved conflict that could bite if something later exercises a
  newer-protobuf-only code path.
- **`iputils-ping`** (`apt-get install`) — `KinovaArm.__init__` (`kinova.py:114-123`)
  does its own `subprocess.run(["ping", "-c", "1", "192.168.1.10"])` pre-flight before
  touching Kortex at all; the base `osrf/ros:noetic-desktop-full` image doesn't ship it.
  Missing `ping` crashed `arm_server.py` with `FileNotFoundError: [Errno 2] No such file
  or directory: 'ping'` before any Kortex connection was attempted — not a networking
  problem, purely the missing binary. Installing it (container-local, no arm interaction)
  was the only blocker.

### `ARM_RPC_HOST` mechanism (commit `2c0e3498`)
Re-created the same fix the Jetson rig carries as an uncommitted local patch: NOW
committed as an env-var read with the lab default preserved --
`NUC_HOSTNAME = os.environ.get("ARM_RPC_HOST", "192.168.1.3")` in `arm_interface.py`.
`ARM_RPC_HOST=127.0.0.1` lets `arm_server.py`/its clients bind/connect on localhost when
client and server share a box, instead of the unreachable lab-NUC address. Verified both
the default (`192.168.1.3`, unset) and overridden (`127.0.0.1`) values resolve correctly.

### First launch: looked hung, wasn't — stdout buffering
First `arm_server.py` launch (buffered stdout, redirected to a log file) showed **zero
output for ~2.5 minutes** despite the process being alive and past the lock-file step —
looked exactly like a hang (leading candidate: the `clear_faults()` poll loop spinning
forever on a persistent fault). Sent `SIGINT` to investigate; the *entire* log --
including the eventual `"Arm manager server started"` line **and** the shutdown
sequence -- flushed out in one burst right at the kill. Diagnosis: `arm_server.py`
never sets `flush=True` and Python block-buffers stdout when it's not a TTY, so nothing
hits disk until the buffer fills or the process exits. It had actually completed
construction on its own **before** the kill arrived; real first-connection Kortex
session/actuator-enumeration latency, not an infinite loop. Confirmed by re-running with
`PYTHONUNBUFFERED=1`/`python -u`: the same sequence now streams live, and completed in a
few seconds on the (no-longer-first) reconnect.

### Read-only telemetry, verified sane
Connected via the same direct `ArmManager` pattern the existing `scripts/real_gen3_*.py`
already use (not `ArmInterfaceClient`, which blocks forever on
`rospy.wait_for_message("/watchdog_status", ...)` with no watchdog process running), and
called **only** `get_state()`:
```
joint positions (rad): [-0.0225 -0.2067  3.0458 -2.6496 -0.2642 -0.6645  1.7631]
joint velocity:        [0. 0. 0. 0. 0. 0. 0.]
EE pose (x,y,z,qx,qy,qz,qw): [0.1247 0.0562 0.1688 0.6676 0.7412 0.0634 -0.0288]
gripper_pos: 0.0087
```
Sanity checks all pass: 7 finite joint values, EE quaternion norm exactly 1.0, gripper
reads ~0.009 (matches the documented "0.009 = open" convention), `get_state()` returned
with no exception. Fault state: `clear_faults()`'s poll loop only exits once
`GetArmState().active_state == ARMSTATE_SERVOING_READY` — since construction completed,
the arm was fault-free at that point (no separate fault query needed/attempted).

### Clean shutdown, twice
Both sessions ended with `kill -INT <pid>` → `arm_interface_instance.close()` -->
`Base.Stop()` (no-op, nothing was moving) → session close → `/tmp/kinova.lock` removed.
Verified the lock file is gone after each stop. **No motion, no gripper action, no mode
change beyond what `KinovaArm.__init__` itself performs** (`SetControlMode(POSITION)`,
`SetServoingMode(high/SINGLE_LEVEL)`, `SetSafetyErrorThreshold(10 deg)` -- config writes,
not motion) was issued at any point this session.

### Also this session (same RoboStack env, see commits)
- `open_door.py:202` — added the `rviz_interface is not None` guard around
  `visualize_poses(...)` in `open_microwave()` (commit `a16222fb`). Verified: the real
  `execute_action()` dispatch (YAML load -> tree build -> tick, not a direct method call)
  now runs `open_microwave()` to completion in replay mode (`NullSimulator`,
  `robot_interface=None`, a synthetic `handle_opening_pos.pkl`) with no exception --
  previously hit exactly this line as `AttributeError: 'NoneType' object has no attribute
  'visualize_poses'`.

---

## 2026-07-21 — native install blocker: Noetic-3.8 vs PRPL-3.10 (feed-noetic container)

**Not the arm/microwave rig** — this session used a separate Docker container,
`feed-noetic` (`osrf/ros:noetic-desktop-full` image, Ubuntu 20.04, **Python 3.8.10**)
on host `Pachirisu` (Ubuntu 24.04), repo bind-mounted at the same path inside the
container. Goal: get `pip install -e ".[robot]"` to succeed under stock ROS Noetic's
system Python. No arm hardware touched.

### Result: genuine, unfixable-via-pins blocker found, then unblocked via RoboStack

`pyproject.toml`'s `[robot]` extra is unversioned, so pip on a 2026 index backtracks
into source builds / unbuildable transitive deps. Fixed those first (see
[`constraints.txt`](constraints.txt), repo root):

| pin | reason |
|---|---|
| `pin==2.7.0` | newest releases dropped py3.8 wheel support here → source build |
| `ruckig==0.9.2` | same — no cp38 wheel for newest releases |
| `openai==1.55.0` | unpinned dep resolves to newest releases with a huge transitive graph → slow/failing backtracking |
| `tokenizers==0.20.3` | pulled in transitively by `anthropic` (unpinned). Newest `tokenizers` (0.21.0) has no cp38 wheel here, so pip builds from source; that build backend then requires `puccinialin` (a Rust-toolchain bootstrapper with **no PyPI distribution at all**) → hard failure. 0.20.3 has a prebuilt cp38 wheel, avoiding the source build entirely. |
| `anthropic==0.34.2` | unpinned, backtracks through ~70 historical releases (down to 0.2.x) resolving transitive deps — minutes of wasted resolution time per attempt |

With those pins, resolution succeeds and fails at exactly **one** remaining error:
```
ERROR: Package 'prpl-utils' requires a different Python: 3.8.10 not in '>=3.10'
```
Checked and confirmed this is not pin-fixable:
- `prpl-utils` / `pybullet_helpers` / `relational_structs` are pulled via **direct git
  URL** in `pyproject.toml` (from the `prpl-mono` monorepo), not resolved from PyPI —
  constraints.txt can't override what a URL-pinned requirement installs.
- Checked **every commit** in each package's history (via GitHub API, back to their
  first commit) and **every PyPI release** (0.0.1–0.1.1, they're also published
  there): `requires-python = ">=3.10"` in 100% of them, no exceptions.
- `tomsutils` (separate repo, `tomsilver/toms-utils`, also git-URL-pinned): `>=3.9` at
  its oldest commit, tightened to `>=3.10` later — never compatible with py3.8 either.
- Container only has `/usr/bin/python3.8` — no 3.9/3.10/3.11 installed (expected: ROS
  Noetic's Ubuntu 20.04 base ships py3.8 as system Python).

So: **stock Noetic (py3.8) and the PRPL deps (py3.10+) are irreconcilable in the same
native environment.** No version pin, git ref, or PyPI release bridges that gap.

### Fix: RoboStack (conda-forge-packaged ROS) on Python 3.11

RoboStack packages ROS distros against modern conda-forge Python builds, decoupling
the ROS version from the OS's system Python. Verified end-to-end in the same
container:

1. Miniforge → `/opt/miniforge3`.
2. `mamba create -n ros_env python=3.11` (not 3.10 — `robostack-staging`'s
   `ros-noetic-desktop-full`/`ros-noetic-ros-base` builds only target py3.9/3.11/3.12,
   no py3.10 build exists; 3.11 still satisfies PRPL's `>=3.10`).
3. Channels: `robostack-staging` (priority) + `conda-forge`.
4. `mamba install ros-noetic-ros-base` — used the lighter `ros-base` metapackage, not
   `desktop-full` (don't need rviz/gazebo/GUI tooling for rospy/roscomm). **Gotcha:**
   the first `desktop-full` attempt aborted mid-transaction on a transient download
   timeout (two packages, `roswtf`/`rqt-robot-steering`) and mamba silently rolled
   back the whole env — `rospy` wasn't actually installed despite the wrapping shell
   command reporting exit 0 (a trailing `echo` after the real command masked mamba's
   failure). Bumped `remote_max_retries`/`remote_read_timeout_secs` and switched to
   `ros-base`; it completed cleanly on retry.
5. `python -m pip install -e ".[robot]" -c constraints.txt` inside `ros_env` —
   **succeeded outright**, including editable-building `prpl-utils`, `pybullet_helpers`,
   `relational_structs`, `tomsutils` (they're pure Python; `>=3.10` was the only gate).

Re-verified explicitly, all four in the **same** interpreter:
```
$ python -c "import rospy; print(rospy.__file__)"
/opt/miniforge3/envs/ros_env/lib/python3.11/site-packages/rospy/__init__.py
$ python -c "import pybullet_helpers, relational_structs, prpl_utils; print('prpl ok')"
prpl ok
$ python -c "import feeding_deployment; print('repo ok')"
repo ok
$ python --version
Python 3.11.15
$ which roscore
/opt/miniforge3/envs/ros_env/bin/roscore
```
`rospy`, the PRPL deps, and the repo itself all import from the same
`/opt/miniforge3/envs/ros_env/` interpreter — genuinely unblocked. This is the path to
running the real `bulldog`/rospy-based executive (not the bypass) and the full HLA
system, without droppping either Noetic or the PRPL deps.

### Not yet done
- Only `ros-base` installed, not `desktop-full` — `rviz_interface.py` will still need
  `rviz` (not in `ros-base`) if/when that's exercised; separate from the existing
  `rviz` `None`-guard TODO at `open_door.py:202`.
- This env is untested against real ROS master / catkin workspace builds (`roscore`,
  `catkin_make`, message generation for `feeding_deployment_msgs`) — only Python-level
  imports were verified here.
- Not yet tried on the Jetson rig (`LOCAL_DEPLOYMENT.md`'s box) — that one already has
  a working py3.10 `.venv` path via user-site Kortex SDK; RoboStack would be an
  alternative there too if a real rospy/bulldog is ever needed on that box.

---

## 2026-07-14 — autonomous detect → grasp → open

### Result
**Full autonomous microwave-door open works end-to-end:** live detect → handle in
arm-base frame → depth/lateral-corrected grasp → joint-space arc opens the door ~90°
→ release → retract. Grip holds through the whole swing.

### Perception → grasp (one script)
Grab RealSense (D435I, eye-in-hand) via `pyrealsense2` → GroundingDINO detect
`microwave` (CPU) → deproject box → open3d `segment_plane` → protruding cluster
(DBSCAN) → handle centroid → camera→arm-base via the **easy_handeye2** calib
(`~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib`, eye_in_hand) chained
with live `get_state()` EE pose (replaces tf2). **No SAM/rospy/tf2.**

Empirical corrections (from teleop ground-truth), baked into the grasp:
| const | value | why |
|---|---|---|
| `DEPTH_CORR` | `0.16` | perception overestimates depth ~16 cm; scale the handle ray in by 16 cm |
| `LAT_CORR`   | `0.07` | protruding-cluster centroid sits ~7 cm off the latch; shift −y |
| `GRIP_EXT`   | `0.065`| grasp EE = handle − 6.5 cm along approach. 6.5 (vs earlier 9) = the proven "grip a bit more forward → firmer" fix; grip then held through the swing |

Grasp lands within ~1–2 cm. Sequence: `open_gripper` → move to pre-grasp (handle −
`GRIP_EXT+0.10`) → step in over 4 sub-steps (tracking-abort > 2.5 cm) → `close_gripper`
→ **pause for a human grip check** before opening.

### Seeded-IK fix (wrist-flip prevention)
Symptom: a large move (e.g. back-off after opening) made the wrist "flip all the way
around". Root cause: `p.calculateInverseKinematics` was seeded from the sim's default
(home) config, not the arm's actual pose, so it returned a far-away joint solution.

Fix: **before each IK call, seed the sim from the arm's current real joints** —
```python
for i, jj in enumerate([1,2,3,4,5,6,7]):
    p.resetJointState(rb.robot_id, jj, ai.get_state()["position"][i],
                      physicsClientId=rb.physics_client_id)
sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id, pos, quat, ...)
```
IK then returns a config near the current one → no flip. Confirmed: the detect+grasp
run this session made no weird wrist moves.

**Caveat — don't seed mid-arc.** Seeding is too conservative on the small arc
waypoints: it stalled on door-arc step 2 with sim `ik_err 3.5 cm` (unseeded/home IK
reaches those exactly). Rule: **seed for large jumps (approach, retract, back-off);
use unseeded IK for the arc waypoints**, or try seeded first and fall back to unseeded
when `ik_err > 2 cm` (also guard against a big joint-jump = flip).

### Door-opening arc (the runner)
Uses the **repo's own** arc geometry — `PerceptionInterface._generate_door_arc_waypoints`
(no `self`, invoked unbound) with the microwave params `arc_length_m=0.55`,
`waypoint_spacing_m=0.05`, `direction=-1` (left-hinged → handle sweeps −x toward the
arm, **not** into the microwave), `rotate_orientation=True`. Start = the real grasp
pose; hinge estimated one door-width (`+0.32 m`) to `+y` (the wrist cam can't see the
door while grasping).

Each waypoint: IK → `set_joint_position` (joint control is reliable where
`set_ee_pose`/cartesian aborts at extended configs) → **wait for velocity ≈ 0, then
re-check the EE** (a move can return before it settles) → tracking-abort if the EE is
> 2.5 cm off (door bind / latch / hinge limit → natural stop) → **1 s pause** between
steps. Reproduce with a heredoc that:
1. reads current EE (the grasp pose) → `start_pose`, hinge = `(x, y+0.32, z)`;
2. `wps = _generate_door_arc_waypoints(None, start_pose, hinge, 0.55, 0.05, direction=-1, rotate_orientation=True)`;
3. builds the sim (`create_scene_description_from_config(".../configs/vention.yaml","skewer")` → `FeedingDeploymentPyBulletSimulator`);
4. per waypoint: IK (see seeding rule above) → `set_joint_position` → settle-wait → tracking check → 1 s pause.

This session's arc: **step 1 opened cleanly** (handle pulled to `[0.532,-0.118,0.48]`,
grip 0.99 held); step 2 stalled only because IK was seeded (see caveat) — not a
hardware problem.

### Speed / safety
`set_speed("low")`, hand on the physical e-stop, camera rig for another project sits
under the wrist so keep z raised and sweep forward/away from the base.

### State at end of session
Arm was **holding the door grasped** (gripper ~0.99) at ~arc step 1, paused. To reset:
restart `arm_server.py` + re-run `bulldog_bypass.py`.

### Next step
Finish the arc with the seeded→unseeded IK fallback, then fold detect→grasp→open into
the `open_microwave` HLA (needs the `rviz` `None`-guard at `open_door.py:202` + perception
wired in without `PerceptionInterface`/rospy).

---

## Earlier (see git log + `LOCAL_DEPLOYMENT.md`)

- 2026-07-09: arm bring-up — torque zeroing (`kinova.py`), `arm_server.py`, stub base +
  bypass, motion ladder rungs 1–4 (connect, telemetry, +5° joint, +5 cm cartesian).
- 2026-07-14: live rospy-free perception (RealSense + GroundingDINO CPU), the
  easy_handeye2 calib vs the wrong lab `sensors.launch` extrinsic, the ~16 cm depth bias.
