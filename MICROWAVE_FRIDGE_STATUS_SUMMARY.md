# Microwave + fridge task status summary

(Written 2026-07-30. Self-contained — pulls in the relevant history from
`TESTING_LOG.md`, `LOCAL_DEPLOYMENT.md`, and `CLAUDE.md` plus today's live Pachirisu
session, so this file can be read/transported on its own.)

## Microwave task

### How much has to change vs. the original repo
Surprisingly little. Diffing against the actual fork point (`a9e707bf`, where this
branch diverged from `upstream/main`): only **5 existing files** got touched, each with
small surgical fixes:
- `actions/open_door.py` — added an `rviz_interface is None` guard (3 lines) so the HLA
  doesn't crash with no RViz running.
- `control/robot_controller/arm_interface.py` — made the NUC hostname an env-var
  override (`ARM_RPC_HOST`) instead of hardcoded `192.168.1.3`; added a read-only
  `get_arm_state()` diagnostic.
- `control/robot_controller/kinova.py` — un-commented an existing debug print
  (temporary, for diagnosing a `METHOD_FAILED` abort).
- `perception/grounded_sam.py` — made SAM load lazily (the appliance/handle path never
  needs it — saves GPU/CPU memory on constrained boxes), fixed the GroundingDINO config
  path to point at the installed pip package instead of an assumed clone at
  `/home/isacc/Grounded-Segment-Anything`, added a CUDA-actually-works probe (torch
  `cuda.is_available()` can lie).
- `perception/appliance_perception/appliance_perception.py` — mostly black/isort
  reformatting noise; the real substance is a depth-cast bug fix (a `*1000`
  double-scaling that wrapped `uint16` and silently corrupted logs) and a guard for a
  DBSCAN-on-empty-list crash.

**The core detection algorithm (plane-fit + DBSCAN protruding-cluster) is untouched
original code.** Everything else — roughly 15 new scripts under `scripts/` and
`scripts/scratch/` — is orchestration that calls the existing repo classes
(`PerceptionInterface`, `AppliancePerception`, `ArmInterfaceClient`,
`FeedingDeploymentPyBulletSimulator`) from outside the HLA/behavior-tree system. None of
it is new robot logic — it's wiring plus a set of empirically tuned constants (see
below), tuned per physical rig.

Also worth noting: the **pre-existing repo already has hardcoded personal paths** from
prior lab members baked in — `/home/isacc/...`, `/home/rj277/...`, `/home/emprise/...`
in `ros_wrapper.py`, `remote_molmo.py`, `deca_perception.py`. Pre-existing tech debt,
not part of this fork's work.

### Hardcoded / rig-specific values (not portable across rigs)
| Constant | Value | What it papers over |
|---|---|---|
| `DEPTH_CORR` | 0.16 m | Perception systematically overestimates handle depth ~16cm |
| `LAT_CORR` | 0.07 m | Protruding-cluster centroid sits ~7cm off the true latch point |
| `GRIP_EXT` | 0.065 m | Grasp point = handle − 6.5cm along approach (tuned from 9cm after grip slipped) |
| `DOOR_W` (Jetson) | 0.32 m | Blind guess for hinge offset — no real hinge detection |
| `arc_length_m` (Jetson) | 0.55 m | Tuned for the 0.32m guess; wrong radius on Pachirisu's real microwave (~12cm) |
| `direction` sign | `-1` (Jetson) vs `+1` (Pachirisu) | Repo's own default for `"microwave"` is rig/mirror-layout-specific, not universal |
| `GRASP_QUAT` approach axis (repo constant) | fixed quaternion | Found 180° backwards on Pachirisu — latent defect in a repo-shipped constant |
| `GRASP_QUAT` roll (see below) | — | Never independently validated; needed a further +180° empirical correction today |
| easy_handeye2 / OpenCV calib files | rig-specific `.calib`/`.json` | Camera-to-base transform, non-portable, re-derived per box |

### Full timeline of what's worked / what hasn't

**Jetson (Orin Nano + Gen3, no NUC/base/ROS):**
- Hit a real wall: **IKFast build hangs indefinitely** on this box (custom Gen3 URDF,
  aarch64/8GB) — the repo's `plan_to_ee_pose`/sim-cartesian path is unusable here.
  Worked around with PyBullet native `calculateInverseKinematics` +
  `resetJointState` instead.
- GroundingDINO on the Tegra iGPU hits an NVML assert — runs on **CPU only**
  (~34s/frame).
- **2026-07-14: full autonomous cycle achieved end-to-end** — live detect → handle in
  arm-base frame (easy_handeye2 calib + live FK, no tf2) → depth/lateral-corrected grasp
  → open the door ~90° via a joint-space arc → release → retract. Grip held through the
  whole swing.
- **Seeded-IK fix** discovered here: before every IK call, seed the sim from the arm's
  *current real joints* so IK returns a nearby config, preventing a large-move
  wrist-flip. **Caveat found the same session:** seeding is too conservative *mid-arc*
  — it stalled on a small arc waypoint (sim ik_err 3.5cm); unseeded (home-seeded) IK
  reached the arc exactly. Rule of thumb settled on: seed for big jumps, unseeded (or
  seeded-with-fallback) for small arc waypoints. This caveat later turned out **not to
  generalize** — see the 2026-07-28 Pachirisu incident below.

**Pachirisu (RTX/24.04 + RoboStack, same physical arm):**
- **2026-07-21 (native-install blocker):** ROS 1 Noetic (targets Python 3.8/Ubuntu
  20.04) vs. this repo's newer (PRPL, Python 3.10+) dependencies is a genuine,
  unfixable-via-pins conflict on a native install. Fixed via **RoboStack** (conda-forge
  packaged ROS) giving real `rospy` on Python 3.11 in one interpreter — impossible on
  the Jetson.
- **2026-07-21 (first bring-up):** network/arm reachability verified read-only;
  `ARM_RPC_HOST` env override committed; `kortex_api` installed (pins
  `protobuf==3.20.0`, a latent-but-benign conflict with `google-generativeai`/
  `anthropic`, flagged not fixed).
- **2026-07-21 (later, first commanded motion + GPU vision):** first real joint motion
  (J5 wrist, small angles) via a careful propose→go→execute→poll→re-verify ladder. Hit
  (and got past, never fully root-caused) a `METHOD_FAILED` abort on every
  `REACH_JOINT_ANGLES` call, including a zero-delta one, despite
  `ARMSTATE_SERVOING_READY` — started working right after interacting with the arm's
  web dashboard, which may hold some kind of control lock `GetArmState()` doesn't
  surface. Also: real GroundingDINO Swin-B ran on the RTX 5070 GPU, detecting the real
  microwave + handle at confidences matching the Jetson's documented range.
- **2026-07-22 (eye-in-hand calibration):** `cv2.calibrateHandEye()` (not
  `easy_handeye` — not a RoboStack binary, normally needs MoveIt which this repo doesn't
  use) via a physical ArUco board. Self-consistent to ~2cm across 17 poses; validated
  directly on the real robot at **1.1mm**. Same session: a real USB host-controller
  crash (`xHCI host controller not responding, assume dead`, PCI `0000:00:14.0`) dropped
  *8 devices at once*, not just the camera — fixed via
  `unbind`/`bind` on the PCI driver (needed `sudo dmesg` to diagnose, several attempts
  to fix). Also a clean recovery from an accidental physical e-stop.
- **2026-07-22 (later, recurring USB failures):** the same full-controller crash
  happened 3 times in one session, including during plain (non-`align_depth`) streaming
  that had been rock-solid the prior session — ruled out `align_depth` as the specific
  trigger; concluded the camera's USB connection was degrading with repeated cycling.
- **2026-07-22 (evening, depth stable + handle localization succeeded, but a ~20-27cm
  error found):** `align_depth` ran a whole session with zero crashes (luck, not a
  fix). The real `detect_handle_and_placement` (plane-fit + DBSCAN) worked, repeatable
  to 2.4cm — but a real-hardware check (user manually touched the arm to the physical
  handle) found the computed handle pose was off by **~20-27cm**, a large systematic
  error. Spent the session isolating the cause (ruled out fingertip offset, bad frames,
  depth/plane-fit math, and — despite a suspicious lead — translation magnitude of the
  calibration).
- **2026-07-27 (recalibration root-caused + fixed to 5.3cm):** root cause was
  **`cv2.calibrateHandEye()`'s TSAI method being numerically unstable on this data** —
  confirmed by comparing all 5 OpenCV hand-eye methods on the same samples; PARK/
  HORAUD/ANDREFF/DANIILIDIS converge tightly, TSAI alone was a 50cm+ outlier. Recaptured
  11 poses with real translation diversity (max 57cm vs. the old set's 5cm), solved with
  PARK. Manual-touch validation closed the error to **5.3cm** (likely just the known
  gripper-fingertip-to-wrist offset). Same session: found `arm.set_ee_pose()` (Kortex
  cartesian) does **not** reliably "abort with no motion" as `LOCAL_DEPLOYMENT.md`
  claimed — it can attempt a real, partially-executed trajectory and hit
  `JOINT_ACCELERATION_LIMIT_REACHED` mid-move. Also: the Jetson-proven seeded-IK
  pattern's **IK solve itself failed to converge** for a hover target (18.5cm error) —
  a genuine solver/reachability issue, left unresolved this session. And: GPU can be
  full from another lab member's process — CPU fallback works fine for one-off calls.
- **2026-07-28 (IK convergence root-caused; hover+grasp validated; door-arc wrong
  twice; one real motion incident):** the 07-27 IK convergence failure was **not** a
  solver/seed issue — the repo's fixed grasp-orientation constant had its local-Z
  (approach) axis pointing **180° backwards**; re-pointing it at the true look-at
  vector took IK from ~17cm error to 0.01cm from every seed. With that fixed:
  hover approach landed within **0.7mm** (30cm back-off) and **3.7mm** (15cm back-off);
  gripper closed on the handle, visually confirmed. **Door-opening arc surfaced three
  problems:** (1) wrong hinge-radius assumption (blind 32cm guess vs. real ~12cm on
  this microwave); (2) wrong direction sign (repo's `-1` default for `"microwave"`
  swings away from the base at this rig's geometry, `+1` is correct here); (3) **most
  importantly**, the arc waypoints' IK — following the Jetson script's own "deliberately
  UNSEEDED is fine for small waypoints" note — found a Cartesian-correct but
  joint-space-**distant** alternate solution branch, and the real arm swept an
  uncontrolled ~150° reconfiguration on J1/J3/J7. Grip held (lucky), no fault latched,
  but this proved **the Jetson's "unseeded is fine" note does not generalize** — it
  depended on that rig's fresh-sim default happening to sit near its working posture.
  Fixed by seeding IK from the arm's real current joints **and** adding an explicit
  joint-space-delta guard (>25° aborts). A **fourth issue, still open:** the arc-model
  assumes a purely vertical hinge axis, but a user-demonstrated real waypoint was off by
  +10.5cm in y and **+5.8cm in z** — this microwave's door visibly rises as it opens
  (likely a multi-bar/lift-type hinge), not a simple pivot.
- **2026-07-30 (today, live session):** brought the full stack up fresh (roscore,
  camera, arm server, stub base, bulldog bypass) after finding nothing from the prior
  session was still running.
  - **Real USB fault, worse than previously logged:** `lsusb` showed **zero devices at
    all** (not even keyboard/mouse) at one point — a full xHCI controller fault. Fixed
    via `unbind`/`bind` on the PCI driver (matching the 07-22 fix), without rebooting —
    important since another lab member (niharika) had an active session + GPU job
    (`serve_policy.py`, ~10.8GB VRAM) that a reboot would have killed with no warning.
  - Camera then still wouldn't stream frames (`control_transfer` errors climbing,
    zero frames) even after the controller-level fix. Root cause: **USB autosuspend was
    on** for the camera (`power/control=auto`). The repo already ships a fix for this
    exact symptom — `scripts/usb_hardening.sh` / `config/udev/99-usb-hardening.rules`
    (from earlier upstream work, `extend usb autosuspend hardening to owc tb4 hub
    chain (suspended camera breaks stream start)`) — targets this camera's exact USB ID
    (`8086:0b3a`). Running it fixed the stream (stable ~30Hz color+aligned-depth). Known
    gap (documented in the script itself): a hardware reset/re-enumeration resets
    `power/control` back to `auto`, so it needs re-applying after every such event —
    happened twice more this session and was re-applied each time.
  - Detection re-confirmed working (0.53-0.65 confidence across 3 fresh trials with the
    v2/fixed calibration), red action-point overlay visually correct on the real chrome
    handle bar each time.
  - Hover approach: a **single seeded-IK jump** to the hover target converged
    positionally (~1cm) but wanted a **75-85° joint-space jump** — the same
    branch-jump danger as the 07-28 arc incident, just this time on the *approach*
    phase. Fixed the same way: a **chained/interpolated path** (12-24 small steps, each
    seeded from the previous), verified in sim (all steps <25° delta) before every real
    motion. Executed cleanly multiple times, sub-cm tracking error throughout.
  - **New finding: grasp roll/orientation was never independently validated.** The
    07-28 fix only corrected the approach *axis*; a live visual check (via video) found
    the gripper was oriented wrong for this microwave's **vertical** handle bar — needed
    an empirical **+180° roll** correction (found in two steps, +90° then another +90°,
    while watching camera feedback) before it looked/worked correctly. Root cause: the
    repo's original fixed `GRASP_QUAT` constant's roll was almost certainly tuned for a
    different handle/rig; the shortest-arc axis correction preserves whatever roll the
    old (wrong) constant had, which is coincidental, not verified against any real
    handle's actual geometry.
  - Executed hover → grasp-approach → gripper-close as three separate, continuous
    (non-stepwise) motions per request, each verified in sim first; all landed
    within ~0.5cm.
  - **Door-arc: wrong-direction incident, real e-stop.** Reusing the fixed direction
    (`+1`) and radius-based arc sizing from 07-28 but computing the hinge-offset
    relative to the arm's *current* grasp point (which had shifted from a manual
    backoff + Xbox teleop adjustment since detection), the swing went the wrong way on
    camera. User stopped it (hard e-stop — confirmed via `BrokenPipeError`/
    `Connection reset by peer` in the arm-server log, the same signature as the 07-27/
    07-28 incidents). Not root-caused before the session ended; recovered cleanly
    (restart `arm_server.py`, clear the stale `/tmp/kinova.lock` — a zombie process
    made the staleness check think the old server was still alive — re-run
    `bulldog_bypass.py`).
  - Set up (but did not run) a git worktree of the pristine pre-fork code
    (`a9e707bf`) for a raw/unmodified-code comparison test; removed at end of session
    per request.

### Robustness, honestly
This is **not a robust, closed-loop system** — it is a research rig held together by
per-session manual verification and empirically-discovered corrections. Every real
motion goes through a propose → sim-verify → explicit human go-ahead → execute → check
cycle; nothing has run autonomously start-to-finish on Pachirisu yet without a human
catching a problem in the loop. Concretely, **every single Pachirisu session so far has
hit at least one new, previously-unknown failure mode** — a 20-27cm calibration bug, an
unresolved IK convergence issue, a 150° uncontrolled swing, USB controller crashes
(repeatedly, on different specific triggers each time), a wrong grasp roll, and a wrong
arc direction (twice, for two different underlying reasons). The Jetson achieved one
full unattended cycle in an earlier session (2026-07-14); Pachirisu has not yet
completed one.

### HLA — important gap
**None of this work touches the actual `OpenDoorHLA`/behavior-tree system**, on either
rig. Everything (detection, hover, grasp, arc) has been driven through bespoke scratch
scripts calling the underlying pieces directly (`ArmManager`, `PerceptionInterface`,
PyBullet IK), bypassing the HLA entirely. Folding this into the real HLA — so it's
plannable/composable with the rest of the executive rather than a one-off script — is
still fully ahead of us, and is probably the single biggest remaining piece of work,
separate from the physical-accuracy issues above. (The one relevant fix already
made: the `rviz_interface is None` guard in `open_door.py`, needed for the HLA to even
run without RViz.)

### HLA investigation (2026-07-30) — why we haven't been running the real HLA, tested empirically
After the arc incident, spent time actually checking (not assuming) how much of the
repo's own `OpenDoorHLA`/behavior-tree machinery could run on Pachirisu, given it has
real `rospy` (unlike the Jetson). Confirmed live, with imports/instantiation attempts
(no arm motion):

- **`rospy` and `tf2_ros` themselves import fine** on Pachirisu (RoboStack). This is a
  real improvement over the Jetson, which has no ROS at all.
- **`netft_rdt_driver` (the wrist force-torque sensor's ROS driver) genuinely does not
  exist here** — confirmed via a direct `import netft_rdt_driver` failure
  (`ModuleNotFoundError`), not just documentation.
- **`PerceptionInterface`'s ROS support is poisoned by that one missing package, even
  though `rospy`/`tf2_ros` are fine.** Its import block bundles `rospy`, `tf2_ros`,
  *and* `netft_rdt_driver` inside one `try/except ModuleNotFoundError` — one failure
  fails the whole group. Confirmed live: `perception_interface.ROSPY_IMPORTED == False`
  on this box, purely because of the bundled `netft_rdt_driver` import.
- **`ArmInterfaceClient` gets further** — its own import block doesn't bundle
  `netft_rdt_driver`, so its `ROSPY_IMPORTED == True`. But constructing it live, it
  prints `"Waiting for Watchdog status..."` and then **hangs forever** on
  `rospy.wait_for_message("/watchdog_status", Bool)` — confirmed by actually running it
  and having to kill it after a timeout.
- **The only thing that would ever publish `/watchdog_status`** (`safety/watchdog.py`)
  has an *unguarded* top-level `from netft_rdt_driver.srv import String_cmd` (unlike
  `perception_interface.py`'s guarded version) — so that module can't even be imported
  on this box, let alone run, meaning nothing can ever satisfy `ArmInterfaceClient`'s
  wait.
- **The actually encouraging finding:** `HighLevelAction.__init__` (the real base class
  `OpenDoorHLA` inherits from) just **stores** whatever `robot_interface`/
  `perception_interface` objects it's given — it's duck-typed, not hard-wired to the
  real `ArmInterfaceClient`/`PerceptionInterface` classes. So the HLA abstraction itself
  isn't blocked by any of the above — only the *specific concrete implementations* are.
  A thin adapter exposing the same method names (`move_to_joint_positions`,
  `open_gripper`, `close_gripper`, `detect_handle_and_placement`, etc.) but backed by
  the already-proven raw `ArmManager` + monkeypatched-transform `AppliancePerception`
  from this session's scratch scripts could plausibly be handed to `OpenDoorHLA()`
  directly — no `netft_rdt_driver`, no real watchdog needed. That's a genuinely open,
  buildable path, not a wall — just not yet attempted.

### Bottom line
The perception/calibration pipeline is in good shape (5.3cm accurate, repeatable
detection). The grasp-orientation and arc-geometry logic need principled (not
empirical trial-and-error) fixes — both have now been wrong in non-obvious ways on
*separate* occasions, which suggests the current approach (fix the symptom you can see,
move on) hasn't run out of hidden bugs yet. And the HLA integration hasn't been started
at all — but per the investigation above, it's not blocked so much as just not yet
attempted: a thin duck-typed adapter over the pieces already proven today is a concrete,
buildable next step, not a redesign.

---

## Fridge task

**All fridge validation in this repo's history predates this fork's single-machine
track.** `git log --grep=fridge` shows real prior work — `open fridge door`,
`fridge manipulation`, `pickup plate from fridge`, `fridge works?`, plus later
soft-stop/trajectory-blending work specifically for the fridge pregrasp — but that was
all on the **original multi-machine lab rig** (compute box + NUC + mobile base + ROS
Noetic natively). None of it has been run on the Jetson or Pachirisu single-machine
setups this fork lives in. Specifically:

- **The fridge workflow depends on the mobile base** (`navigate_to_fridge.yaml` — you
  drive up to the fridge first, then open it; there's also `pick_plate_from_fridge.yaml`
  and `place_plate_in_fridge.yaml` as later steps in the same workflow). The base is
  explicitly out of scope / parked and unpowered for this whole fork. So even before
  touching door-opening, the fridge task as originally designed needs a component
  we've deliberately excluded — unlike the microwave, which was chosen specifically
  because it doesn't need the base.
- **None of this fork's tuned constants transfer.** `DEPTH_CORR`/`LAT_CORR`/
  `GRIP_EXT`, the calibration, the grasp-roll fix — all specific to this microwave's
  handle and this camera mount. A fridge handle is physically different (the code
  already treats it as a distinct case — `_SWING_KY["bottom textured fridge door"]` vs
  `"microwave"` in `appliance_perception.py`), so it would need its own full
  empirical-tuning pass — same amount of effort the microwave has taken across five-plus
  sessions.
- **One thing that might actually be easier:** a lot of fridges have a genuinely
  vertical hinge axis, so `_generate_door_arc_waypoints`'s vertical-axis assumption —
  which is wrong for this microwave's lift-type hinge — might actually hold up fine for
  a fridge. That's a hypothesis, not verified.
- The original fridge work also had its own hard-won fixes worth knowing about if
  it's revisited: `soft_stop` (a tapered/interpolated approach for `move_to_ee_pose`)
  was added specifically for the fridge pregrasp to fix stop-and-go trajectory
  blending, then later **restricted to fridge-pregrasp-only** because a global default
  caused stop-and-go on unrelated free-space reaches. A reminder that fixes tuned for
  one appliance can actively regress others if applied too broadly.

**Net for the fridge:** essentially none of this fork's hardware-validation work
carries over. It would need its own bring-up from scratch, and first needs a decision
about whether to reintroduce the base or scope it down to an arm-reachable stationary
fridge.
