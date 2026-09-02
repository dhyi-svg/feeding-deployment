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

---

## 2026-08-22 — Jetson: table height changed; ZERO grasp motion — blocked by a dead rclpy executor in the detection server

**Nothing was grasped and the arc was never attempted.** The only motion all session was
one reposition to a saved viewing pose. `arm_commands_log.txt` for the whole grasp phase
contains only `set_speed` lines — every grasp attempt aborted in the detection stage,
before any joint command. No fault latched, no collision, no e-stop from a moving arm.

### The real blocker: the shared rclpy executor thread dies and nothing restarts it

`ros2/node.py` ran `_EXECUTOR.spin` **bare in a daemon thread**. If spin raises, the thread
dies silently and every callback on the shared node — camera *and* tf — stops forever. The
process keeps running and looks healthy.

Symptom chain, in the order it confused us:
1. `detect 1` succeeds; `detect 2` refuses with "Camera frame is Ns old".
2. `ros2 topic hz` shows the camera publishing fine the whole time (~10-15 Hz).
3. Probing the service again minutes later returned **296.7 s** where it had said 59.8 s —
   the stored frame was *frozen*, not merely stale. That is what proves the subscriber is
   dead rather than the camera being slow.

Two separate triggers were seen: a long (~60 s) GroundingDINO pass holding the GIL, and
restarting the camera node underneath a live detection server (the subscription never
re-matched to the new publisher).

**Fixed** (`ros2/node.py`): spin now runs under `_spin_forever()`, which logs and restarts
on exception, plus `executor_alive()`. `detection_service.py`'s stale-frame error now names
which half broke ("the rclpy executor thread is DEAD" vs "the camera has stopped
publishing"). **Patched but NOT yet proven on hardware** — the server was reloaded with the
fix and the session ended before a two-detection run could confirm it. That confirmation is
step one tomorrow.

### Also fixed: detection server refused every second detection

`DetectionService.detect()` sampled the frame once and refused if stale. Since a detection
holds the GIL ~60 s, the stored frame is *always* stale immediately afterwards, so back-to-
back detections could never both succeed. Now polls up to `max(limit, 10 s)` for a fresh
frame before refusing. This one **is** proven — it took a run from "refused" to two looks
agreeing to 0.4 cm.

### DDS was pinned to the arm's ethernet

`~/ros2_ws`'s setup.bash exports `CYCLONEDDS_URI` pointing at a config that pins DDS to
`enP8p1s0` — the point-to-point arm link — so all local ROS traffic was forced over it
(hundreds of `ddsi_udp_conn_write ... failed` lines). `config/cyclonedds_local.xml` (already
in the tree, untracked, never exported) fixes it: export it and the failures go to **zero**.
Not the cause of the frozen-frame bug, but it must be set. The arm RPC is TCP to
192.168.1.10, unaffected by loopback-only DDS.

### `goto_preset.py` was broken — called a method that does not exist

It connects with a raw `ArmManager` proxy but called `arm.execute_command(JointCommand(...))`,
which only exists on `ArmInterfaceClient`. Fixed to `arm.set_joint_position(target.tolist())`
(what the client forwards to anyway). It then drove the arm to `microwave_approach_start_pos`
with **0.01 deg** residual, EE matching the recorded capture to 0.1 mm.

### Table height changed — two constants are now wrong

The microwave's table was lowered ~15 cm and **cannot be put back**.
- `PLAUSIBLE_Z` floor lowered 0.35 -> **0.25** (`real_gen3_ros2_grasp_microwave.py`), at the
  user's direction, after it refused a *good* detection at z=0.320. Deliberate loosening of
  a guard; the x/y bounds were untouched and still passed comfortably.
- **`top_of_appliance` is now WRONG and must not be trusted**: it returns z≈0.216, *below*
  the handle at z≈0.316. Cause is visible in the log — `Top of plane pixel: (463, 505)` in a
  **480-row image**, i.e. off the bottom of the frame. The steep downward view from the
  lowered table clips the door face. It gates nothing today and nothing reads
  `/tmp/microwave_top_z.txt`, but it drives `post_release_pose` (lift to top + 5 cm) the
  moment that is automated — it would send the gripper **down**.

### Detection itself is healthy in the new configuration

Worth stating plainly, because the framing looked bad by eye and was not:
handle `[0.5565, -0.2953, 0.3159]`, two looks agreeing to **0.4-0.5 cm**, plane verticality
**1.00**, elongation 6.0, 2768 pts, viewing distance **40 cm**, confidence 0.58-0.59. The
full dry run passed every gate: IK error **0.00 cm** on both moves, `--steps 12` giving
worst sub-steps of 5.4 deg (pre-grasp) and 1.6 deg (grasp), endpoints 0.001 cm from the
gated target.

### Other things found
- **Bulldog heartbeat starves during the detector's model load** and latches an e-stop
  (0.3 s beat vs 1.0 s timeout, ~2.4 GB paging off microSD). Order matters: bring the
  detector up **before** `bulldog_bypass.py`, or take bulldog down during a reload.
  JETSON_TESTING.md §2 says start the detector first — correct, but impossible as written,
  because `DetectionService.__init__` waits for camera frames before loading the model, so
  the `ros2 launch` has to precede it.
- **Duplicate `robot_state_publisher` + `calibration_tf` survived a launch restart**, two
  publishers on the same TF frames at once. Verify with `ps` after any launch restart.
- `ros2 launch` needs the `~/ros2_ws` overlay: Humble's own `robotiq_description` (2023) is
  too old for `kortex_description`'s xacro (`Invalid parameter "isaac_joint_commands"`).
  Sourcing `/opt/ros/humble/setup.bash` *after* the overlays shadows the fix.
- Arm ethernet (`enP8p1s0`) dropped twice — once before bring-up, once at session end.
  `carrier` = 0 is the check.

### State at end of session
Everything shut down; nothing left running. Arm powered off (link down), so final pose
could not be read — but it never left the viewing pose, and the gripper was open (0.0044)
at the last successful read. Microwave sits on the lowered table, roughly square to the arm.

### Next step
1. Confirm the executor fix: restart the stack, run the grasp **dry run**, and check both
   detections succeed. If a stale-frame error appears, it now says whether the executor
   died — that distinction is the whole point.
2. Then the grasp execute (dry run already passed every gate from this exact pose).
3. Before any arc: `top_of_appliance` is wrong, and the arc's assumed `+0.32 m` hinge was
   validated against the *old* table geometry — re-check both.


## 2026-08-12 (later) — Jetson: grasp succeeded, arc partially completed and pulled the microwave; `stop_action` FAILED TO STOP THE ARM

**Grasp: success.** Re-ran `real_gen3_ros2_grasp_microwave.py --steps 12 --execute` after
returning the arm to `MICROWAVE_HOME`. Gripper closed on the handle, visually confirmed.
`gripper_pos` read **1.0000** this time (0.4693 on the earlier confirmed grasp — see
above; the value does not distinguish a grasp).

**Arc: mostly completed, then had to be stopped.** `real_gen3_ros2_open_arc_microwave.py`
ran and swung the door most of the way, but **began pulling the microwave off the table**
and was stopped before finishing. The appliance is not fixed to the table, so once the
door reaches its limit the arc keeps pulling and the whole appliance moves. The arc's
per-waypoint tracking abort (2.5 cm) did not catch this — the EE was tracking its
commanded waypoints correctly; it was the *microwave* that moved, which nothing measures.

### SAFETY INCIDENT: `arm_stop_action.py` reported success and did not stop the arm

The documented "first response" abort did nothing. From `arm_commands_log.txt`:

```
12:45:11  stop_action              <- reached the server, client printed success
12:45:12  set_joint_position       <- the arc sent the NEXT waypoint 1 s later
12:45:15  set_joint_position
12:45:18  set_joint_position
12:45:21  set_joint_position
12:45:22  emergency_stop           <- bulldog kill; the only thing that worked
```

Two independent failures:

1. **`Base.StopAction()` did not stop the in-flight motion.** `move_angular()` commands
   via `base.ExecuteAction()`; `StopAction()` did not abort it. `Base.Stop()` — what
   `emergency_stop()` calls — is what actually stops this arm.
2. **Even a working abort would not have helped.** `stop_action` cancels one motion; the
   arc loop immediately commands the next waypoint. Nothing tells the *script* to stop.
   The bulldog kill worked because it **latches** `emergency_stop_active`, after which
   `set_joint_position`'s assert blocks every subsequent waypoint. **The latch is the
   load-bearing part, not the stop.**

`stop_action`'s own docstring says it was written to preempt a single long move from the
teleop recovery screen — it was never a general stop, and was wrongly presented as one.

**Standing rule until proven otherwise: the physical e-stop and killing
`bulldog_bypass.py` are the only stops. `arm_stop_action.py` is NOT a safety mechanism.**

### Follow-ups

1. A real soft stop needs `arm.stop()` (i.e. `Base.Stop()`) **plus** a latch that makes
   subsequent motion commands refuse — recoverable without restarting `arm_server`,
   unlike `emergency_stop()`. Must be verified in free space at low speed with nothing
   grasped before it is relied on.
2. The arc needs an abort for *appliance* movement, not just EE tracking error. The
   microwave is unsecured; a door that has hit its limit and an appliance being dragged
   look identical to the current guard.
3. Consider securing the microwave to the table for testing, and/or shortening
   `ARC_LEN` so the swing stops short of the door's hard limit.

---
## 2026-08-12 — Jetson: successful chained approach + grasp on the real handle (no arc attempted)

**Result: the first grasp using the new `--steps` chained-motion path completed.** Ran
`real_gen3_ros2_grasp_microwave.py --steps 12 --execute`. The script ran to its final
line (`CHECK VISUALLY before running the arc.`) with no aborts, so both gated moves
(pre-grasp, then grasp) executed and the gripper closed.

**No arc was run.** That is by design, not a failure — the grasp script's docstring says
it "does NOT run the opening arc", and the arc is a separate script. It stops after
closing so the grip can be checked first. The door was NOT swung this session.

End state:

```
EE      : [0.5423, -0.3461, 0.5195]     ~0.5 cm from the detected handle
gripper : 0.4693                        CLOSED, partially
```

### Grasp confirmed on the handle; `gripper_pos = 0.4693` at a successful grasp

The grasp was **visually confirmed successful** — the gripper closed on the microwave
handle.

`gripper_pos` read **0.4693** at that grasp, and **1.0000** at a second confirmed grasp
later the same day. Two successful grasps, two very different readings — so the inherited
guidance holds on this rig too: **`gripper_pos` cannot confirm a grasp.** Keep the human
check.

### Grasp height ran high

Observed across the successful grasps: the gripper closed on the handle but sat too high
on the bar. Relevant to the 2026-08-12 detection changes, which moved the reported handle
height down ~5 cm (`z` 0.517 -> 0.467) as a side effect of measuring protrusion from the
fitted plane. That shift is in the correcting direction.

### Detection

Two detections captured to `~/captures/grasp_exec` (`1__*`, `1_1_*`, 12:22 and 12:23).
The run passed the 3 cm agreement gate. Numbers themselves were lost — see below.

### Lost the stdout log (process gap, now fixed)

`tee ~/runs/$(date +%F)/02b_grasp.log` failed silently because `~/runs/2026-08-12/` did
not exist — the date had rolled over and `mkdir -p ~/runs/$(date +%F)` was a separate
step in `JETSON_TESTING.md`, easy to skip. The detection agreement figure and the 24
per-step tracking values only ever existed in terminal scrollback.

`DETECTION_LOG_DIR` was unaffected and captured both detections, so the perception side
is still replayable via `scripts/session/replay_detection.py`.

Fixed by chaining `mkdir -p ~/runs/$(date +%F) && ` into every logged command in
`JETSON_TESTING.md` rather than relying on a preceding step.

### Also this session

- **Detection server built** (`scripts/session/detection_server.py`,
  `perception/detection_service.py`). Startup was measured, not guessed: imports 43 s,
  `GroundedSAM()` 42 s warm / ~110 s cold, and ~2.4 GB of libraries and weights paged off
  a **13.5 MB/s** microSD — ~3 min of disk before any compute. That startup is
  per-invocation and not per-detection, which is why a run took ~8 min. With the server
  resident a run should be ~73 s. **Not yet exercised on hardware** — this session's run
  used the in-process fallback.
- Both task scripts now prefer the server and lazy-import the heavy modules; dropping
  `perception_interface` (27.8 s, used only for two four-line matrix helpers, now
  inlined) took the client import set from 11.9 s to 1.8 s. Pre-grasp geometry verified
  unchanged to **0.000 mm** against a previous run.
- `--max-detects` (default 5): keep detecting until two agree, instead of aborting the
  whole run on one bad pair. Added after a run where two detections disagreed by 20 cm —
  both at ~0.60 confidence, so **confidence did not distinguish the good detection from
  the bad one**. The captured overlays showed the bad one had clustered onto the table
  clutter below the microwave, which out-voted the handle because DBSCAN takes the
  largest cluster.
- `docs/position_locked_assumptions.md` written: 15 things that break when the appliance
  moves or rotates, each traced to upstream or this branch. Nine are upstream. RANSAC's
  `plane_model` is computed and discarded (upstream too) — using it would fix three of
  them at once.

### Next

1. The arc — hand on the e-stop, `DOOR_W = 0.32` hinge and
   `DIRECTION = -1` are confirmed correct-side for this layout (verified against the
   saved camera→base transform: image-right is base −y, so the hinge is at +y of the
   handle).
2. Exercise the detection server on hardware — it has never run against the live camera.

---
## 2026-07-31 — Pachirisu: real OpenDoorHLA ticked end-to-end (sim-only, duck-typed adapter) -- the 07-30 "thin adapter" plan actually attempted

Followed directly on 07-30's stated next step ("actually attempt the thin-adapter
approach into the real OpenDoorHLA ... now that it's confirmed buildable"). No arm
motion, no camera -- deliberately scoped to a sim-only wiring check first (see
`scripts/scratch/sim_open_microwave_hla_dryrun.py`).

### Result: it works
`OpenDoorHLA` was constructed with `robot_interface=None`, `NullSimulator` (the
existing `--no_waits` sim stub `run.py`/`test_navigate_action.py` already use), and a
~15-line stub `perception_interface` implementing only `perceive_handle_opening_poses`
(no camera, no ROS, hardcoded placeholder poses). `execute_action()` on it -- the real
production path: load `open_microwave.yaml`, resolve its `!hla open_microwave` tag,
tick the tree -- ran the entire `open_microwave()` control flow (both the pull-open
phase and the previously-never-attempted push-open phase) to completion with zero
exceptions. Confirms the 07-30 "duck-typed, not hard-wired" hypothesis empirically, not
just at the import level: the real HLA/behavior-tree machinery runs against Pachirisu
with no `ArmInterfaceClient`/`PerceptionInterface`/`netft_rdt_driver` in the loop at all.

Note on the stub: with `robot_interface=None`, every `move_to_*` method's real-robot
branch (the only place that reads `Pose` content or calls `_validate_ee_pose`) is
unreachable -- `move_to_joint_positions`/`move_to_ee_pose`/`move_to_ee_pose_trajectory`
all fall through to `sim.visualize_plan(...)`, which ignores its argument as long as the
sim is `NullSimulator`. So this run validates *control flow*, not pose values -- the
stub's `pre_grasp_pose`/`grasp_pose`/etc. are structural placeholders (needed only so
dict keys exist and `push_waypoints[-5]` has length), not something to reuse for a real
attempt.

### New blocker found and fixed: `feeding_deployment.actions.base` doesn't import in `ros_env` without a second, undocumented source step
Importing `feeding_deployment.actions.base` (needed for **any** HLA, not just
`OpenDoorHLA` -- it unconditionally imports `ArmInterfaceClient` at module level for a
type hint) failed with `ModuleNotFoundError: No module named 'feeding_deployment_msgs'`
-- `arm_client.py` -> `safety/collision_threshold.py` -> `feeding_deployment_msgs.srv`.
This is a catkin *message* package (needs code generation, not just pip), and unlike
`ros-base` (installed via RoboStack/mamba) it isn't part of the conda env at all -- it
lives in a **separate, already-built catkin workspace** at `/opt/msgs_ws`
(`/opt/msgs_ws/devel/lib/python3/dist-packages/feeding_deployment_msgs`), invisible to
`ros_env` until `source /opt/msgs_ws/devel/setup.bash` is run *after* `conda activate
ros_env`. Fixed by sourcing it; not previously written down anywhere (the "Setup --
talking to the arm" section above predates Pachirisu and doesn't mention it). The
07-30 session's claim that `ArmInterfaceClient`'s "own import block... doesn't bundle
`netft_rdt_driver`" and "gets further" must have had this already sourced in that
terminal without writing it down -- worth carrying forward as a standing setup step for
any future `ros_env` session that touches `feeding_deployment.actions.*` or
`arm_client.py`.

### Bonus finding: pure-PyBullet-sim mode (not `NullSimulator`) crashes on the first move, repo-wide
Re-ran the same script with a real `FeedingDeploymentPyBulletSimulator(use_gui=False)`
instead of `NullSimulator` (still `robot_interface=None`, still no hardware) to check a
suspicion from reading `base.py`: every `move_to_joint_positions`/`move_to_ee_pose`
method has its planning call commented out (`plan = None`, `# plan =
self.sim.plan_to_...`), so `self.sim.visualize_plan(plan)` is always called with
`plan=None` in pure-sim mode. `FeedingDeploymentPyBulletSimulator.visualize_plan()` does
`for sim_state in plan:` -- confirmed it throws `TypeError: 'NoneType' object is not
iterable` on the very first move (`open_microwave`'s initial retract). `NullSimulator
.visualize_plan(*args, **kwargs): pass` is silently immune, which is presumably why this
has never surfaced: every existing sim-only harness (`test_navigate_action.py --no_waits`,
this session's script) already routes through `NullSimulator`, and nothing appears to
exercise `move_to_joint_positions`/`move_to_ee_pose` against a real (non-Null)
`FeedingDeploymentPyBulletSimulator` with `robot_interface=None`. Pre-existing, repo-wide
(not introduced this session, not Pachirisu-specific); left unfixed -- out of scope for
today, and the planning-disabled comments suggest it's mid-refactor rather than an
oversight. Flagging in case a future PyBullet-GUI demo (for visually filming a sim run)
hits it.

### Not done
- Live-detection version: swap the stub's hardcoded poses for a real
  `AppliancePerception.detect_handle_and_placement()` call (monkeypatched-transform
  pattern from `test_detect_handle_and_placement.py`) inside the adapter, so
  `perceive_handle_opening_poses` returns a real, live handle pose through the actual
  HLA path. Needs the full physical stack up (`feed-noetic` container was fully
  stopped at 07-30 end-of-session; container itself was restarted this session, but
  roscore/camera/`arm_server`/`stub_base_server`/`bulldog_bypass` were not -- no camera
  or arm access was attempted, by design, while unattended). Since `robot_interface`
  stays `None` either way, this would still be a no-motion, camera-only extension of
  today's test, not a step toward the real door-opening arc.
- The push-open phase's poses (`pre_push_pose`/`push_pose`/`push_waypoints`/closing
  waypoints) remain entirely unbuilt for this rig -- today's stub only proved the
  control flow *reaches* them without crashing, not that real values exist for them.
- No behavior-tree parameter edits (`ManipulationConfirm`, etc.) were needed or
  attempted; the stub perception adapter ignores confirm mode entirely.

---

## 2026-07-30 — Pachirisu: USB fault fixed for good (autosuspend), grasp roll bug found+fixed live, arc wrong-direction incident (again, different cause), HLA duck-typing investigated

Live session, user recording video for a working-demo / raw-code comparison. Full stack
brought up fresh (nothing survived from 07-28's session end). Got further on real
findings than on the video itself.

### USB: two independent faults, both fixed without a reboot

First, a **full xHCI controller fault** — `lsusb` showed zero devices at all (not even
keyboard/mouse), worse than any previously-logged instance. Fixed via the same
unbind/rebind on the PCI driver (`0000:00:14.0`) as 2026-07-22, without rebooting —
important because another lab member (niharika) had an active session + a GPU job
(`serve_policy.py`, ~10.8GB VRAM) that a reboot would have killed with no warning.
Root-caused which controller: both the keyboard/mouse's USB bus *and* the camera's are
children of the same PCI device, so the fix (and the fault) always affects both
together — the mouse dying mid-fix was this, not a separate problem, and self-recovered
once the bus finished re-enumerating.

Second, **even after that fix, the camera enumerated but wouldn't stream** (`control_transfer`
errors climbing, zero frames ever reaching `/camera/color/image_raw`). Root cause: USB
**autosuspend was on** (`power/control=auto`) for the camera. The repo already ships a
targeted fix for this exact symptom — `scripts/usb_hardening.sh` /
`config/udev/99-usb-hardening.rules` (from earlier upstream work titled "extend usb
autosuspend hardening to owc tb4 hub chain (suspended camera breaks stream start)") —
and it already lists this camera's exact USB ID (`8086:0b3a`) as a target. Running it
fixed the stream immediately (stable ~30Hz color + aligned-depth for the rest of the
session). Known gap, documented in the script itself and reproduced live: a hardware
reset/re-enumeration resets `power/control` back to `auto`, so it needs re-running
after every such event — happened twice more this session (each subsequent
replug/rebind) and was re-applied each time.

### Hover approach: same branch-jump danger as the 07-28 arc incident, just earlier in the pipeline

A **single seeded-IK jump** straight to a 30cm hover target converged positionally
(~1cm) but wanted a **75-85° joint-space jump** from the arm's tucked start pose — the
same "Cartesian-fine, joint-space-distant branch" danger documented for the door arc on
07-28, just this time on the approach phase, which nobody had stress-tested with a
large single jump before. Fixed the same way conceptually: a **chained/interpolated
path** (12-24 small steps generated via Slerp + per-step seeded IK from the previous
step), verified in sim (every step's joint delta and IK error checked against a
guard) before any real motion. Executed cleanly multiple times this session — sub-cm
tracking error throughout, once with 24 steps producing 0.0-0.1cm tracking error on
every single step.

### New bug found: grasp roll/orientation was never independently validated

The 07-28 `geometry_corrected_quat()` fix only ever corrects the approach *axis*
(shortest-arc rotation to point the gripper's local-Z at the real handle) — it
explicitly *preserves* whatever roll the original, still-uncorrected `GRASP_QUAT`
constant happened to have. A live visual check (via the recording) caught that the
gripper was oriented wrong for this microwave's actual handle geometry — **a vertical
chrome bar**, not the horizontal-bar geometry the original constant's roll was almost
certainly tuned for on a different rig/appliance. Needed an empirical **+180° roll**
correction, found in two live iterations (+90°, then another +90° after the first
still looked wrong), verified by eye against the video each time before continuing.
Root cause, as best understood: the shortest-arc axis correction inherits whatever roll
the old (wrong) constant had as an arbitrary byproduct of that rotation — nothing has
ever independently checked that roll against a specific handle's real orientation,
because a wrong roll doesn't break IK convergence the way a backwards axis does (silent
failure mode, not a hard one).

Full sequence executed as three separate continuous (non-stepwise) motions per
request — hover, grasp-approach, and gripper-close — each planned/verified in sim
first: hover landed within 0.06cm, grasp-approach within 0.4cm (then backed off ~5cm on
request, within 0.27cm of the new target), gripper closed on the handle (visually
confirmed) after a brief user Xbox-teleop adjustment in between (which, as documented,
faulted the Kortex session — recovered via the standard restart-`arm_server`-plus-
re-run-`bulldog_bypass` procedure, plus a stale-lock removal this time: the lock file's
PID belonged to a zombie/defunct process, so the usual liveness check via `os.kill(pid,
0)` didn't detect it as dead).

### Door-arc: wrong-direction incident, real e-stop, not root-caused this time

Repeated the 07-28 fix (direction=+1, radius-based arc sizing, seeded IK, joint-delta
guard) but computed the hinge as a relative offset from *today's* detected
handle/hinge poses, applied to the arm's *current* grasp point (which had shifted from
the manual backoff + Xbox adjustment above, not from the original detection pose). All
3 planned waypoints passed sim verification (max joint delta 16.1°, matching the
~12cm radius from 07-28). Commanded for real — swung the **wrong direction** on
camera. User stopped it (hard e-stop): confirmed via `BrokenPipeError` /
`Connection reset by peer` / `Network is unreachable` in `arm_server.log`, the same
signature as the 07-27/07-28 incidents. **Not root-caused before the session ended** —
leading candidate, untested: transferring the hinge *offset* (rather than an absolute
hinge position) from detection time onto a grasp point that had moved via a manual
Xbox nudge implicitly assumes the orientation at grasp time still matches the
orientation at detection time; if the teleop nudge changed orientation non-trivially,
the transferred offset vector no longer points at the true hinge. Worth checking first
next session, before touching the arc math again.

Recovered cleanly afterward: killed the stale `arm_server` PIDs, cleared the (again
zombie-caused) stale `/tmp/kinova.lock`, relaunched, re-ran `bulldog_bypass`, verified
`ARMSTATE_SERVOING_READY` before doing anything else.

### HLA investigation: why the real `OpenDoorHLA`/`PerceptionInterface`/`ArmInterfaceClient` don't just run here, checked empirically (no motion)

User asked directly why we've been calling low-level pieces from scratch scripts
instead of the real HLA, given Pachirisu has real `rospy` (unlike the Jetson). Checked
with actual imports/instantiation, not just re-reading docs:
- `rospy` and `tf2_ros` import fine on Pachirisu — confirmed. Genuinely better than the
  Jetson here.
- `netft_rdt_driver` (the wrist F/T sensor's ROS driver) genuinely does not exist —
  confirmed via a direct `import netft_rdt_driver` failure, not just documentation.
- `perception_interface.py` bundles `rospy`, `tf2_ros`, and `netft_rdt_driver` in one
  `try/except ModuleNotFoundError` — the missing `netft_rdt_driver` poisons the whole
  group. Confirmed live: `PerceptionInterface`'s module-level `ROSPY_IMPORTED` is
  `False` on this box, purely because of that bundling, even though `rospy`/`tf2_ros`
  individually work.
- `arm_client.py`'s own import block is narrower (doesn't bundle `netft_rdt_driver`),
  so its `ROSPY_IMPORTED` is `True`. But constructing `ArmInterfaceClient()` live, it
  prints `"Waiting for Watchdog status..."` then **hangs forever** on
  `rospy.wait_for_message("/watchdog_status", Bool)` — had to kill it after a timeout.
  The only thing that would ever publish that topic, `safety/watchdog.py`, has an
  *unguarded* top-level `from netft_rdt_driver.srv import String_cmd`, so it can't even
  be imported here.
- Encouraging counterpoint: `HighLevelAction.__init__` just **stores** whatever
  `robot_interface`/`perception_interface` objects it's handed — duck-typed, not
  hard-wired to the real classes. So `OpenDoorHLA` itself isn't blocked by any of the
  above; only the specific concrete `ArmInterfaceClient`/`PerceptionInterface`
  implementations are. A thin adapter matching their method surface but backed by the
  already-proven raw `ArmManager` + monkeypatched-transform `AppliancePerception` from
  this session's scripts is a genuinely open, buildable path into the real HLA — not
  yet attempted.

### End-of-session state

Arm moved by the user to a safe, ungripped position; microwave closed. Full clean
shutdown performed and verified: `bulldog_bypass` → `stub_base_server` →
`arm_server` (graceful SIGINT, no stale lock) → `roscore`, then the `feed-noetic`
container itself stopped (`docker stop`). Nothing left running. A git worktree of the
pristine pre-fork code (`a9e707bf`) was created for a planned raw-code comparison test
but never used (the door-arc incident interrupted that plan) — removed at end of
session.

### Not done / next session
- Root-cause the arc wrong-direction incident (see hypothesis above) before attempting
  the door-arc again.
- Actually attempt the thin-adapter approach into the real `OpenDoorHLA` (bypassing
  `ArmInterfaceClient`/`PerceptionInterface`'s `netft_rdt_driver` dependency chain via
  duck-typed substitutes), now that it's confirmed buildable rather than assumed
  blocked.
- The raw-code (pre-fork) comparison test was set up but never run; worth revisiting if
  a side-by-side comparison is still wanted.
- A written status summary combining this and prior sessions' findings (diff scope,
  hardcoded values, robustness assessment, fridge-task applicability) was saved to
  `MICROWAVE_FRIDGE_STATUS_SUMMARY.md` at the repo root.

---

## 2026-07-29 — Jetson: ROS 2 path runs on hardware; detection works, but the PERCEIVED HINGE IS ON THE WRONG SIDE

### Result

Rungs 0-3 of `docs/microwave_ros2_runbook.md` pass. **The arm was never
commanded** — everything below is read-only. The ROS 2 TF chain came up on
hardware for the first time and the repo's real `detect_handle_and_placement`
ran end-to-end through it.

### Rungs 0-2

- Arm ethernet up (`192.168.1.18/24`), pings `192.168.1.10`; RealSense D435i enumerated.
- The `arm_server.py` left running since Jul 14 was **wedged at 99.7% CPU** and its
  Kortex session was dead (`INVALID_USER_SESSION_ACCESS`). Restarted clean.
- **Gripper read 0.009 = open.** The arm was *not* still holding the door, contrary
  to the standing warning in `CLAUDE.md`. That warning can be retired.
- `/joint_states` 49.96 Hz, aligned depth ~14 Hz, and
  `arm_base_link -> camera_color_optical_frame` = `[0.255, 0.018, 0.565]`.
- Rung 3 was run **without** `bulldog_bypass.py`, so arm motion stayed locked.

### Two blockers found and fixed

1. `arm_client.py` had its own `assert ROSPY_IMPORTED` (missed in the port) — it
   now accepts ROS 1 or ROS 2, and skips the `/watchdog_status` wait, which does
   not exist when bulldog is bypassed.
2. `detect_items()` unpacked a fixed 6-tuple from `supervision`. **0.6.0 is
   installed here** (pinned by `groundingdino-py`) and yields 5; the lab's 0.21.x
   yields 6. Now indexes `.confidence`/`.class_id`, working on both — no install.
3. A **static-TF discovery race**: the listener had the dynamic tree from
   `robot_state_publisher` but not yet the latched `/tf_static`, so tf2 reported
   "two or more unconnected trees" and the immediate single retry also failed,
   killing the detection. `get_frame_to_frame_transform` now retries for up to
   `TF_LOOKUP_TIMEOUT_SEC` (10 s) instead of giving up at once.

### Detection result

`microwave handle` at **0.67** confidence (matches the 0.65-0.68 seen on
Pachirisu). Plane depth 0.558 m. Poses in `arm_base_link`:

| | x | y | z |
|---|---|---|---|
| handle | 0.7859 | -0.1358 | 0.4779 |
| hinge | 0.8602 | -0.2729 | 0.4593 |
| placement | 0.8182 | -0.1998 | 0.5215 |
| top_of_appliance | 0.7745 | -0.1294 | 0.3853 |

### ⚠️ The perceived hinge is on the WRONG SIDE — do not run the arc on it

```
perceived hinge - handle = [+0.074, -0.137, -0.019]
assumed (standalone)     = [ 0.000, +0.320,  0.000]
```

The sign of **y is flipped**: the plane-fit puts the hinge ~13.7 cm to **-y**,
the proven assumption puts it 32 cm to **+y**. Those are opposite edges of the
door. Its magnitude is also far short of a real door width (~32 cm), so the
"far edge of the fitted plane" heuristic (`np.max` anchor +2 cm strip) is not
finding the true hinge from this viewpoint — likely only part of the door plane
is visible.

Feeding that hinge to `_generate_door_arc_waypoints` with `direction=-1` would
sweep the door **the wrong way** — the exact failure that e-stopped rung 5 on
2026-07-14 ("arc swept inward toward base"). **Keep using the `+0.32 m`
assumption until the hinge heuristic is fixed and re-verified.**

### ⚠️ The depth bias has CHANGED — do not apply DEPTH_CORR=0.16 blindly

Against the 2026-07-14 teleop ground truth `[0.713, -0.099, 0.465]`:

```
delta  [+0.073, -0.037, +0.013]   |d| = 0.083 m
range  perceived 0.930 m  vs  truth 0.857 m   -> +7.3 cm, NOT +16 cm
```

The overestimate is now **~7 cm**, less than half the documented 16 cm. That is
expected in direction: the 16 cm was measured with the standalone calib chained
to `get_state()["ee_pos"]`, whereas this goes through **tf2** (URDF FK from
`robot_state_publisher` + the easy_handeye2 static transform) — a different FK
source with a different systematic error. Applying `HANDLE_DEPTH_CORR=0.16` on
this path would now **undershoot by ~9 cm**.

**Caveat, unresolved:** this comparison assumes the microwave is in the same
physical spot as on 2026-07-14. That was not confirmed. If it moved, part of the
8.3 cm delta is the appliance, not bias. **Re-measure against a fresh teleop
ground truth before trusting any correction constant on the tf2 path.**

### Fresh ground truth + two-distance check — the bias is a fixed OFFSET

Gripper hand-guided to touch the handle: true handle (EE origin) =
**`[0.6932, -0.1171, 0.5031]`**. Note the handle has **moved 4.7 cm** since
2026-07-14, so part of the delta against the old ground truth was the appliance,
not bias — always re-touch before trusting a correction.

Detected from two camera distances against that fixed truth:

| | camera->handle true | perceived | offset | scale |
|---|---|---|---|---|
| #1 | 0.4627 m | 0.5595 m | **+0.0968** | 1.2092 |
| #2 | 0.3258 m | 0.4176 m | **+0.0918** | 1.2818 |

Over a 13.7 cm change in distance the **offset moved 5 mm** while the **scale
moved 0.073**. So the error is a constant offset, not a proportional one:

> **`HANDLE_DEPTH_CORR = 0.094`, `HANDLE_LAT_CORR = 0.0`** on the tf2 path.
> The documented `0.16` belongs to the old calib+FK path and would undershoot
> by ~6 cm here. Lateral residual is <1 cm, nothing like the old `0.07`.

**Independent validation, worth noting:** the perceived handle came out at
`[0.7859, -0.1358, 0.4779]` and `[0.7826, -0.1280, 0.4779]` from two very
different arm poses — **8 mm apart**. The TF chain (joint bridge +
robot_state_publisher + easy_handeye2) is therefore self-consistent across
viewpoints; the 9.4 cm is a systematic offset, not calibration noise or jitter.

**Unresolved ambiguity.** A two-distance test separates offset from scale but
*cannot* separate a genuine perception offset from a constant EE-frame
convention offset (tool tip vs tool flange) — both are distance-invariant.
`kinova.py:420` says `get_state` returns the **tool tip** pose, which favours
this being real perception error (the handle centroid landing at/near the door
plane rather than on the protruding handle), but the same comment says it "isn't
exactly the same as" the tool frame, leaving ~cm of doubt. Resolve by measuring
the handle's protrusion from the door with a ruler and comparing to 9.4 cm.

**This also kills the scale hypothesis for Pachirisu's ~20 cm error** — if the
error model is a fixed offset, sitting further back does not inflate it, so that
box's problem is something else (still most likely its calibration reference
frame, per 2026-07-22).

### Correction verified live: 1.6 cm

With `HANDLE_DEPTH_CORR=0.094`, `HANDLE_LAT_CORR=0.0`:

```
corrected handle [0.6902, -0.1314, 0.4965]
truth            [0.6932, -0.1171, 0.5031]
error            [-0.0030, -0.0143, -0.0066]   |e| = 1.6 cm    PASS
```

Matches the ~1–2 cm the proven standalone pipeline achieved. The residual is
mostly lateral (1.4 cm); a small `HANDLE_LAT_CORR` could shave it, but one sample
does not justify tuning — leave at 0.0 until measured twice.

### A `/tf_static` failure worth knowing about

Mid-session a detection died with "two or more unconnected trees" even though the
calibration publisher was alive and `tf2_echo` had worked minutes earlier.
`/tf_static` carried only `robot_state_publisher`'s transforms; ours were absent.
Latched (TRANSIENT_LOCAL) `/tf_static` is **not reliable with several independent
publishers** — a late-joining listener can receive some publishers' transforms and
not others.

Fix: `calibration_tf.py` now also re-sends the calibration on `/tf` at 20 Hz with
fresh stamps. That guarantees delivery regardless of join order, and the fresh
stamps also cut the "extrapolation into the past" fallbacks. After the fix the
tree resolved immediately and detection ran clean.

### Hinge error reproduced from a second viewpoint

```
pose #1  hinge - handle = [+0.074, -0.137, -0.019]
pose #2  hinge - handle = [+0.139, -0.113, -0.033]
assumption              = [ 0.000, +0.320,  0.000]
```

**y is negative at both poses** while the working assumption is +0.32. Not a
one-off bad frame — the plane-fit far-edge heuristic genuinely picks the wrong
edge on this microwave/viewpoint. Do not drive the arc from the perceived hinge.

### ✅ First autonomous approach through the ROS 2 path — 1 mm

Approach-only (no grasp, no arc), `set_speed("low")`, gates on:

```
handle (corrected)  [0.6922, -0.1307, 0.4970]   1.5 cm from touched truth
pre-grasp target    [0.5722, -0.1307, 0.4970]   12.0 cm standoff, range 0.77 m
final EE            [0.5724, -0.1309, 0.4959]
tracking error      [+0.0002, -0.0002, -0.0012]   |e| = 0.1 cm
gripper             0.0087 (untouched)
```

Full chain end to end: ROS 2 TF → live GroundingDINO → corrected handle →
repo pre-grasp geometry → seeded IK → one joint move → **1 mm**.

### ⚠️ `GRASP_QUAT` is 180° off about y on this rig

The blocker before that move was **not** the offsets — it was the orientation.
`detect_handle_and_placement` stamps every pose with a fixed
`GRASP_QUAT = (-0.5, 0.5, 0.5, -0.5)`, which in this base frame maps **local +z
to base −x**: the gripper faces *backward, at the robot*. With that orientation
the repo's `pre_grasp = handle @ trans(0,0,-0.12)` lands **behind the microwave**
at range 0.96 m — past the Gen3's ~0.90 m reach, and physically unreachable
(the appliance is in the way).

Frame convention here, for the avoidance of doubt: **+x is out/forward, +z is up.**

Flipping the offset sign is the wrong fix — it puts the EE in front of the handle
but still facing away, an awkward wrist pose (IK error 15.8 cm, 169° flip). IK
sweep over candidates:

| orientation / offset | gripper +z | range | IK err | joint jump |
|---|---|---|---|---|
| repo quat, −0.12 | −x (backward) | 0.961 | 39.0 cm | 172° |
| repo quat, +0.12 | −x (backward) | 0.769 | 15.8 cm | 169° |
| **quat·Ry(180), −0.12** | **+x (forward)** | **0.769** | **0.00 cm** | **43°** |

Rotating 180° about local y turns the gripper to face the appliance, and then the
repo's **own** `−0.12` offset puts the standoff in front of the handle. Only
candidate that solves exactly, and it needs the least joint motion.

**Open question:** whether the lab's `arm_base_link` is oriented differently (making
the constant correct there) or whether this is a latent bug upstream. Until that is
settled, treat the 180° y-rotation as **rig-specific** and do not change the shared
constant.

### Two smaller things

- `JointCommand` takes `pos=`, not `joint_positions=`. The wrong keyword raised a
  `TypeError` *before* anything was commanded — a safe failure, but worth knowing.
- The sim scene puts the robot at a world offset, so an `arm_base_link` target must
  be lifted via `multiply_poses(scene.robot_base_pose, pose)` before PyBullet IK.
  Skipping that gives a 227 cm IK error — caught by the IK gate.

### Measured grasp pose — `GRIP_EXT` is ~0 on this path, NOT 0.065

The 12 cm pre-grasp standoff was too far to grasp from, so the arm was driven in
by Xbox teleop to where the gripper should actually close, and the pose read back:

```
grasp EE (teleop)   [0.6965, -0.1098, 0.4952]   gripper +z = [1.00, -0.00, 0.01]
corrected handle    [0.6922, -0.1307, 0.4970]
EE - handle (base)  [+0.0043, +0.0209, -0.0018]   |d| = 2.1 cm
  in handle-local:  along approach (z) +0.4 cm ; lateral (x) +2.1 cm
```

**Along the approach axis the offset is ~0.4 cm — effectively zero.** The
inherited `GRIP_EXT = 0.065` does *not* transfer to this path.

The reason is that the two constants are **coupled**: `HANDLE_DEPTH_CORR = 0.094`
was fitted so the corrected handle coincides with the EE pose when the gripper is
touching the handle. So the corrected handle *is already* the grasp point, and
applying `GRIP_EXT` on top would back the gripper off by a further 6.5 cm and
miss. Use one or the other, never both.

> **Rule:** with `HANDLE_DEPTH_CORR` calibrated against a touched EE pose,
> **grasp EE target = corrected handle**, `GRIP_EXT = 0`. If the depth correction
> is ever re-derived against the handle *surface* instead of the touched EE pose,
> `GRIP_EXT` has to come back.

**A repeatable ~2 cm lateral bias.** The perceived handle sits ~2 cm to base −y of
the grasp point, and the same direction showed up in the correction check
(1.4 cm). Two samples agree in sign, so `HANDLE_LAT_CORR` is worth deriving —
but from a third measurement, not by fitting these two.

### ✅ Full perception-driven grasp over ROS 2 — detect → pre-grasp → grasp → close

```
detect (39 cm view)  handle [0.6920, -0.1070, 0.4924]   1.5 cm from touched truth
pre-grasp            target [0.5720, -0.1070, 0.4924]   tracking 0.5 cm
grasp                target [0.6920, -0.1070, 0.4924]   tracking 0.7 cm
close                gripper 1.0000  -> confirmed GRIPPING the handle (visual)
```

The whole chain runs through the ported ROS 2 path: TF → live GroundingDINO →
corrected handle → repo pre-grasp geometry (with the rig's 180° wrist fix) →
seeded IK → two joint moves → grip.

**Detect ONCE, from a viewing distance.** Re-detecting at ~2 cm returned a handle
6.7 cm off (mostly +6.6 cm in z) — the wrist camera cannot see the microwave from
there and the plane fit is meaningless. The sequence caches the poses from a
single look at ~39 cm, exactly as the HLA does.

### ⚠️ `gripper_pos` does NOT tell you whether the grasp succeeded

`gripper_pos` read **1.0000** after closing on the handle, and a first pass wrongly
called that "closed empty". It is not: `LOCAL_DEPLOYMENT.md` documents 0.009 open ↔
1.0 closed, and `CLAUDE.md` records **~0.99 while holding the door**. The value
saturates at ~1.0 whether or not something thin is between the fingers, so it
**cannot** discriminate a successful grasp.

> Do not gate anything on `gripper_pos` alone. The proven 2026-07-14 flow already
> did the right thing: `close_gripper` → **pause for a human grip check** before
> opening. Keep that. A machine check would need gripper current/effort, which is
> not currently exposed.

### The lateral "bias" was detection variance, not bias

Three detections of the same fixed handle gave y = −0.1307, −0.1066, −0.1070
against a teleoped grasp y of −0.1098 — a ~2.4 cm spread with no consistent sign.
The 2.1 cm "lateral offset" measured from a single teleop comparison was noise;
applying it as a fixed correction overshot. **`LATERAL = 0`.** The 2F-85's 85 mm
opening absorbs the remaining variance. Depth is a genuine fixed offset (verified
at two distances); lateral is not — do not treat them the same way.

Also: the reach gate at 0.85 m was over-tight and blocked a feasible grasp (IK
solved exactly). The arm reaches 0.858 m under teleop; gate raised to 0.88 m
against a ~0.90 m spec.

### ✅ FULL AUTONOMOUS MICROWAVE DOOR OPEN over ROS 2 — ~91°

Detect → grasp → arc, all through the ported ROS 2 path. 10 of 11 waypoints ran,
then a clean stop at the door's limit:

```
grasp pose   [0.6484, -0.1984, 0.4846]     hinge (assumed) [0.648, 0.122, 0.485]
11 waypoints, arc 0.55 m, spacing 0.05, direction -1, seeded IK (all 0.00-0.07 cm)
wp01..wp10   tracking 0.1 - 0.6 cm  (gripper 0.991 throughout)
wp11         no motion, 4.7 cm -> natural stop, held
swept 45.6 cm chord from the grasp pose
```

Waypoints come from the repo's **own** `_generate_door_arc_waypoints`
(`perception_interface.py:424`), invoked unbound with the microwave parameters
`perceive_handle_opening_poses` uses.

**The `+0.32 m` hinge assumption is now validated, not just inherited.** Forcing
the handle along a circular path about an assumed centre is a strong test: if the
centre were wrong, the door would bind and the radius would not hold. Measured:

| | |
|---|---|
| radius at grasp | 0.3204 m |
| radius at finish | 0.3241 m |
| **radius preserved** | **3.7 mm over a 90.8° sweep** |
| z drift | −1.6 mm (hinge axis vertical, as assumed) |
| swing angle | **90.8°** |

So the assumed hinge is within a few mm of the true one, tracking stayed
sub-centimetre throughout, and the stop at wp11 is the **door's own limit** — not
a bind. That also retro-answers the earlier open question about the 2026-07-29
first attempt (stopped at wp09, 37.9 cm): the geometry was fine there too, the arm
had simply been positioned such that the last waypoints were unreachable.

**The perceived hinge remains unusable** — it was wrong-side at two viewpoints.
The working configuration is the repo's arc algorithm with the assumed hinge.

### The appliance drifts, and that is why absolute references fail

y across successive detections: −0.131 → −0.107 → −0.160 → −0.179 → −0.206, with x
creeping outward too. Within-session detection pairs agree to **0.0–0.6 cm**, so
this is not perception noise — **the microwave is nudged a little by every door
swing.** Consequences, both now handled:

- A touched ground-truth pose goes stale immediately; gating detection against it
  rejects good detections (a 7.2 cm refusal). Replaced with a **two-detection
  self-consistency** check plus a loose plausibility box — no ground truth needed.
- The appliance drifted far enough out (grasp range 0.884 m) to exceed the reach
  gate, and had to be slid back toward the arm. Expect to re-seat it periodically.

### Next step

Release and retract without loading the door: **release first**, then clear
laterally, then back out — never withdraw while gripping. Then the remaining gap
to running the HLA unmodified: the hinge heuristic, and the repo's
`move_to_ee_pose_trajectory` cartesian path (this used per-waypoint joint moves,
since cartesian aborts at extended configs on this arm).

---

## 2026-07-28 — Pachirisu: hover approach + real grasp validated end-to-end, door-arc model found wrong twice (direction + non-vertical hinge axis), one real motion incident

Follow-on from 2026-07-27's unresolved IK convergence. Goal: resolve it and push toward a
full detect->grasp->open attempt. Got much further than expected -- hover approach and
grasp are now proven on real hardware -- but the door-opening arc surfaced two real
modeling bugs and one real safety incident.

### IK convergence root cause: the fixed grasp quaternion's approach axis was pointing backwards

Diagnosed the 2026-07-27 unresolved IK convergence with software-only checks before
touching real detection: position-only IK converges instantly from any seed; 150+ wide
random-restart full-joint-space seeds ALL plateau at the same ~17cm residual regardless
of seed (the signature of a truly unreachable pose, not a bad seed); a full 360deg roll
sweep about the approach axis made zero difference. Root cause: the repo's fixed
grasp-orientation constant (`(0.5,-0.5,-0.5,0.5)` / `GRASP_QUAT=(-0.5,0.5,0.5,-0.5)`, same
rotation) has its local-Z (approach) axis pointing exactly opposite the real look-at
vector to the handle -- a 180deg inversion, not a minor offset. Re-pointing the axis at
the true look-at vector (shortest-arc rotation, preserving the original roll) took IK
from ~17cm error to 0.01cm, fully within joint limits, from every seed tried.

Also found and fixed a real bug in that fix's own first draft: the exactly-antiparallel
special case (exactly what happens whenever the look-at direction is a pure world axis,
e.g. a pure -x back-off) picked an arbitrary fallback rotation axis that could coincide
with the vector being flipped, silently making the "fix" a no-op. Rewrote as a single
correct `geometry_corrected_quat()` helper in `scripts/scratch/debug_ik_convergence2.py`.

### Real detection, twice, on the actual Comfee microwave

Brought up roscore + `realsense2_camera` (`align_depth:=true`) and `arm_server.py`
(read-only) fresh this session. Started from the arm in a fully-retracted/tucked pose
(`ee_pos~(0.15,0.04,0.19)`) with the wrist camera pointed at the floor -- confirmed
visually before trusting any detection. User manually repositioned the arm to face the
microwave (twice -- once initially, once after moving the microwave further away);
`detect_handle_and_placement("microwave handle", ...)` fired both times at 0.41-0.49
confidence, `handle_pixels.png` visually confirmed landing squarely on the real chrome
handle bar both times.

Also fixed a real, unguarded crash in the repo itself: `appliance_perception.py`'s DBSCAN
clustering call had no empty-list check before `.fit(handle_points)`, unlike the
identical pattern one block earlier in the same function
(`if len(bounding_box_points_3d) == 0`) -- added the matching guard (now returns
`None, None, None, None` gracefully instead of crashing sklearn's `check_array`).

### Hover approach validated live, twice (30cm and 15cm back-off)

Recomputed the hover target from the real detected handle position using the
geometry-corrected orientation, verified via a full seeded chain (from the arm's actual
current joints, not an arbitrary seed) that tracking held within joint limits the whole
way, then commanded it for real:
- 30cm back-off: landed within **0.7mm** of target.
- 15cm back-off (closer, per user request): landed within **3.7mm**.

Both moves used `set_joint_position` (not `set_ee_pose`, consistent with 2026-07-27's
finding that cartesian moves can jerk unexpectedly) after computing joints via
`p.calculateInverseKinematics`, seeded from the real current joints.

User then manually moved the gripper the rest of the way to touch the handle and asked
for the offset to be noted (empirically ~7cm forward + ~1.6cm lateral from the computed
hover point, though user asked to record it as ~12-13cm per their own estimate -- noted
as instructed, not reconciled against the computed number).

### Grasp succeeded; one transient, unexplained arm-state anomaly

`close_gripper()` from the user-positioned touch point closed successfully (visually
confirmed) -- but the arm's `get_arm_state()` briefly reported
`ARMSTATE_SERVOING_MANUALLY_CONTROLLED` right after, even though the user was not
touching the arm. `arm_server.log` showed a `REACH_JOINT_ANGLES` action hit
`ACTION_ABORT` (`abort_details: ROBOT_MOVEMENT_IN_PROGRESS`) at that moment -- gripper
actuation is implemented as a joint-angle action on this Kortex firmware, and it
apparently collided with residual settling from the prior move. State reverted to
`ARMSTATE_SERVOING_READY` on its own within a few seconds, no fault latched. Not fully
root-caused; flagging as a watch-item (echoes 2026-07-21's unresolved `METHOD_FAILED`
abort oddity -- another Kortex-session quirk on this rig that resolved itself without
full explanation).

### Door-opening arc: two real modeling bugs found, one real safety incident

Reused the Jetson's exact proven one-waypoint-per-call pattern
(`scripts/real_gen3_open_microwave.py`, state persisted, per-step tracking-error abort)
via a new `scripts/scratch/run_door_arc_step.py`, but two things about this specific
rig/microwave broke the ported defaults:

1. **Radius mismatch.** The repo's own `detect_handle_and_placement` supplies a real
   hinge position; on this microwave it put the handle only ~12cm from the hinge (vs.
   the Jetson's blind `DOOR_W=0.32m` guess). Reusing the Jetson's `arc_length_m=0.55`
   constant at this radius demands ~262deg of rotation -- immediately exceeds the ~90cm
   reach guard by waypoint 2. Fixed by sizing `arc_length_m = radius * target_angle_rad`
   for an explicit target angle (started with a conservative 25deg test swing, not a
   full open) instead of reusing an absolute-length constant tuned for a different rig's
   geometry.
2. **Direction sign.** The repo's own default for `handle_type=="microwave"` is
   `direction=-1`. At this rig's real hinge geometry, `-1` swings the handle AWAY from
   the base (immediately over the reach guard); user correctly reasoned that opening a
   door by pulling the handle should bring it TOWARD the arm, not away. Flipping to
   `direction=1` both matched that physical intuition AND kept every waypoint
   comfortably under the reach guard (87-89cm vs. 92-95cm) -- confirming the Jetson's
   `-1` default was tuned for that rig's specific (mirrored) microwave-to-arm layout, not
   a universal constant.
3. **Real safety incident.** Step 1 (with the two fixes above) was commanded and
   produced a violent, unpredictable ~150deg swing on J1/J3/J7 even though the Cartesian
   target was only ~1cm off (ik_err 0.01cm) -- the grip held (lucky), no fault latched,
   but this was flagged by the user as dangerous. Root cause:
   `_generate_door_arc_waypoints`'s IK (following the Jetson script's own documented
   "deliberately UNSEEDED" pattern) was solved in a **freshly-created simulator with a
   default/arbitrary initial joint state**, not seeded from the arm's actual current
   posture -- found a Cartesian-correct but joint-space-distant alternate solution
   branch (elbow/wrist flip). `set_joint_position` has no path planning, so the real arm
   swept through that reconfiguration directly. **The Jetson script's "unseeded is fine
   for small arc waypoints" note does not hold in general** -- it depended on the fresh
   sim's default init happening to already be close to that rig's working posture; it is
   not a safe pattern to reuse blindly on a different rig/starting configuration.
   - Fix (implemented in `run_door_arc_step.py`, verified in sim before any further real
     motion): seed IK from `ai.get_state()["position"]` (the actual current real
     joints), and explicitly guard on **joint-space delta** (`>25deg` aborts), not just
     Cartesian `ik_err`. Verified step 2 (18deg max delta) and step 3 (9.7deg) both
     converge cleanly under this guard before re-attempting.
   - Step 2 was then run for real with the fix: reached within 1cm, no wild swing this
     time (user: "kinda moved unpredictably, but not really badly" -- worth another look
     next session, 18deg on J6 in a single step is still non-trivial and this posture may
     be kinematically sensitive/near a singularity).
4. **Arc geometry model also wrong: assumes a purely vertical hinge axis, but this door
   isn't a simple pivot.** After step 2, the user manually walked the gripper (still
   holding the handle) to where they judged the true "waypoint 2" should be, for
   comparison: real point `(0.7166, -0.0725, 0.5239)` vs. the computed step-2 target
   `(0.728,-0.177,0.466)` -- off by **+10.5cm in y and +5.8cm in z**. The z discrepancy is
   the important one: `_generate_door_arc_waypoints` assumes the door rotates about a
   purely vertical (z) axis (constant height throughout), but the real door's height
   clearly changes as it opens -- this microwave's door mechanism is not a simple single
   vertical-axis pivot (likely a multi-bar/lift-type hinge). This is a second,
   independent reason not to trust the formula-driven waypoints beyond a single verified
   step at a time.

### End-of-session state

Gripper is holding the microwave handle at the user's manually-demonstrated "waypoint 2"
pose `(0.7166, -0.0725, 0.5239)`, door partially open. `arm_server.py` +
`stub_base_server.py` + `bulldog_bypass.py` all still running (motion unlocked) at
session end -- pick up from here or restart the bring-up per the "Setup" section above.
No fault latched, `ARMSTATE_SERVOING_READY` as of the last read.

### Not done / next session

- Door-arc kinematics need a real model of this microwave's actual hinge mechanism (not
  a pure vertical-axis circle) -- either have the user demonstrate several more real
  waypoints by hand and fit/interpolate from those directly (working well so far), or
  investigate the physical hinge to build a correct parametric model.
- The J6-heavy, 18deg-in-one-step posture around step 2 may be kinematically sensitive
  (near a singularity) -- worth a closer look before trusting further automated steps
  through this region.
- Empirical grasp-approach offset for this rig noted as ~12-13cm (per user) -- not yet
  reconciled with the ~7cm/1.6cm delta actually measured between the computed hover
  point and the user's touch point.
- Continue the door-opening test swing from the current real waypoint-2 pose, or
  re-plan from scratch with a corrected (non-vertical-axis) arc model.

---

## 2026-07-28 — Jetson: microwave path ported from ROS 1 to ROS 2 (no hardware run)

### What changed and why

Previous Jetson sessions ran a **rospy-free** pipeline *alongside* the repo:
`pyrealsense2` for frames, the easy_handeye2 calib chained with live FK instead
of tf2, and standalone scripts instead of the HLAs. That proved the geometry on
hardware but meant the real `open_microwave()` skill never actually ran.

This box **does** have ROS 2 Humble, with `rclpy`, `tf2_ros`, `sensor_msgs`,
`cv_bridge` and `realsense2_camera` all importable from the project venv
(Python 3.10.12, matching Humble). So the perception path was **ported to ROS 2**
rather than bypassed, and both `OpenDoorHLA` and `PressMicrowaveButtonHLA` now
import and run unmodified.

### The port is boundary-only

Only two message spellings and the client library actually differ:

| ROS 1 | ROS 2 |
|---|---|
| `CameraInfo.K/D/R/P` | `CameraInfo.k/d/r/p` |
| `stamp.secs/.nsecs` | `stamp.sec/.nanosec` |
| implicit global node | explicit `Node` + an executor something must spin |

So `CameraInfoCompat` wraps a live ROS 2 message and re-exposes the ROS 1 names,
and `ros2/node.py` provides the process-wide node rospy implied. **No detector
math was rewritten** — `detect_handle_and_placement`, `detect_items`,
`pixel2World` and `perceive_handle_opening_poses` are byte-identical to upstream.

### The missing piece was joint states

tf2 can only answer `arm_base_link → camera_color_optical_frame` if something
publishes the arm's joints. The lab gets that from its Kinova ROS driver; this
rig drives the arm over the repo's own RPC and has **no ROS arm driver at all**
(deliberately — the ROS 2 workspace on this box belongs to other projects and is
off-limits). `ros2/joint_state_bridge.py` closes the gap by republishing
`get_state()` as `sensor_msgs/JointState`; `robot_state_publisher` does the rest.

`kortex_description`'s `gen3.xacro dof:=7 gripper:=robotiq_2f_85` turned out to
emit exactly `base_link` / `end_effector_link` — the frame names the saved
calibration already refers to, so nothing had to be renamed. Verified by
generating the URDF with `xacro`.

### Button detection no longer needs Molmo

`detect_start_button` reached the button only via a remote Molmo VLM behind an
ngrok tunnel on the lab network — unreachable here, so the button skill could
not run at all. Added a local GroundingDINO backend (the model is already loaded
for handle detection) applying the rule Molmo's own prompt states: the **right**
one of the two rectangular buttons on the **bottom** row of the control panel.

**The trap is orientation.** The wrist camera is mounted upside down, so
"bottom" and "right" mean the flipped (visually upright) view, while
`detect_items` hands back raw original-image coordinates. Getting that backwards
picks the diagonally opposite button and looks entirely plausible in a log.
`tests/test_button_detection.py` pins it down (5 tests, models stubbed).

`BUTTON_BACKEND=auto` (default) tries Molmo first, so **lab behaviour is
unchanged**; `grounding_dino` forces local, `molmo` forces the original.

### What was verified — and what was not

Verified, no hardware:
- `OpenDoorHLA` and `PressMicrowaveButtonHLA` import with `ROS2_IMPORTED = True`.
- The genuine `perceive_handle_opening_poses("microwave")` runs offline with the
  detector stubbed and produces all 21 pose entries (11 opening / 6 push / 10
  closing / 10 offset-closing / 7 pull-closing / 13 push-closing waypoints) —
  `scripts/generate_microwave_poses_offline.py`.
- The real behaviour tree executes through staging → `settle_camera` →
  perception → the RViz `None` guard → the second staging move.
- Button selection, orientation flip, whole-panel rejection, empty case.
- Full suite: **165 passed, 1 failed** — the failure
  (`test_outside_mouth_distance_near`, expects 0.07, YAML says 0.05) is
  **pre-existing on `origin/microwave-task`**, from the lab's "lower
  outside-mouth transfer distances" commit; confirmed by running it on a clean
  checkout of origin. Unrelated to the microwave task.
- The launch file builds under system python3.

**Not verified — no hardware run this session.** The arm was not commanded at
all. In particular: the live TF chain has never been brought up, so
`tf2_echo arm_base_link camera_color_optical_frame` is the **first thing to
check** next session, before any motion.

`validate_open_microwave_hla_sim.py` stops at the first Cartesian move with
`Sim cartesian controller: Failed to reach target pose in time`. That is the
**known IKFast wall on this box**, not a regression: a trivial 2 cm move from
the arm's current pose fails identically. Sim motion validation stays
unavailable here until IKFast or the sim cartesian controller is fixed.

### Deliberately not used

The ROS 2 workspace on this box (`~/ros2_ws`, Demo-Software) contains
`cmu_door_opener` (YOLO-seg button detection + force-monitored push),
`arm_driver` and `cornell_feeding`. These belong to other projects on the shared
Jetson and were **not** built on, at the user's direction. Only distro-level
ROS 2 (`rclpy`, `tf2_ros`, `realsense2_camera`, `kortex_description`) and the
user's own easy_handeye2 calibration are used.

### Next step

Bring the TF chain up and confirm it (`tf2_echo`) before anything moves. Then a
detection-only run through the ported path — compare the handle pose it reports
against the rospy-free pipeline's known-good numbers, keeping the documented
`DEPTH_CORR=0.16` / `LAT_CORR=0.07` biases in mind, since those corrections live
in the standalone scripts and are **not** yet folded into the HLA path.

---

## 2026-07-27 — Pachirisu: recalibration root-caused and fixed (5.3cm), approach-hover attempt hit a real motion incident + unresolved IK convergence

Follow-on session, continuing from the ~20-27cm calibration error found 2026-07-22. Goal:
recalibrate, validate, then attempt the deferred approach-only (no grasp) hover test.
Recalibration succeeded and is now validated to ~5cm; the hover test did not complete --
a real-arm incident (unexpected jerky motion from a cartesian move) and then an
unconverged IK solve stopped it short.

### Recalibration: the real bug was TSAI instability, not (primarily) pose diversity or frame mismatch

Before recapturing anything, the old calibration's raw sample dump
(`~/deployment_ws/pachirisu_wrist_camera_calib/handeye_raw_samples.json`, still on disk
even though the script that produced it is gone) was inspected directly: the 17 samples
had a max pairwise gripper-translation distance of only **5.0cm** -- effectively a
rotate-in-place pose set, which starves `cv2.calibrateHandEye()`'s translation solve.

Recaptured with a new one-shot capture script (`scripts/scratch/capture_calib_sample.py`,
non-interactive -- appends one sample per invocation so it can be driven by chat command
while the user moves the arm by hand; an earlier interactive/`input()`-loop version was
rejected for exactly this reason). Board geometry (12-marker ArUco board, `DICT_5X5_250`,
marker 0 as reference, 5.08cm markers) reused as-is from the old JSON. Each sample reads
`arm.get_state()['ee_pos']` directly at capture time (ruling out any reference-frame
mismatch in a separately-derived FK, the leading theory from last session).

Live camera preview: this container's `cv2` build is headless (no `imshow`) and no ROS
image-viewer package (`image_view`/`rqt_image_view`) is installed in `ros_env`
(`ros-base`, not `desktop-full`). Built a tiny MJPEG HTTP server instead
(`scripts/scratch/mjpeg_preview.py`, serves `/camera/color/image_raw` on
`:8095`) and opened it in a host browser via `DISPLAY=:3 xdg-open` -- simplest live view
without adding GUI packages to the container.

First 9-sample set (max translation 57.2cm, up from 5.0cm) made things **worse**, not
better: TSAI gave a `t_cam2gripper` norm of 57.8cm (vs the old calibration's 20.4cm) and
self-consistency spread of 15-22cm (vs the old set's 2-6cm). Comparing all 5 of OpenCV's
hand-eye methods on the same samples isolated the actual cause: PARK/HORAUD/ANDREFF/
DANIILIDIS all agreed closely (6.6-8.4cm norm, near the user's measured ~5cm ), while
**TSAI alone was the outlier** -- both this time and, very likely, in the original
20.4cm result. Filtering to only full-12-marker-visible samples (partial-marker frames
add real solvePnP noise) and growing the set to 11 such samples brought even TSAI into
much closer agreement with the others (8.5cm vs 6.6-8.4cm). Saved with **PARK**
(agrees tightly with HORAUD/DANIILIDIS) as
`~/deployment_ws/pachirisu_wrist_camera_calib/wrist_camera_calib_v2.json`.

Self-consistency spread is still 20-45cm across samples even after these fixes -- not
fully resolved, method-agreement was the trust signal used to proceed, not a clean
consistency number.

### Validation: manual-touch error closed from ~20-27cm to 5.3cm

Re-ran `detect_handle_and_placement` (via `scripts/scratch/test_detect_handle_and_placement.py`,
now with `CALIB_PATH` overridable by env var) against the new calibration --
`microwave handle` fired at 0.57-0.61 confidence, handle pose repeatable to ~3.3cm across
two detections minutes apart, consistent with prior sessions. User touched the arm to the
physical handle; comparing `get_state()['ee_pos']` to the computed handle pose:

| | pos | 
|---|---|
| touch (wrist `ee_pos`) | (0.634, -0.125, 0.478) |
| computed handle | (0.683 / 0.716 across 2 detections, -0.116/-0.123, 0.459) |
| delta | ~5.3cm |

5.3cm is in the same range as the known, separate gripper-fingertip-to-wrist offset
(~5-6cm, previously identified and ruled out as *the sole* cause of the old ~20cm error) --
so this residual is plausibly just that offset, not leftover calibration error. GPU was
unavailable for detection this session (see below), so this ran on CPU
(`CUDA_VISIBLE_DEVICES=""`, same fallback the Jetson rig uses) -- still fast enough for a
one-off detection, no real slowdown concern for this kind of test.

### GPU was full from another user's process -- not touched

`GroundedSAM()` init hit `CUDA_AcceleratorError: out of memory` -- `nvidia-smi` showed
PID (unrelated to this session) holding ~10.8GB/12GB: a `serve_policy.py` (pi05 LoRA
checkpoint) process belonging to another lab member, running since the day before. Did
**not** kill another user's process on a shared machine without checking first; worked
around by running detection on CPU instead (see above).

### Approach-only hover attempt: real motion incident, then unresolved IK convergence

Brought up `stub_base_server.py` + `bulldog_bypass.py` (motion unlocked), `set_speed("low")`.

First hover-target math needed two rounds of correction from the user before commanding
anything (good catches, no motion sent during either): the repo's own pre-grasp offset
convention (`handle_transform @ [0,0,-0.12]`, from `perception_interface.py`) produced a
target *further* from the arm base than the handle itself, along an axis that -- once
sanity-checked against "arm base ~ origin, handle mostly out along +x" -- didn't hold up
as an obviously-safe direction to trust blindly for a first real move on this rig. Revised
to the simplest defensible version: back off 12cm along **world -x only**, holding y/z
(height) fixed at the handle's values. Final target: `(0.596, -0.123, 0.459)`, same
orientation as the detected handle `(0.5, -0.5, -0.5, 0.5)` xyzw.

Commanded via `arm.set_ee_pose()` (Kortex cartesian). **This did not behave like the
documented "aborts, no motion" failure mode** -- `set_ee_pose()` returned `False` and a
same-second `get_state()` read looked unchanged, but the user observed real, jerky
motion, and the `arm_server.py` log confirmed it: a `JOINT_ACCELERATION_LIMIT_REACHED`
trajectory-info notification mid-move, i.e. a real, partially-executed, aborted
trajectory -- not a clean instant no-op. User moved the arm back by hand.
**`LOCAL_DEPLOYMENT.md`'s "aborts... no motion" note is not reliable enough on its own
to trust for a real move of this size on this rig** -- needs updating / more caution
next time, not just cited as fact.

`get_arm_state()` afterward read `ARMSTATE_SERVOING_READY` (no fault latched) with
`velocity` all zero -- arm was fine, just displaced from where the software last thought
it was.

Switched to the Jetson-proven seeded-IK + joint-space pattern (`solve_ik_seeded`/
`move_and_check` from `scripts/real_gen3_detect_grasp_microwave.py`, reimplemented
standalone as `scripts/scratch/approach_hover_seeded_ik.py`, compute-only / no auto-command).
Two things checked, in order:
1. **Frame composition sanity check** (new, not in the Jetson original): FK the sim from
   the arm's *actual current* joints and compare to the real `ee_pos`, with and without
   composing `sd.robot_base_pose`. Confirmed consistent: `sim_ee_world == real_ee_pos +
   robot_base_pose.position` (that offset, `(1.0, 3.0, 0.54)`, is just this `vention.yaml`
   scene's fixed placement for a full mobile-base/kitchen sim, not a bug) -- so the
   frame math itself is trustworthy.
2. **IK solve itself did not converge**: 18.5cm `ik_err` for the hover target from the
   current seed, with one joint solution component (~-5.95 rad) clearly outside that
   joint's real hardware range -- a genuine solver/reachability failure from this specific
   starting posture, not a frame bug. Correctly aborted (no motion commanded) per the
   `ik_err > 2cm` guard.

Session stopped here at the user's call (one real incident already; software-only IK
debugging can safely continue without the arm powered/unlocked). Bulldog bypass killed
cleanly (`SIGINT`, confirmed heartbeat-loss message + arm still `ARMSTATE_SERVOING_READY`,
no fault) to relock motion before ending the session.

### Not done / next session

- **IK convergence for the hover target is still unresolved.** Try a different current
  arm seed posture (small manual reposition before attempting the big jump), explicit
  joint-limit arrays passed to `calculateInverseKinematics`, or a rest-pose bias --
  all software-only, no hardware risk to iterate on.
- The approach-only hover test itself (no grasp) is still not done -- blocked on the above.
- Treat `arm.set_ee_pose()` as **not** a safe default for large moves on this rig until
  proven otherwise -- prefer seeded-IK + `set_joint_position` (once IK converges), same
  as the Jetson rig's proven pattern.
- GPU contention with other users' long-running jobs on this shared machine is now a
  known recurring risk for anything needing `torch`/CUDA here -- check `nvidia-smi` before
  assuming GPU detection will just work, and don't kill others' processes without asking.

---

## 2026-07-22 (evening) — Pachirisu: depth stable this session, full handle localization succeeded, but ~20cm base-frame error found in the eye-in-hand calibration

Follow-on session. Goal was to retry `align_depth` (blocked twice before by USB controller
crashes) and, if stable, finally run the repo's real `detect_handle_and_placement` per the
plan at `~/.claude/plans/quiet-herding-puzzle.md`. Both succeeded — but a real-hardware
comparison surfaced a large, unexplained position error in the current calibration that
still needs root-causing.

### Depth held up this time — no USB crash

Same `align_depth:=true` launch that crashed 3 times last session ran for the entire
session (10+ min continuous, plus GPU/CPU load from repeated detection runs) with zero
xhci_hcd incidents. The runaway `control_transfer` warning burst that preceded every past
crash showed up once at startup (right after "Sync Mode: On", exactly as before) but this
time settled into steady low-rate background noise instead of escalating — both
`/camera/color/image_raw` and `/camera/aligned_depth_to_color/image_raw` held a clean,
low-jitter 30Hz throughout. **Not root-caused / not necessarily fixed** — this may just be
this session's luck rather than the fragility being resolved. **Action item for next
session: plug the RealSense into a confirmed USB3 port/cable directly** (last session's
dmesg showed it enumerating through a sub-hub port, `2-9-4` this session — still not a
verified root port) rather than continuing to treat each session's outcome as informative
on its own.

### `detect_handle_and_placement` ran for real, and is repeatable

Built `scripts/scratch/test_detect_handle_and_placement.py` per the plan: monkeypatches
`AppliancePerception.get_frame_to_frame_transform` on one instance (no repo source
touched) to build the base<-camera transform at call time from the prior session's
`cv2.calibrateHandEye()` result composed with a fresh `arm.get_state()["ee_pos"]` read,
exactly the composition validated to 1.1mm last session. Read-only arm connection
throughout (`ArmManager` direct-connect, only `get_state()` calls — `arm_server.py` up,
no `bulldog_bypass.py`, motion stayed locked the whole time).

**`handle_type="microwave handle"` (single-class prompt) succeeded on the first try** —
0.65-0.68 confidence across 3 runs from 2 different arm/camera viewpoints, no need for the
planned `"microwave"` whole-appliance fallback. This is notable: every session before this
one used a 3-class combined prompt (`["microwave", "microwave handle", "door handle"]`)
and the handle class never fired at all. Two full runs from different arm poses (the arm
moved between them, incidentally, from the manual-touch tests below) agreed on the
computed handle pose to within **2.4cm** — good evidence the detection + transform
composition is self-consistent, which matters for what comes next.

Hit one merge-carried regression along the way: `AppliancePerception.__init__` (from
today's `main`-merge) eagerly reads `grounded_sam.sam_predictor`, forcing a ViT-H load the
appliance/handle path never needs — and the SAM checkpoint isn't even present on this box.
Worked around in the scratch script only (`gsam._sam_predictor = object()` stub before
constructing `AppliancePerception`), not in the merged source. Images (input frame +
detection overlay) saved to `~/deployment_ws/pachirisu_handle_detection/`
(host-persisted).

### Real-hardware comparison found a ~20-27cm error — root cause still open

User manually touched the arm to the physical handle twice (once at a natural grasp
height, once at the lowest reachable point on the handle, 5.6cm lower) and we read back
`get_state()["ee_pos"]` both times, comparing against the computed handle pose:

| | measured EE pos | delta vs computed handle | distance |
|---|---|---|---|
| touch | (0.675, -0.173, 0.510) | (+0.093, -0.087, +0.236) | 26.8cm |
| lowest reachable point | (0.675, -0.173, 0.455) | (+0.093, -0.087, +0.180) | 22.1cm |

Both measurements shared identical dx/dy (only z differed, by exactly the 5.6cm the arm
was physically lowered) — a consistent, systematic bias, not noise. Ruled out several
candidate explanations one at a time:

- **Gripper fingertip vs wrist-frame offset** (~2-2.5in / 5-6.4cm): applying it made the
  gap slightly *worse* (approach axis at this orientation points almost entirely along
  world x, barely touches z) — not the explanation.
- **Re-running detection from scratch**: two independent detections (different arm poses)
  agreed to within 2.4cm on the computed handle pose, ruling out "one bad frame."
- **Camera-mount translation magnitude**: `t_cam2gripper` in the calibration has a norm of
  20.4cm; user's physical measurement of the actual wrist-to-camera distance is ~5cm — a
  4x discrepancy that looked like a smoking gun. **Tested directly** (rescaled
  `t_cam2gripper` to 5cm, same direction, via a `CALIB_TCAM_NORM_OVERRIDE` env var added
  to the scratch script) — this made the gap *worse* (24.4cm -> 29.0cm; 19.7cm -> 24.0cm),
  ruling out the translation magnitude as the (sole) cause despite the suspicious number.
- **Raw depth sensing**: user tape-measured the actual camera-to-microwave-door distance
  at 54cm; the detection's own `Plane depth` printout read 0.542-0.57m across the 3 runs —
  within 2-3cm. **Depth/plane-fit itself is accurate**, ruled out as a contributor.
- **Rotation validity**: `R_cam2gripper` is a proper rotation (orthonormal, det=1) and
  physically plausible (camera's looking direction expressed in the wrist frame is
  `(0.075, -0.110, 0.991)` — almost exactly the wrist's own forward axis, as expected for a
  forward-facing wrist camera). Not obviously broken, but not yet cleared either.

**Leading hypothesis, not yet confirmed:** a reference-frame mismatch between whatever
robot pose the original calibration script fed into `cv2.calibrateHandEye()` and what
`get_state()["ee_pos"]` reports today. That original calibration script no longer exists
(only its JSON output survived, per last session's note), so this can't be verified from
the artifact alone — the rotation can look individually valid and directionally sensible
while still being paired with a translation solved against a different origin, which would
produce exactly this "small rotation error, big position error at range" signature.

### Not done / next session

- **Redo the calibration.** This time, capture the robot-pose half of every hand-eye sample
  directly from `get_state()["ee_pos"]` (not a separately-derived FK), so no
  reference-frame mismatch is possible. **User will move the arm by hand for the
  calibration poses next time** rather than a scripted pose sweep.
- **Plug the RealSense into a confirmed USB3 port before starting**, per the depth-stability
  note above.
- Re-run `detect_handle_and_placement` + the manual-touch comparison after recalibrating,
  to confirm the ~20cm error actually closes.
- The approach-only reach test (compute a hover pose from the detected handle, command a
  single Cartesian move, no grasp) was planned for this session but never attempted — correctly
  deferred once the manual-touch comparison surfaced the calibration error; redo after
  recalibration, not before.

### End-of-session state

Stopped cleanly, SIGINT-only throughout in this order: `arm_server.py` (arm disconnected
cleanly), `realsense2_camera` (killed on exit, clean), `roscore`. No stale
`/tmp/kinova.lock`, no leftover roscore/realsense/nodelet processes (one pre-existing
defunct `[nodelet]` zombie from a prior session was already present at this session's
start, unrelated, harmless). Container `feed-noetic` left running but idle. No motion was
ever commanded to the arm this session — read-only `get_state()` only; all actual motion
was the user manually moving the arm by hand.

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
