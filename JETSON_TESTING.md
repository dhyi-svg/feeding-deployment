# Jetson testing — running a live microwave session

The **operator** file: terminal layout, positioning the arm, the stop levers, and the
gotchas that cost time in a live session.

Companion to, not a replacement for, `JETSON_SETUP.md` — that file is the ordered
bring-up (install → env → hardware checks → servers → ROS → scripts). This one assumes
those commands and covers what a session actually feels like from the chair. Where they
disagree on a command, **this file is newer** (see "Corrections to older docs" at the
bottom).

Last updated: 2026-08-11, written and revised during live bring-ups.

---

## 0. Terminal layout

Six terminals, one job each. **1–4 block and stay open; you work in 5; 6 stays idle.**

**Every command block below is labelled with the terminal it belongs in.** If a block
says "Terminal 5", it runs in the work terminal — terminals 1–4 are occupied by
long-running processes and you cannot type in them.

| # | Job | Blocks? | Needs the env block? |
|---|---|---|---|
| 1 | `arm_server.py` | yes | yes |
| 2 | `stub_base_server.py` | yes | yes |
| 3 | `bulldog_bypass.py` | yes | yes |
| 4 | `ros2 launch` | yes | **no** — system `python3`, sets its own |
| 5 | **work terminal** — speed, checks, all task scripts | no | yes |
| 6 | stop / panic | no | yes |
| 7 | `detection_server.py` | yes | yes |

**Env block — paste into terminals 1, 2, 3, 5 and 6 (NOT terminal 4):**
```bash
cd ~/feeding-deployment
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$HOME/feeding-deployment/src:$PYTHONPATH"
export ARM_RPC_HOST=127.0.0.1
export CUDA_VISIBLE_DEVICES=""       # GroundingDINO: CPU only on this box
export HANDLE_DEPTH_CORR=0.094       # motion scripts refuse to run without this
export HANDLE_LAT_CORR=0.0           # lateral error is detection VARIANCE, not bias
PY=$HOME/feeding-deployment/.venv/bin/python
```

---

## 1. Before anything: is the arm actually ready?

Run this **before** starting `arm_server.py` — it opens its own Kortex session and only
one process may hold the arm at a time.

**Terminal 5** (before `arm_server.py` is started):
```bash
$PY scripts/session/arm_probe_state.py
```

Want **`ARMSTATE_SERVOING_READY (7)`**.

### `ARMSTATE_SERVOING_MANUALLY_CONTROLLED (9)` — the green-LED trap

Something else holds manual control. Seen live on 2026-08-05: the arm booted with a
**green base LED instead of blue**, the web app initially refused to load, and the state
read `MANUALLY_CONTROLLED` through two full power cycles. Cause: **an Xbox controller
plugged into the arm's own USB port.** Unplugging it cleared the state instantly — but
**the LED stayed green**. Trust the probe, not the LED.

Check, in order:

1. **A gamepad in the Gen3's own USB port on the base.** It grabs control independently
   of this computer, so nothing on the Jetson (`/dev/input/js*`, `ps aux`) will show it.
2. **The Kinova web app open on a jog/control page** — close the tab entirely, not just
   navigate away.

Distinct from a *transient* `MANUALLY_CONTROLLED` for ~5 s after a `REACH_JOINT_ANGLES`
call, which clears on its own (see `TESTING_LOG.md`). The Xbox case is persistent and
will not clear until you unplug.

If the web app won't load, try **`http://`** — port 443 is closed on this arm, 80 is
open, so an `https://` default silently fails while the arm is perfectly healthy.

### Hardware present

**Terminal 5:**
```bash
cat /sys/class/net/enP8p1s0/carrier     # want 1
ping -c1 192.168.1.10                   # want a reply
lsusb | grep -i intel                   # want the RealSense (8086:0b3a)
# want NOTHING. The [h]/[a] brackets stop grep from matching its OWN command line --
# a plain `grep -iE "home_launch|..."` always prints itself and looks like a hit.
ps aux | grep -E "[h]ome_launch|[a]rm_driver"
```

---

## 2. Bring-up

**Terminal 7 — START THIS FIRST.** It takes ~6 min to load (2.4 GB of libraries and
weights off a 13.5 MB/s microSD) and needs nothing but the camera, so start it now and
let it load while you do the rest of the bring-up. Without it every task script loads its
own copy and each run costs ~8 min instead of ~73 s.

```bash
DETECTION_LOG_DIR=~/captures/session $PY -u scripts/session/detection_server.py
```

It only depends on the camera, so it survives `arm_server` restarts, e-stops, teleop and
bulldog restarts — start it once when you sit down, not once per run.

**Terminal 1** (wait for it to report connected):
```bash
$PY -u src/feeding_deployment/control/robot_controller/arm_server.py
```

**Terminal 2:**
```bash
$PY scripts/stub_base_server.py
```

**Terminal 3** — this unlocks motion:
```bash
$PY scripts/bulldog_bypass.py
```

**Terminal 4** — no env block; needs system `python3` for `lark`, and the launch file
already defaults `arm_rpc_host=127.0.0.1` and sets its own `PYTHONPATH`:
```bash
cd ~/feeding-deployment
ros2 launch launch/ros2/microwave_bringup.launch.py
```

**Terminal 5 — set the speed. The task scripts never do this themselves.**
```bash
$PY scripts/session/arm_set_speed.py low
```

They only *print* the speed, so the arm runs at whatever it was last left at. `low` =
30 °/s joint, 60 °/s² accel, 0.15 m/s linear, and is the floor through this API.
**Must come after terminal 3** — `set_speed` calls `_require_bulldog()` and asserts
otherwise.

---

## 3. Verify the TF chain

**Terminal 5:**
```bash
ros2 topic hz /joint_states                                     # ~50 Hz
ros2 topic hz /camera/aligned_depth_to_color/image_raw          # ~14 Hz
$PY -u scripts/session/check_tf_vs_fk.py
```

Want `VERDICT: OK`.

**Do not check the raw `tf2_echo` translation against a fixed expected value.** The
camera is eye-in-hand, so `arm_base_link → camera_color_optical_frame` is a function of
the current joint configuration — there is no correct constant. `check_tf_vs_fk.py`
compares the **EE → camera offset** against the saved calibration (~7 cm) instead, which
*is* invariant. A broken joint bridge or calibration shows up as metres, not centimetres.

Two benign messages you will see and can ignore:

- `Invalid frame ID "arm_base_link" ... frame does not exist` at startup — latched
  `/tf_static` still arriving. It resolves in a second or two.
- `tf2 lookup at the frame stamp failed ... using the latest instead` — the documented
  fallback in `tf_interface.py`. Fine while the arm is static, which it is during
  detection.

---

## 4. Position the arm so the camera sees the microwave

**There is no recorded viewing pose, and no script moves the arm to one.** The task
scripts detect from wherever the arm happens to be. `config/local_arm_presets.yaml` has
only `home_pos` and `retract_pos`.

Position it **by hand with the Xbox controller**, then **unplug the controller** (that is
what restores `SERVOING_READY` — see §1), then check by eye:

**Terminal 5:**
```bash
$PY -u scripts/session/grab_camera_frame.py /tmp/view.png
```

Open `/tmp/view.png` and confirm. Target: **~39–40 cm from the handle**, which is what the
2026-07-29 successful run used.

### What has to be in frame

Working backwards from what `detect_handle_and_placement` consumes:

| Needed | Why |
|---|---|
| **The whole handle, with margin** | GroundingDINO boxes it, then DBSCAN clusters the points protruding from the door plane. A handle clipped at the frame edge gives a truncated cluster and a centroid that slides along the bar. |
| **A good patch of the flat door face** | The RANSAC plane fit is what "protruding" is measured against. |
| **The top edge of the appliance** | `top_of_appliance` is the max-height point of the plane cloud; the grasp script caches it to drive the post-release lift. |

**Not** needed: the hinge, or the far side of the appliance. The arc uses the assumed
`+0.32 m` hinge, not the perceived one.

The whole microwave does not have to be visible — but in practice, if the handle is
comfortably inside the frame at ~40 cm, most of the front face is too.

### If you move the microwave instead of the arm

Fine, and often easier. Two consequences:

- The `TRUTH` / `STALE_TRUTH` constants printed by the scripts are now meaningless.
  They are **reporting-only and not gated on** — ignore the "cm from touched truth" line.
- **Rotation is what matters, not translation.** `DOOR_W` adds 0.32 m along **+y in the
  arm base frame**, a fixed world direction, and the hinge is computed relative to the
  grasp pose — so sliding the microwave a few cm cancels out, but **yawing it does not**.
  Keep the door face square to the arm.

---

## 5. Stopping — read this before you run anything with execute

Everything past this point moves the arm. Know how to stop it first.


> **`arm_stop_action.py` IS NOT A STOP. Verified broken 2026-08-12.** It printed
> success during a live arc and the arm kept moving for 11 more seconds — `Base.StopAction()`
> did not abort the in-flight motion, and even if it had, the arc loop would have
> commanded the next waypoint anyway. It is left in the tree as a diagnostic only.

**Only two things are PROVEN to stop this arm.** A third is implemented but unverified.

| | mechanism | status |
|---|---|---|
| 1 | **physical e-stop** | proven, no delay |
| 2 | `pkill -f bulldog_bypass.py` | proven — stopped the 2026-08-12 incident, ~1 s delay |
| 3 | `scripts/session/arm_halt.py` | **UNVERIFIED — do not rely on it yet** |

**Escalate in this order:**

**Terminal 6** — leave it idle with this pre-typed, one keystroke away:
```bash
# Kills the heartbeat -> emergency_stop within ~1 s. This LATCHES, which is what
# actually halts a multi-waypoint script: every later command then asserts.
pkill -f bulldog_bypass.py
```

**The physical e-stop beats it and has no delay.** Keep a hand on it during any motion —
the bulldog path takes ~1 s, which was long enough on 2026-08-12 for the arc to keep
pulling.

Recovery after either: restart `arm_server.py`, then re-run `bulldog_bypass.py`.

### `arm_halt.py` — implemented 2026-08-12, NOT yet verified on hardware

```bash
$PY scripts/session/arm_halt.py            # stop + latch
$PY scripts/session/arm_halt.py --clear    # release
$PY scripts/session/arm_halt.py --status   # read only
```

Calls `Base.Stop()` (what `emergency_stop` uses) **and** latches server-side, so all 15
motion methods assert and a client loop dies rather than sending its next waypoint.
Recoverable without restarting `arm_server`. **Requires an `arm_server` restart to load.**

Until it has been seen to fire correctly, treat it as untrusted and use 1 or 2. The test:
restart the server, `--status` → False, `halt`, command the arm to its *current* joint
values (no motion even if the latch fails) → must raise, `--clear`, same command →
accepted. Then live in free space at low speed, nothing grasped, and confirm no
`set_joint_position` lines follow `halt` in `arm_commands_log.txt`.

**Why the latch matters:** a stop that only cancels the current motion is useless against
a script in a loop — it just sends the next waypoint. Anything proposed as a soft stop
must both halt motion *and* make subsequent commands refuse.

**Ctrl-C on a motion script is not a stop.** The scripts install no signal handler, and
`execute_command` is a blocking RPC into a *separate* process — killing the client does
not recall a command the controller already holds. Ctrl-C prevents the *next* waypoint,
not the one in flight.

`bulldog_bypass.py` has **no software e-stop** — it replaces the real bulldog, which
subscribes to `/experimentor_estop`, with nothing. The one retained safety property is
fail-closed on process death: if the bypass dies, the arm e-stops within ~1 s. That is
what makes option 2 work.

**Recovery after option 2, an e-stop, or Xbox teleop:** the Kortex session faults. Restart
`arm_server.py`, then **re-run `bulldog_bypass.py`** — a fresh server re-locks motion.

---

## 6. Run the task

Two rules that apply to **every** command in this section:

- **Always pass `-u`.** Piping through `tee` makes Python's stdout block-buffered, so
  every `print` sits in a 4–8 KB buffer until the process exits — you get a blank terminal
  for minutes, then everything at once. (`pybullet build time` still appears, because the
  C extension writes straight to fd 1.) On the arc this is the difference between watching
  per-waypoint tracking errors live and finding out afterwards.
- **Always `tee`.** Detection images never reach disk (§7), so a run's stdout is the only
  surviving record of what the detector saw. Log the dry runs too.

### 6a. First, understand `--steps`

Read this before running anything below — the commands in 5b use it.

A dry run prints `max joint jump`: the largest per-joint difference between where the arm
is and where it is going. That is the **size of the move**, not a discontinuity. But a
single command moves the arm there in one go, the controller interpolates in *joint*
space (so the gripper traces an unchecked curve), and nothing in this pipeline
collision-checks the path.

> **The default is `--steps 1` — the move is NOT split unless you pass `--steps N`.**
> There is no automatic splitting and no threshold that kicks in. Omit the flag and you
> get the historical single command. `--steps 12` is the recommended starting point.

`--steps N` splits the move into small, individually-gated, abortable increments. **Every
step is checked in sim before anything is commanded, and the whole move is refused if any
step fails** — a partially-executed chain would leave the arm somewhere unplanned.

Pass it on the **dry run too**, even though nothing moves. The point of a dry run is to
inspect the plan you are about to execute, so it has to be the *same* plan: without the
flag you are shown a single large jump and then execute something structurally different.

**What a step feels like at `--steps 12`** (measured, 26.8 cm reach): the gripper moves
**2.2 cm** per step, no part of the arm moves more than ~3.3 cm, each increment takes
0.17–0.33 s at `low` speed, and the whole move is ~13 s wall clock. Twelve small nudges
with ~1 s pauses — not one smooth reach, and you get 11 abort points.

**`--interp` picks how the split is done** (only applies when `--steps > 1`). Measured on
the 2026-08-05 pose, 68.4° unchained, 12 steps:

| `--interp` | worst single step | endpoint | final posture vs the gated solution |
|---|---|---|---|
| **`joint`** (default) | **5.7°** | 0.001 cm | **0.0°** |
| `cartesian` | 10.0° | 0.001 cm | **83.8°** |

**`joint`** interpolates the joint vector straight to `q`, the exact solution the dry
run's gates validated. Per-step motion is bounded by construction (total/N) and the arm
ends in the posture you were shown. The gripper follows the same curve the unchained move
would have taken — no new path, just cut into abortable pieces.

**`cartesian`** interpolates the gripper along a straight line and re-solves IK per step.
The gripper path is predictable, but each solve drifts through the 7-DOF null space: both
modes hit the target pose to 0.001 cm, yet cartesian ended **83.8° away in joint space**
(J1 −4.5° → −88.2°). Same gripper pose, very different arm posture — and the grasp and arc
scripts seed their IK from wherever this leaves the arm. Use it only when the
straight-line gripper path itself matters.

In cartesian mode, sub-stepping alone does **not** bound joint motion — it bounds gripper
motion, and a sub-step's IK can still land on a distant branch. `--max-step-deg`
(default 20°) is what bounds it. In joint mode the bound is structural. Either way the
script confirms the chain ends at the gated target and prints the posture difference, so
null-space drift cannot pass silently.

### 6b. The run sequence

**Terminal 5** — everything below runs here.

```bash
```

**Step 1 — pick ONE path: (a) approach only, or (b) full grasp. They are alternatives,
not a sequence.** Each path is its own dry run followed by the identical command plus
`--execute`. Every invocation, dry run included, costs **~85 s** while GroundingDINO
Swin-B runs on CPU.

**Set `DETECTION_LOG_DIR` on every one of these.** It costs nothing and turns on the
repo's own detection diagnostics — see "Detection logging" below for why you want it.

**(a) Approach only** — one move to a 12 cm standoff. Never closes the gripper, never
touches the door. Choose split or unsplit:

```bash
# split into 12 gated increments -- RECOMMENDED (see 5a)
mkdir -p ~/runs/$(date +%F) && \
DETECTION_LOG_DIR=~/captures/approach_dry \
  $PY -u scripts/real_gen3_ros2_approach_microwave.py --steps 12           2>&1 | tee ~/runs/$(date +%F)/01a_approach_dry.log
mkdir -p ~/runs/$(date +%F) && \
DETECTION_LOG_DIR=~/captures/approach_exec \
  $PY -u scripts/real_gen3_ros2_approach_microwave.py --steps 12 --execute 2>&1 | tee ~/runs/$(date +%F)/02a_approach.log

# ...or unsplit: one single command. This is what you get if you omit --steps.
mkdir -p ~/runs/$(date +%F) && \
DETECTION_LOG_DIR=~/captures/approach_dry \
  $PY -u scripts/real_gen3_ros2_approach_microwave.py                      2>&1 | tee ~/runs/$(date +%F)/01a_approach_dry.log
mkdir -p ~/runs/$(date +%F) && \
DETECTION_LOG_DIR=~/captures/approach_exec \
  $PY -u scripts/real_gen3_ros2_approach_microwave.py --execute            2>&1 | tee ~/runs/$(date +%F)/02a_approach.log
```

**(b) Full grasp** — detect twice → pre-grasp → grasp → close. Does the approach itself,
so do not run (a) first. `--steps` splits **both** of its moves, each guarded separately:

```bash
mkdir -p ~/runs/$(date +%F) && \
DETECTION_LOG_DIR=~/captures/grasp_dry \
  $PY -u scripts/real_gen3_ros2_grasp_microwave.py --steps 12           2>&1 | tee ~/runs/$(date +%F)/01b_grasp_dry.log
mkdir -p ~/runs/$(date +%F) && \
DETECTION_LOG_DIR=~/captures/grasp_exec \
  $PY -u scripts/real_gen3_ros2_grasp_microwave.py --steps 12 --execute 2>&1 | tee ~/runs/$(date +%F)/02b_grasp.log
```

Measured on the 2026-08-11 pose, both moves land within 0.001 cm of the gated target:

| move | unchained | `--steps 12` |
|---|---|---|
| pre-grasp | 57.0° | **4.8°** |
| grasp | 18.6° | **1.6°** |

The second move is only 12 cm, so 12 steps makes it 1.6° apiece — finer than needed.
`--steps 6` still keeps the pre-grasp under 10° per increment if that feels slow.

What to want from either dry run: handle confidence ~0.55–0.68, all safety gates `PASS`,
IK error < 2 cm. With `--steps`, additionally every sub-step under 20° and an endpoint
`0.0 deg from the gated joint solution`. For (b), the number that matters most is
**`detections agree to X cm`** — under ~1 cm means detection is behaving; the script
refuses above 3 cm.

> **Do not run (a) then (b).** Approach leaves the arm at a 12 cm standoff, and the grasp
> script re-detects from wherever it is. Detection needs a viewing distance — at ~2 cm it
> returned a handle 6.7 cm off (2026-07-29), because the wrist camera cannot see the
> appliance and the plane fit is meaningless. The two-detection agreement gate can pass
> with both looks equally wrong. If you ran (a), back the arm off to ~40 cm before (b).

### Detection logging and offline replay

`AppliancePerception` already builds every diagnostic overlay (plane fit, candidate
cluster, final handle/hinge pixels) and already writes a JSON sidecar holding the
intrinsics **and the 4×4 base←camera matrix** — `_log_detection_inputs` exists
specifically "so the detection can be re-run offline". Both hooks silently no-op when no
data logger is attached, which was the default until 2026-08-11. Setting
`DETECTION_LOG_DIR` attaches one.

With it set you can re-run the **genuine** detector offline — no arm, no camera, model
loaded once — across several captures at a time:

```bash
$PY -u scripts/session/replay_detection.py ~/captures/pose1 ~/captures/pose2 ~/captures/pose3
```

It prints each capture's handle pose, the worst pairwise disagreement against the 3 cm
gate, and writes `replay_*.png` beside each capture so you can **see** which pixels were
clustered as "handle" rather than inferring it from a centroid coordinate.

Handle poses are in `arm_base_link`, a **world-fixed** frame, so captures taken from
different viewpoints *must* agree. Disagreement is a real finding, not an artefact of
having moved the arm. This is also how you iterate on `HANDLE_DEPTH_CORR`, prompts or
thresholds against **fixed** data, so a change in the answer means a change in your code
rather than a different frame.

Optional, between an (a) dry run and its execute, to inspect the *unchained* gripper path
(note the `=` — argparse eats a leading `-` otherwise):

```bash
$PY -u scripts/session/check_joint_path.py --to=<joint target printed by the dry run>
```

**Step 2 — check the grip by hand.** `gripper_pos` saturates near 1.0 whether or not the
handle is between the fingers; it cannot confirm a grasp, and the script deliberately
prints no verdict.

**Step 3 — door arc. MOTION, and it moves immediately: there is no `--execute` on this
one.** Start any recording first.

```bash
$PY -u scripts/real_gen3_ros2_open_arc_microwave.py 2>&1 | tee ~/runs/$(date +%F)/03_arc.log
```

Plans all waypoints first (seeded IK, unseeded fallback), then executes with a settle-wait
and a 2.5 cm per-waypoint tracking abort. Typically 10 of 11 waypoints, ~90°. **A stop at
the last waypoint is the door's own limit, not a fault.**

After the arc the arm is still gripping: **release first, clear laterally, then back out.**
Never withdraw while gripping. Still done by hand.

---

## 7. Logging, and two gaps

**What you get:**

- `src/feeding_deployment/control/robot_controller/safety_log/arm_commands_log.txt` —
  one timestamped, full-precision line per command. Replayable.
- Script stdout: detections, corrections, every gate result, IK error, joint jump,
  per-waypoint commanded-vs-actual tracking, gripper at each step.
- `/tmp/microwave_top_z.txt` — cached `top_of_appliance` z.

**Gap 1 — the command log is wiped on every `arm_server.py` start.** It is opened `"w"`,
not `"a"` (`arm_interface.py:42-45`). Since every session restarts the server, **copy it
off before restarting** or the prior run is gone.

**Gap 2 — detection images only reach disk if you ask (FIXED 2026-08-11).** `_log_image`
and `_log_detection_inputs` write nothing unless a `data_logger` is attached, and the
scripts used to construct `AppliancePerception(GroundedSAM())` with the default
`data_logger=None` — so every overlay was built, held in memory and discarded at exit.
The approach and grasp scripts now attach one when **`DETECTION_LOG_DIR`** is set (see
§6b "Detection logging"). **Set it on every run**: without it a bad detection still
cannot be reviewed afterwards, and `replay_detection.py` has nothing to work from. The
arc script does not detect, so it has no equivalent.

**So `tee` everything**, including dry runs — and set `DETECTION_LOG_DIR`. Between them
they are the whole record: stdout for the numbers, the capture directory for the pixels.

---

## 8. Corrections to older docs

Found live on 2026-08-05; the older files still carry the earlier version.

| File | Says | Actually |
|---|---|---|
| `JETSON_SETUP.md` §5, `docs/microwave_ros2_runbook.md` rung 2 | expect `tf2_echo` ≈ `[0.255, 0.018, 0.565]` | Pose-dependent — the camera is eye-in-hand. No fixed value exists. Use `check_tf_vs_fk.py`. |
| `JETSON_SETUP.md` §6b → §6c | approach `--execute`, then grasp `--execute` | Alternatives, not a sequence. Approach leaves the arm at 12 cm; the grasp script re-detects from there and detection needs ~40 cm. |
| `JETSON_SETUP.md` §6, runbook rung 5 | `set_speed("low")` mentioned in passing | Nothing sets it. Run `scripts/session/arm_set_speed.py low` explicitly, after the bypass. |
| all | (silent) | Pass `-u` to every task script, or `tee` hides all output until exit. |
| all | (silent) | `stop_action()` exists and is the right first response — recoverable, unlike killing the bypass. |
