# Autonomous microwave button press — runbook (ROS 2)

Visual servo onto the button + force-stopped press, pure translation, step-and-check.
**Proven on hardware 2026-09-21/22** (rchi-cpu-5, Comfee microwave, D435i wrist camera):
the arm pressed `timer_clock` unaided and the microwave registered it (commit `a2793b27`).
History and the bugs found getting there: `docs/button_press_handoff_2026-09-21.md` and the
constant-by-constant comments in the code.

## Where the code lives

| What | Where |
|---|---|
| **What the code does (plain-language walkthrough)** | `src/feeding_deployment/button_press/README.md` |
| Press driver (servo → approach → press → retract) | `src/feeding_deployment/button_press/autonomous_press.py` |
| Its ROS 2 inputs (pixels, force, depth, tf) / arm IK + motion gates | `button_press/perception.py` / `button_press/arm.py` |
| Contact rule + its measured constants (pure, tested) | `src/feeding_deployment/button_press/contact.py` |
| Panel plane fit, rays, servo correction (pure, tested) | `src/feeding_deployment/button_press/geometry.py` |
| Button detector ROS 2 node (`/button_detector/*`) | `src/feeding_deployment/button_press/detector_node.py` |
| Tool-force press detector (`/press_detector/*`) | `src/feeding_deployment/button_press/press_detector.py` |
| Reference-homography detector (the matching itself) | `src/feeding_deployment/perception/appliance_perception/reference_button_detector.py` |
| Perception launch (detector + press detector) | `launch/ros2/button_press_bringup.launch.py` |
| Build / extend / check a reference | `scripts/button_press/{make_reference,add_reference_view,check_reference_match}.py` |
| Standalone ament package + sheppy template | `scripts/button_press/pkg_template/` |
| Tests | `tests/test_button_press_{contact,geometry}.py`, `tests/test_reference_button_detector.py` |

Everything under `feeding_deployment.button_press` runs as a module
(`python3 -u -m feeding_deployment.button_press.<name>`), like `feeding_deployment.ros2.*`.
Before 2026-09-22 these were `scripts/scratch/button_live/*.py` and
`scripts/scratch/detect_button_press_force.py`; behaviour is unchanged apart from the fixes
listed at the end.

**Not yet integrated:** `PressMicrowaveButtonHLA` (`actions/press_microwave_button.py`) still
runs the older open-loop pre-press/press pose sequence. The driver is exposed as a library
(`autonomous_press.build_arg_parser()`, `Run(args).run()`) for that wiring.

**Someone stands at the physical e-stop for every `--execute`.** Killing `bulldog_bypass.py`
e-stops the arm within ~1 s; Ctrl-C in the driver stops the *next* step (the in-flight
≤1 cm step completes). `arm_halt.py` is unverified on hardware.

---

## 0. Environment (every terminal)

```bash
cd ~/feeding-deployment-button-task            # or wherever this checkout is
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_large_images.xml   # or images drop to 2-5 Hz
export ARM_RPC_HOST=127.0.0.1
export PYTHONPATH=$PWD/src:$PYTHONPATH        # PREPEND. `PYTHONPATH=src` alone drops rclpy.
mkdir -p ~/press_logs
```

## 1. Arm servers (no motion yet)

Order matters; each is its own terminal. **Start `joint_state_bridge` (step 2) within 10 s of
`arm_server`** — the Kortex session dies after 10 s with no client traffic and the bridge is
its keepalive.

```bash
python3 -u src/feeding_deployment/control/robot_controller/arm_server.py   # "Arm manager server started at 127.0.0.1:5000"
python3 -u scripts/stub_base_server.py                                       # bulldog needs a base to talk to
python3 -u scripts/bulldog_bypass.py                                         # motion UNLOCKED from here on
python3 scripts/session/arm_set_speed.py low                                 # scripts never set speed; you do
```

Check:
```bash
python3 -c "
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
ai=ArmInterfaceClient(); s=ai.get_state()
print('gripper',round(float(s['gripper_pos']),3),'| arm',ai._arm_interface.get_arm_state()['name'],'| speed',ai.get_speed())"
```
Want `ARMSTATE_SERVOING_READY`, speed `low`, gripper **~0.99 (closed)** — the press uses the
closed fingertips. `INVALID_USER_SESSION_ACCESS` / `ARMSTATE_IN_FAULT`: teleop, an e-stop,
or a new Kortex session took it. Recovery order: `arm_server` → bridge → stub base → bypass
→ `arm_set_speed.py low`.

## 2. Camera + tf chain

```bash
ros2 launch launch/ros2/microwave_bringup.launch.py      # system python3; camera + robot_state_publisher + joint_state_bridge + calibration_tf
```
Standing rule before any motion:
```bash
ros2 topic hz /joint_states                                          # ~50 Hz. Silent => bridge attached to a dead arm_server
ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame   # must print a transform
```

## 3. Perception

**Arm parked, nobody touching it for the first 3 s** (the press detector baselines then):
```bash
ros2 launch launch/ros2/button_press_bringup.launch.py \
    reference_dir:=$HOME/wrist_ref_red target_button:=timer_clock    # or start_30s
ros2 run rqt_image_view rqt_image_view /button_detector/debug_image
ros2 topic echo /button_detector/status        # want "locked <target> inliers=N" with N >= 15
```
Or by hand, one terminal each:
```bash
python3 -u -m feeding_deployment.button_press.detector_node --ros-args \
    -p reference_dir:=$HOME/wrist_ref_red -p target_button:=timer_clock
python3 -u -m feeding_deployment.button_press.press_detector --threshold 8 --publish --print-hz 0 --log ~/press_logs/force.jsonl
```
Want press-detector `rest noise` ~0.1 N and no `ARM WAS MOVING` warning; the overlay's bottom
bar turns from grey "no data" to green `|dF| 0.0x N`.

- **Never restart the press detector while the arm stack is up** — each restart opens new
  Kortex sessions and has killed `arm_server`'s (three times on 2026-09-21). Re-baseline it
  live instead: `kill -USR1 $(cat /tmp/press_detector.pid)`. The driver does this itself
  between phases.
- Fewer than ~15 inliers after the arm moved ⇒ the framing drifted; add a reference view
  (§6), do not retune the detector.

## 4. The press — one rung per invocation

`--tip-dist 0.154` is **measured** on this rig (plane 17.0 cm along the ray, contact at
1.6 cm); ruler numbers given during the session were ~10 cm off. Re-measure with `--jog` if
the gripper or camera mount changes.

```bash
P="python3 -u -m feeding_deployment.button_press.autonomous_press --target timer_clock"
$P                                                        # DRY RUN: preflight + every planned step + gates
$P --execute --stage 1                                    # lateral servo only; crosshair lands on the claw marker
$P --execute --stage 2 --tip-dist 0.154 --cap-override 0.05   # approach, stops 5 cm short
$P --execute --stage 2 --tip-dist 0.154                   # approach until CONTACT
$P --execute --tip-dist 0.154 --presses 1                 # servo, approach, press, retract
```
The exact command that pressed `timer_clock` from a ~17 cm standoff:
```bash
$P --execute --tip-dist 0.154 --presses 1 --fine-step 0.002 --press-travel 0.001 --cap-override 0.03
```
`start_30s` with `--presses N` actually runs the microwave.

After an abort the arm **holds**; the way back is printed. Retreat with
`$P --execute --stage 4 --resume-travel <metres printed>` (add `--resume-far-travel` if the far
phase ran), `$P --goto-start [--execute]` (≤20° per joint, back to the joints saved in
`~/press_logs/start_joints.json` — overwritten by every run), or
`scripts/session/goto_preset.py`. Per-step JSONL lands in `~/press_logs/`.

**Far start (~32 cm) is not proven.** Beyond 23 cm the driver runs a far phase first (holds
the button 110 px above the claw and closes to 20 cm). It has never completed on hardware;
start from ≤ ~20 cm until it has.

## 5. Shutdown

Re-check `gripper_pos` and `get_arm_state()` (idle, `SERVOING_READY`), then stop in reverse:
driver → press detector → detector node → bulldog bypass (arm e-stops ~1 s later, expected)
→ stub base → arm_server → camera/tf.

## 6. Building or extending the reference (no motion)

One reference per microwave unit (`reference.json` + frames). The detector abstains rather
than guess, so a reference that stops locking after the framing changes needs a new *view*,
not looser thresholds.

```bash
# does an existing reference match the live camera? (tries 0 and 180 deg)
python3 scripts/button_press/check_reference_match.py --ref $HOME/wrist_ref_red --topic /camera/color/image_raw --out /tmp/ref_match.png

# add a view for the current framing (validates on fresh frames before installing; backs up reference.json)
python3 scripts/button_press/add_reference_view.py --ref $HOME/wrist_ref_red \
    --near <X> <Y> --panel <X0> <Y0> <X1> <Y1> --note "why this view"

# from scratch (single-view; OVERWRITES): grab a sharp frame, then mark the button
python3 scripts/button_press/make_reference.py --grab --topic /camera/color/image_raw --out /tmp/ref_frame.png
python3 scripts/button_press/make_reference.py --mark /tmp/ref_frame.png --near <X> <Y> --panel <X0> <Y0> <X1> <Y1> --radius 14 --ref-dir <dir>
```
**Look at the output images** — inliers say the panel matched, not that the mark sits on the
right button. `add_reference_view.py` once placed a named button on label text; it now refuses
<15-inlier projections, but always eyeball the new view's marks. Restart the detector node
after changing the reference (it loads at startup).

`LEFT_CLAW_PIXEL` (377, 394) in `detector_node.py` is the left fingertip's pixel for the
current gripper opening; re-measure it if the opening changes (override with
`-p left_claw_pixel:=[x,y]`).

## Gotchas (each cost real time on 2026-09-21)

- Force baseline goes stale after any move (pose-dependent wrench bias; the detector stops
  adapting above 4 N) — hence the driver's SIGUSR1 re-baselines. Slack the wrist camera cable
  (suspected contributor to drift).
- Only act on a `button_pixel` received *after* the arm settled; the node runs at ~5 Hz.
- `pkill -f <name>` from a wrapper shell can kill the shell itself; use the pidfile or
  `pgrep -f "^python3 -u -m feeding_deployment.button_press.<name>"` + `kill`.

## Changes when this moved out of `scripts/scratch/` (2026-09-22)

- **Bug fix:** `Run.reanchor_seed()` read `self.execute`, which `Run` never sets, so the far
  phase always crashed with `AttributeError` at its very end. Now `self.args.execute`. (The
  proven runs started close and never reached it.)
- The driver finds the press detector via `/tmp/press_detector.pid` (override
  `$PRESS_DETECTOR_PIDFILE`) instead of a `pgrep` on the old script path, and never matches
  itself (SIGUSR1 would kill it).
- Contact rule and plane fit extracted into `contact.py` / `geometry.py` unchanged; replaying
  scripted force sequences through old and new code gives identical decisions (only abort
  message wording differs).
- `SCENE_CONFIG` resolves from the package, so the driver no longer has to run from the repo
  root.
