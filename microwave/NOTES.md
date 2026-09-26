# Microwave open / close — notes (rchi-cpu-5, 2026-09-23)

Working notes for the scripts in this folder. Newest learnings first within each section.

## 2026-09-25 — gripper turned around 180°, push-close validated

Same Robotiq gripper, remounted rotated 180° about the wrist. Finger length unchanged, so
`GRIP_EXT` / `GRASP_X_ADJUST` / `VERTICAL_CORR` still work as tuned.

### What ran on hardware

- **Open (twice):** `--phase grasp --execute` then `--phase swing --smooth --hinge ...`.
  Detection 0.31–0.36 conf, two looks agree 0.2–0.7 cm, hinge edge 34.7–35.0 cm (unchanged).
  Grasp tracking 0.3–0.5 cm, `gripper_pos` after close **0.65–0.66** (was 0.92–1.00 before the
  turn-around; it held the handle both times, confirmed by eye). Smooth 45° and 50° swings, 0.0 cm
  off the last waypoint.
- **J6 has more room with the turned gripper:** 45° open → J6 90°, 50° → 88°, and 2° steps to
  56.7° with J6 95.7° (limit guard 115°; ~2.5° of J6 per 2° of door). Sim says the guard hits at
  ~78° door. J7 stayed ~154°.
- **Close while holding the handle:** `close --phase swing --smooth --target-close-deg 45`
  (0.0 cm) → `--phase push --push-dist 0.05 --no-release` (0.4 cm) → `--phase release`. Door shut.
- **push-close (new, no grasp) — door closed AND latched**, from the container-hold pose:
  ```bash
  python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --door-deg 90 --execute
  ```
  Left 12 steps (≥3.3 cm from the door model), forward 10, swing 22 steps / 89°, then the
  torque-watched push: baseline J1–J4 `[0.34, 8.49, 5.26, 9.98]` Nm, change **2.29 → 2.87 → 6.98
  Nm** at +5/+10/+15 mm → stopped at +15 mm, backed out 20 cm. The 3 Nm / rising-twice rule
  fired cleanly on the first try.

### The hinge must be re-derived every time the microwave moves

`FIXED_HINGE` and the 09-23 door-file hinge were both stale (radius 25.6 cm instead of 35).
Today's hinge came from the 09-23 hinge offset expressed in the DOOR's frame (9.0 cm deeper along
the normal, 32.9 cm along the face from the grasp), applied to today's grasp + door normal — it
matched the detector's hinge edge to ~1 cm: `[0.7907, 0.1419]` in the morning, `[0.6967, 0.2495]`
after the microwave was moved ~9 cm and turned ~9°. Translation-only shifting (the `--fast` rule)
ignores the rotation and was 3 cm off here. Still passed by hand as `--hinge`; de-hardcode TODO.

### push-close — how it works and what the demos taught

`door_push.py` (new). No grasp, gripper never touched, hand orientation never changes (so a held
container stays upright). Plan = the user's hand demo: **left** at the current depth to ~8 cm past
the hinge, **forward** to 28 cm from the hinge, **swing** about the hinge (radius spiralling in to
26 cm so the hand ends clear of the handle at ~33 cm) until 3 cm short of the model's closed door,
**push** straight in 5 mm steps until the joint torques say the door is shut, **back out**.
Routes tried in order: left-then-forward, straight, round the free edge's corner, back-left-forward.

- **The model's closed door face is 1–1.5 cm too deep.** The first push-close ran the swing
  0.5 cm "into" the model face and kept pushing a door that was already shut (user stopped it).
  Hand demos ended at x 0.672–0.680; the model said 0.686. Hence stop-short + torque-watched push.
- **`DOOR_PAST_HANDLE` 0.122 → 0.06** (`microwave_common.py`). The detector's 47 cm "span" includes
  the control panel; the user's hand passed the 90°-open door's edge at x ≈ 0.27, which only fits
  a door ending ≤ ~6 cm past the handle. Affects the release/back-off door model too.
- **"~90° open" is a guess.** A hand demo's start pose would have been inside a 90° door in the
  model, so the real "fully open" was probably 100–110°. `--door-deg` is the planner's only
  source for the open angle unless push-open/the pull wrote it to the door file.
- Contact numbers: free door push ≈ 2–3 Nm torque change; frame hit ≈ 7 Nm one step later.

### ±180° wrap — the thing that blocked most of the afternoon

The planners refuse any step where J1/J3/J5/J7 crosses ±180 (Kortex's behaviour there is still
**untested**). Getting to the hinge side from in front of the microwave crosses J1 **or** J3
depending on how the same arm posture is written (J1 162 / J3 −1 ≡ J1 8 / J3 179):
- **push-close only plans if the start has J3 ≈ +165…+178°** (moving left drives J3 DOWN, away
  from 180). A start with J3 ≈ −172° crosses in the first 2 cm — the user's own hand demo did
  exactly that. Fix used: slide the hand 3–4 cm left by hand before starting; read J3 back.
- The only wrap-free alternative is the other elbow (J4 ≈ +130 instead of −139) — a one-off
  hand reconfiguration, then re-validate the open task in it. Not done.
- `tools/wrap_test.py` is written (J7 then J1 across 180 with a 50 Hz wrong-way watchdog +
  `stop_action`). **First two attempts (09-25 ~15:55) were inconclusive**, see below.

#### wrap_test.py attempts, 09-25 ~15:55 (J7) — inconclusive, and a watchdog hole found

1. Run 1 (J7 −166.4 → −160): `moved +1.0 deg (expected +6.4) … returned False`. The arm was
   in `ARMSTATE_SERVOING_MANUALLY_CONTROLLED` (the user was moving it by hand), so this was expected.
2. The user placed the arm at J `0, 13.9, −180, −129.4, 0, 53.4, 90`. Run 2 (J7 89.9 → −160, short
   way +110.1) printed this, then the script exited:
   ```
   gripper 0.009; J7 now 89.9; speed medium
   approach: J7    89.9 ->  -160.0  (short way +110.1 deg)
     moved +0.0 deg (expected +110.1), now 90.0, max wrong-way 0.0, returned False
   STOPPED at approach -> -160: not the short way (or not reached). Check the arm.
   ```
3. **Then the arm moved.** A read-only check afterwards found it at
   `0.28, −4.92, 179.68, −133.70, 0.09, 38.77, −160.00`, which is **exactly** the logged run-2
   command (15:56:21). Kortex executed the move after the RPC had returned False, while nothing
   was watching. J7 went 90 → −160, but **which way it turned (+110 short / −250 long) was not
   recorded**. Ask the user what they saw, or re-run.
   - The arm state still read `SERVOING_MANUALLY_CONTROLLED` after this.
   - `validate_joint_move` accepts J7 → −160 and → 100 from that pose (10 s duration). A 1 s
     duration fails on speed/accel, which is a diagnostic artifact.
   - Why `move_angular` returned early is not root-caused. Suspects: a stale END/ABORT
     notification setting `end_or_abort_event` right after `clear()`, or an ABORT while in manual
     control followed by a later start. The `arm_server.py` stdout (`EVENT : …` lines) would say.
     This is the same family as [kortex-dropped-substep-bug]: the RPC return value is not the truth.
- **Fix made:** the watchdog in `wrap_test.py` now keeps polling after the RPC returns, until the
  joint has been still for 10 s (60 s cap), so a late move is still watched and still stoppable.
- **Before re-running:** hands off, and the state must read `ARMSTATE_SERVOING_READY` (if it doesn't,
  restart `arm_server.py` + `bulldog_bypass.py`). J3 is sitting at +179.7, right against the wrap.
  The test holds it there, which should be fine, but park it away from ±180 if convenient.
- **General lesson:** any script that trusts `set_joint_position`'s return value to mean "the arm is
  not moving" is wrong. Poll the joints for convergence or stillness instead.

### Other gotchas

- **`SESSION_NOT_IN_CONTROL` = someone is hand-guiding the arm** (state reads
  `SERVOING_MANUALLY_CONTROLLED`). Kortex rejects the next command; hands off before `--execute`.
- **Duplicate-IP route is back after a reboot/NetworkManager reset** (`ip route | grep 192.168.1`);
  needs `sudo ip route replace 192.168.1.0/24 dev enp4s0 metric 50` by the user.
- **Park pose re-recorded (v3)** from the user's hand-placed home: J3 −1° (v2 had 178). But its J1
  is 162.6°, i.e. the same wrap problem moved to J1. The v2 values are kept inside the file.
- The pull and push-open now write `door_open_deg` + `open_sign` to `~/.microwave_door.json`
  (`record_door_angle`), which push-close reads when `--door-deg` isn't given.
- Recording hand demos: `record.py`-style 10 Hz logger of joints/EE/gripper/arm state (kept in
  the session scratchpad; CSVs in `~/microwave_manual_*_2026-09-25.csv`, push-close orientation in
  `~/microwave_push_close_orientation_2026-09-25.json`).

### push-open — written, sim-only

`--phase push-open` on the open script: release at the end of the pull, back off, round the free
edge to the INNER face, push along the arc to `--push-target-deg` (85), out past the edge, park.
The sweep passed in sim at every radius tried (J6 4–37°). Not yet run on hardware — needs a pull
to ~50° first. The user's hand demo of it pushed with a fast sideways fling (door ended ~90° by
momentum); the scripted version is a slow contact arc instead.

## Files

| File | What it is |
|---|---|
| `real_gen3_ros2_grasp_and_swing_microwave.py` | Open task: `--phase grasp` (detect + face-square grasp), `--phase swing` (door arc; `--smooth` = one blended Cartesian trajectory), `--phase release` (release, back off, park). |
| `real_gen3_ros2_close_microwave.py` | Close task: `--phase view` / `regrasp` / `swing` / `push` / `release` / **`push-close`** (no grasp, 09-25). |
| `door_push.py` | push-close (validated 09-25) and push-open (sim only): side-of-gripper door pushes, torque-watched final push. |
| `tools/wrap_test.py` | MOVES THE ARM: supervised ±180 wrap test for J7/J1 with a wrong-way watchdog (now keeps watching after the RPC returns). 09-25 attempts inconclusive, see "wrap_test.py attempts". |
| `microwave_common.py` | Shared: IK, straight-line and slerped Cartesian planning, door collision model, release + back-off + park, re-grasp, open-door view. |
| `park_pose.json` | Hand-placed end ("park") pose for both tasks. The only thing meant to stay hand-set. |
| `~/.microwave_door.json` | Door geometry recorded by grasp (closed normal + grasp point) and swing (hinge). |
| `~/.microwave_last_grasp.json` | Last release: grasp pose, back-off pose, back-off joints. |
| `state_2026-09-23/` | Snapshot of both files above at the end of 09-23. |
| `tools/sim_check_swing.py` | Sim-only: IK / joint jump / J6 for every waypoint of an arc, from the real joints (`--iters 200` = singularity probe). |
| `tools/what_if_release_and_regrasp.py` | Sim-only: dry-run release / re-grasp planners from hypothetical arm states (fake arm). |
| `tools/plan_release_retreat_prototype.py` | Superseded prototype of the door-model planner (now in `microwave_common.py`). |

Shared, non-microwave code changed on 09-23 (lives outside this folder):
`src/feeding_deployment/perception/appliance_perception/appliance_perception.py` now sets
`last_door_normal_base` on each detection (10 added lines, nothing else changed). Deleted:
`scripts/real_gen3_ros2_yolo_grasp_microwave.py` (fully contained in the grasp-and-swing script).

## Commands that worked today

```bash
export ARM_RPC_HOST=127.0.0.1 HANDLE_DEPTH_CORR=0.001 CAMERA_UPSIDE_DOWN=false
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_large_images.xml
# open, FINAL video take (one command: 1 detection look, grasp, no pause, hinge derived, smooth 45 deg)
python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --phase both --fast --smooth \
    --target-angle-deg 45 --execute
# open, as separate steps
python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --phase grasp --execute
python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --phase swing --smooth \
    --target-angle-deg 45 --hinge 0.7396 0.2701 0.5425 --execute
# close while still gripping -- smooth version (video take): 45 deg back, then a 6 cm push, no release
python3 -u microwave/real_gen3_ros2_close_microwave.py --phase swing --smooth --target-close-deg 45 --hinge <same> --execute
python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push --push-dist 0.06 --no-release --execute
# close while still gripping (stop-and-go), then push, then release
python3 -u microwave/real_gen3_ros2_close_microwave.py --phase swing --target-close-deg 69 --hinge <same> --execute
python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push --push-dist 0.04 --no-release --execute   # + 0.02 more
python3 -u microwave/real_gen3_ros2_close_microwave.py --phase release --execute
```
Always dry-run first (omit `--execute`). Check the gripper is really on the handle by eye after
every grasp — `gripper_pos` can't tell.

## Results today

- **Final video take (end of day), one command, `--phase both --fast --smooth`:** detect (YOLO fallback
  "train") → grasp (0.2 / 0.3 cm tracking, `GRASP_X_ADJUST` 0.040) → hinge derived from the handle
  shift → singularity pre-check OK → 45° in one continuous motion, 0.0 cm off. About 10 s of detection
  before any motion.

- **Smooth swing works.** `--smooth` checks every waypoint in sim (IK, joint jump, J6), then sends
  the arc as one `CartesianTrajectoryCommand` (Kortex blends it): 45° in one continuous motion,
  finished 0.0 cm from the last waypoint. At 50° the check stopped it (J6 would reach 120.5°).
- **Smooth close works too:** `--smooth` on the close swing (11 waypoints, 0.0 cm off), then a 6 cm
  push (0.5 cm tracking) shut the door from 45°.
- **Full cycle, stop-and-go:** grasp → 68° open → swing back 69° → push 4 + 2 cm → release.
- **50° open → release → 16 cm back-off** worked.
- **Face-square grasp:** the detector now exposes the door-plane normal
  (`AppliancePerception.last_door_normal_base`). The grasp turns about vertical to approach along
  it (it keeps the gripper roll that fits the vertical bar). `GRASP_X_ADJUST` now acts along the
  approach axis.

## Constraints / gotchas found

- **J6 limits the opening.** J6 climbs with the door angle and hits the 115° guard (hard limit
  119.7°) at about 45–68°, depending on which joint configuration the grasp ended up in.
  **The real open task will need the door open more (~70°) → it will probably have to pull to
  ~45–50°, release, and then PUSH the door open the rest of the way.**
- **Releasing needs J6 headroom too:** backing straight off at 68° open adds ~12° of J6.
  Release at ≤ 50°.
- **J7 / J3 near ±180°:** the free-spinning joints (J1/J3/J5/J7) are wrapped to ±180°, and any move
  where one would change by >170° is refused. Kortex's behaviour across ±180° is **unverified** —
  the arm might take the short way or spin almost a full turn. The park pose has J3 = 177.9°, so
  paths to park from the door side currently cross 180° and get refused. Fix: re-record park with J3
  away from ±180°, or run one supervised J7 wrap test.
- **Joint-space moves near an open door can clip it** even when both ends are clear (e.g. back-off
  point → park: −5.4 cm in the door model). Use straight-line / slerped Cartesian legs checked
  against the door model. For the open task: back off, go toward the base (−x, `--pre-park-x`),
  then to park.
- **Smooth (Cartesian) swing can hit a wrist singularity.** Kortex aborted one smooth swing with
  `SINGULARITY_REGION` → `INVERSE_KINEMATIC_FAILED` about 5 cm into the arc (after a grasp that left
  J5 ≈ 5°). The same stretch made a 200-iteration PyBullet IK miss by 2.6 cm, so the `--smooth`
  pre-check uses that as a singularity probe (> 1 cm miss → fall back to the stop-and-go joint swing,
  which isn't affected). Don't "fix" the probe by adding iterations — that hid the warning once.
  `--fast` (`--phase both`): one detection look, no pause, hinge = last hinge + handle shift.
- **Kortex drops commands** (`ROBOT_MOVEMENT_IN_PROGRESS`). The executor waits for 1° convergence to
  the *commanded* joints and re-sends once. A dropped step followed by a double-size step produced
  `JOINT_ACCELERATION_LIMIT_REACHED`.
- **YOLO is unreliable on this red microwave.** From many views it reports "bus"/"train"/"suitcase"
  with no "microwave" at all. **A person reflected in the glass door makes it worse — nobody stand
  in the reflection during detection.** Workaround in place: `YOLO_FALLBACK_CLASS_IDS` (bus, train,
  suitcase, oven, refrigerator), used only when no "microwave" is found. The downstream checks (door
  plane, handle cluster, two-looks agreement ≤ 3 cm, plausible box) still guard it; today the fallback
  box gave the same handle and hinge edge (35.1 cm) as real "microwave" detections. **Longer term:
  fine-tune / train a detector on this microwave, or tweak further.**
- **Detection on an OPEN door from park fails:** from park the camera sees the microwave front
  square-on and the door edge-on, so the plane fit latches onto the front face. The camera must
  face the open door (`--phase view`). Viewing a 70°-open door hits the J6 guard at the end of the
  path (−116…−123°); viewing a 50° door passes.
- **Grasp was "a little too close"** (user): `GRASP_X_ADJUST` 0.045 → 0.035 → **0.040** (user-tuned
  on the arm). The final take used 0.040.
- **Camera:** the stream stalls after about 1 h (restart the node), and the camera fell off USB
  twice (once the whole xHCI controller: `sudo sh -c 'echo 0000:00:14.0 > /sys/bus/pci/drivers/xhci_hcd/unbind'`,
  then `.../bind`, run **once**). The IR "frames didn't arrive" warnings are a false alarm.
- **`PLAUSIBLE_X`** lowered 0.45 → 0.30 so a 50°-open door's handle (x ≈ 0.41) isn't rejected.
- **Hinge moves with the microwave.** Closing and handling nudge the microwave 1–2 cm. Today the
  hinge was re-derived by shifting it by the grasp-point change; check the swing's printed radius
  against the detector's "Hinge edge" (~35 cm).

## TODO next session

**Added 2026-09-25 (items below are from 09-23):**
- Run `tools/wrap_test.py` (J7, then J1) — if Kortex takes the short way, relax `continuous_ok`
  to the wrapped difference and most of the J1/J3 start-pose fiddling goes away.
- Hardware-test `--phase push-open` after a ~50° pull.
- Close start pose: make push-close pick/verify a start with J3 on the right side itself (or
  move there), instead of the user nudging the hand.
- Door open angle: measure it (detector on the open door, or push-open's recorded end) instead
  of `--door-deg` guesses; the model is conservative at 90° when the door is really wider.
- Hinge: automate the door-frame hinge re-derivation (see the 09-25 section) into the grasp.
- Commit — still nothing from 09-23 or 09-25 is committed.

1. **De-hardcode** (only the park pose stays hand-set):
   - hinge from the detector (it already finds the hinge-edge point — the 2nd return of
     `detect_handle_and_placement`, currently discarded — plus span and normal); drop `FIXED_HINGE`
     and `--hinge` except as an override;
   - swing direction from geometry;
   - close angle = current open angle; push distance = remaining depth to the closed grasp point;
   - door model size and height from detection; fingertip offset from sim FK;
   - `--pre-park-x` from the park file.
2. **Push-open phase** for the open task, to get past the J6-limited ~45–50° pull.
3. **Close-task grasp of the open door** (for now the user moves and grasps by hand): view pose that
   works for ~70° without J6 (partial turn toward the door or a different arm configuration), then
   detect + face-square grasp.
4. **Park pose / ±180° wrap:** re-record park away from J3 ±180°, or verify Kortex's wrap behaviour.
5. **YOLO robustness:** train or fine-tune for this microwave.
6. Commit today's work — **nothing from 09-23 is committed** (working tree on `detect-fridge-handle`).
7. The single-look `--fast` mode skips the two-looks agreement check. Fine for demo takes; for
   autonomous runs keep the default (two looks).

## State at shutdown (09-23, ~14:50)

- All processes stopped cleanly (bypass, arm_server, stub base, camera, calibration_tf, joint bridge,
  robot_state_publisher); ports 5000/5001 free. `/tmp/kinova.lock` was left behind — it clears on
  the next `arm_server.py` start.
- Door ~45° open with the gripper **closed on the handle**; the user had taken manual control
  (`SESSION_NOT_IN_CONTROL`), so the gripper was not opened by software.
- Next bring-up: `RCHI_CPU_5_SETUP.md` (on `microwave-task`). The `robot_state_publisher` used today
  loads `~/.ros/gen3_robotiq_2f_85.urdf` (with gripper), not the no-gripper xacro in the setup doc.
  Start the camera with `FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_large_images.xml` and
  `-p rgb_camera.auto_exposure_priority:=false` (15 Hz on all streams).
