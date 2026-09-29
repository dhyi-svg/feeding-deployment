# Microwave open / close — notes (rchi-cpu-5, 2026-09-23)

Working notes for the scripts in this folder. Newest learnings first within each section.

## 2026-09-29 — container close demonstrated by hand (the reference close for this setup)

**±180 WRAP SETTLED ON HARDWARE (09-29): Kortex takes the SHORT way** for a `JointCommand` across ±180, on
both actuator sizes. `tools/wrap_test.py` (supervised, 50 Hz wrong-way watchdog, gripper empty):
- **J7** (small actuator): +175 -> -175 moved **+10.0** (short way), back **-10.0**, 0.0 deg wrong-way.
  (Sequence now approaches on the side J7 is already on -- 09-25's "approach" from +90 to -160 was itself a crossing.)
- **J3** (large actuator): -175 -> +175 moved **-10.0**, back **+10.0**, 0.0 deg wrong-way. New `wrap_test.py 3`
  mode: only from J3 within 10 deg of ±180, stays within 5 deg of it.
- J1/J5 not tested (same actuator sizes as J3/J7). RPC returned False on one move that fully executed -- judge by joints.
- Consequence: `continuous_ok`'s "refuse any wrapped change > 170" is no longer needed for small per-step moves;
  this is what forced the J3 +165..178 start, the J1 park problem and the retreat-over block.

**Swing open quicker (code, not yet run on hardware):** `--swing-send-every N` (default 3) -- the smooth swing
is still sim-checked every 2 cm, but Kortex gets every 3rd waypoint (+ the last) = 6 cm chords (~1.3 mm inside the
arc at r 34 cm). Kortex slows at every Cartesian waypoint (blend <= 1 cm), so 2 cm spacing crawled at ~2.5 cm/s.
`1` = the old behaviour.
- **Ran on hardware 09-29 15:51** (`--phase both --one-take --swing-max --execute`, default `--swing-send-every 3`):
  2 looks 0.3 cm, handle conf 0.69-0.77, grasp 0.0 cm, gripper 0.843, swing-max **80 deg** (90/85 fail on J6), 24
  sim-checked waypoints -> **8 sent**, 47.8 cm arc in **11.6 s (~4.1 cm/s)**, 0.0 cm off the last waypoint. The 09-28
  dense swing ran ~2.5 cm/s (same arc would be ~19 s). Door/microwave had moved since the morning (handle +3 cm z,
  4 cm left): hinge carried over `[0.9013, 0.0684, 0.3104]` r 34.3 (detector 35.0-35.2). Needed the view pose pulled
  back to hand x ~0.40 (at x 0.46 the microwave filled the frame, left edge cut, 8/8 looks found nothing).
- **The 80-deg swing PULLED THE MICROWAVE along (user, 09-29)** -- the commanded arc doesn't match the door's real
  arc well enough (hinge carried over in the door frame after the microwave moved ~3-4 cm; the door may not be a
  pure vertical pivot -- Pachirisu 07-28 saw it rise ~6 cm). Nothing measures this yet: the smooth swing has no
  force/tracking check mid-trajectory. Ideas: watch joint torques during the swing (like push-close's contact rule)
  and stop/re-centre the hinge; fit the hinge from the first few cm of a compliant/stepped pull; derive the hinge
  from the detector's hinge edge directly instead of carrying it over.
- Changed after that: `SWING_MAX_TRY_DEG` 90 -> **75** (`--swing-max` now tries 75 down to 45); `--swing-send-every`
  default 3 -> **4** (8 cm chords, <= 2.3 mm inside the arc at r 34 cm).
- **Ran 09-29 15:55 with those:** 2 looks 0.4 cm (conf 0.91), grasp 0.0 cm, gripper 0.847, **75 deg**, 23 checked
  waypoints -> 6 sent, 44.9 cm in **9.9 s (~4.5 cm/s)**, 0.0 cm off the last waypoint. Fewer waypoints only helps a
  little now -- the 1 cm blend cap in `kinova.py` is the next lever (arm_server restart; affects grasp/push too).
- **STILL DRAGS THE MICROWAVE at 75 deg** (user): the whole microwave **turns clockwise about an axis** during the
  swing, i.e. the handle is being pulled along a path that isn't the door's real arc, and the body rotates to follow.
  Hinge was carried over after the previous run had already dragged it ~5 cm / ~1 deg. **Fix the swing geometry
  before more speed:** hinge straight from the detector's hinge edge (35.1 cm there vs the carried 34.3), check
  whether the true hinge is further from / closer to the arm than the model, and/or stop on a joint-torque rise.
  Re-derive the hinge every run (the microwave moves after each swing).
- **Little pause just after the swing starts -- NOT intentional.** `kinova.py` gives the FIRST Cartesian waypoint
  blending_radius 0 (full stop). With `--swing-send-every 4` the first waypoint sent is 8 cm into the arc, so the arm
  starts, stops there, then continues. Fix (not done): prepend the arm's current EE pose as waypoint 0 in
  `run_swing` (and the grasp / push trajectories), so the forced stop is where the arm already stands.
- **TODO (still open): coming back from the open.** After the swing the arm holds the handle at ~75-80 deg with J6
  ~111 and there is no validated scripted release + retreat from there (`--retreat-over` blocked 09-28; the out leg
  raises J6; the wrap no longer blocks it since 09-29). The user returns the arm by hand for now.

Bring-up: same as 09-28, but the camera must be started with **`-p enable_infra1:=false -p enable_infra2:=false`**
(without them the node opened Infra1/2 at 848x480@30 next to depth @15 and published NO frames at all).
`ros2 topic hz` prints nothing on this box even when images flow -- check rate with a small rclpy subscriber
(`qos_profile_sensor_data`): color 15.0 / depth 15.2 Hz.

**Hinge re-derived (door closed):** `--phase both --one-take` dry run, 2 looks agree 0.4 cm, door normal
`[-1, -0.027, 0]`, hinge `[0.8929, 0.028, 0.2773]` r 34.3 cm (detector hinge edge 35.1-35.2). Written to
`~/.microwave_door.json` by hand (backup `~/.microwave_door.json.bak_2026-09-29`); door_z 0.143-0.415,
door_mid `[0.76, -0.178, 0.301]`. The open plan (direct grasp + 50 deg swing) also passed that dry run.

**`--route-over` push-close dry runs (container, `--hold-offset-x -0.152 --door-deg 90`) both refused, no motion:**
1. start J `[60.1, 35.1, -160.1, -135.7, 84.5, 96.3, -7.7]`: left leg at door top + 8 cm, step 11/24 J4 -143.6 + WRAP.
2. start J `[44.8, 34.6, -167.2, -134.3, 64.7, 87.5, -9.9]`: J4 fine (max -134.6), step 8/22 WRAP -- **J3**
   crosses -180 moving left (at hand y ~ -0.04). The planner's up/left/down/right-forward shape was right; only the
   J3 wrap blocks it, and a start with J3 +165..178 is not reachable from in front of this microwave without crossing.

**Hand demo of the whole close (user, holding the container):** `demos/microwave_manual_container_close_2026-09-29.csv`
(10 Hz, motion t 40.2-55.6 s, ~16 s, no back-and-forth errors). Hand orientation constant the whole time (container
level, quat `[0.726, 0.083, 0.682, -0.034]`). Door ~90 deg open at start, closed at the end.
| # | leg | end pos (x, y, z) | notes |
|---|-----|-------------------|-------|
| 0 | start | 0.548, -0.119, 0.286 | J `[28.4, 37.9, -174.4, -131.8, 42.8, 80.6, -6.0]` |
| 1 | up | 0.548, -0.117, 0.583 | +30 cm straight up = door top (0.415) + **17 cm** (planner used +8) |
| 2 | left | 0.548, 0.166, 0.582 | +28 cm in y, x/z constant. **J3 crossed the wrap here (-178.6 -> +177.4 at y ~ -0.06)** and ran on to +126; J5 went through 0 to -39 |
| 3 | down (+ a little left) | 0.555, 0.214, 0.339 | down 24 cm to z 0.339 (~mid door, 6 cm above the planner's 0.279), drifting 5 cm further left |
| 4 | right (sweep) | 0.528, -0.002, 0.340 | 22 cm right at constant z, pulled back ~3 cm in x -- the door-closing sweep |
| 5 | right + forward | 0.646, -0.133, 0.338 | +12 cm x, -13 cm y: finishes the arc and pushes shut. End J `[54.9, 42.8, 138.6, -112.3, 37.4, 44.9, 7.2]` |
- J4 stayed -107..-141 throughout (the -141 at the start of the sweep, y ~ 0.08) -- well inside the -144 guard.
- Relative to the planning hinge (real hinge shifted -15.2 cm in x = `[0.741, 0.028]`), the hand spirals in
  26 cm (sweep start) -> 21.5 cm (y 0) -> 19 cm (end); the planner's swing uses 28 -> 26 cm.
- Differences from `--route-over`: higher over the door (+17 vs +8 cm), comes down at a higher z (0.339), and the
  J3 wrap crossing on the left leg -- which the planner refuses -- is exactly what the demo did. Next: replay the
  demo's corners as Cartesian legs (with the wrap gate relaxed for J3 on that leg, or after a supervised
  `tools/wrap_test.py` run), or accept the hand-guided close for now.

## 2026-09-28 — one-take grasp + swing, door opened fully (80°)

Microwave on a lower surface this week (handle z ~0.21-0.26 instead of ~0.54) and moved/turned between
runs (handle y from -0.07 to -0.40, door normal from -6° to +11°). Everything below ran from the
`microwave-wip` worktree (`~/feeding-deployment-microwave`), with `PYTHONPATH=$PWD/src` prepended.

### What works now

```bash
export ARM_RPC_HOST=127.0.0.1 HANDLE_DEPTH_CORR=0.001 CAMERA_UPSIDE_DOWN=false
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_large_images.xml PYTHONPATH=$PWD/src:$PYTHONPATH
python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --phase both --one-take --swing-max --execute
```
- **`--one-take`**: plans and sim-checks everything BEFORE the first motion (grasp path, hinge, the whole
  swing from the *planned* grasp), then runs with no pauses: one motion into the grasp, close, one smooth
  swing. Opens the gripper first if it was left closed.
- **Direct grasp (default in one-take)**: the grasp pose is the ONLY waypoint -- a straight line from the
  view pose, hand rotating on the way (12-15° off the approach axis from the view poses used; no clipping
  seen). `--via-pregrasp` = old route (one brief stop at pre-grasp).
- **`--swing-max`**: tries 90° down to 45° in 5° steps and uses the largest whose whole arc passes the sim
  gates. Capped at 90° (`SWING_MAX_TRY_DEG`) because the smooth swing has no mid-trajectory abort.
  **Result: 80° (85/90 fail on J6 115.3-117.3°), 24 waypoints, 0.0 cm off, final J6 112°. The door opened
  fully -- user confirmed.**
- **Hinge carried over in the door's frame** (`_transfer_hinge`: last hinge's depth along the normal +
  distance along the face from the handle), so a turned microwave is handled; radius came out 34.3 cm
  every run and matched the detector's hinge edge (34.4-34.9 cm).

### Bugs / limits found and fixed

- **J4 soft limit ±147.8° is invisible to the sim.** A blended Cartesian grasp from a start with J4 -145°
  drove J4 into it and Kortex aborted (`JOINT_POSITION_LIMIT_REACHED`). Every sim check now gates
  `J4_GUARD_DEG = 144` (`microwave_common.py`), like J6. **Start from a view pose with J4 around -130° to
  -135°** (open the elbow up when placing it by hand; J4 barely changes if you only move the hand).
- **The Cartesian-trajectory RPC return is not the truth.** `kinova.py`'s wait was tripped by a stale
  END/ABORT notification: returned False instantly (arm still at the start), then Kortex ran the whole
  trajectory. `run_cartesian_trajectory()` (`microwave_common.py`) now judges by the arm's actual
  arrival (within 1 cm of the last waypoint, or still for 2 s). Used by grasp, swing and push-open.
- **Kortex slows/stops at every Cartesian waypoint**: the first waypoint gets zero blend (full stop), the
  rest at most 1 cm (`kinova.py:684-691`), and the last few taper to 4 cm/s. The 45° swing ran at a constant
  2.5 cm/s (0.79 s per 2 cm waypoint). Not fixed -- the blend cap lives in the arm driver (shared checkout);
  fix = run `arm_server.py` from the worktree with bigger blends / optimal blending.
- Door collision model height now follows the detected handle (`DOOR_Z_HANDLE_Z`), same extent as before.
- `MIN_Z` 0.25 -> 0.15 (user-approved), `PLAUSIBLE_Z` floor 0.25 -> 0.15, `PLAUSIBLE_Y` floor -0.40 -> -0.50.
- The arm driver + joint-state bridge wedge after ~1 day (dead Kortex session, `INVALID_USER_SESSION_ACCESS`,
  99% CPU, log grows to 100s of MB). Restart `arm_server.py`, `bulldog_bypass.py` AND `joint_state_bridge`
  (the bridge stays stuck too -> no tf -> detection waits 10 s per retry).
- Slow startup (1-2 min before `start EE`) = a loaded machine, not the script: 17 stuck `anydesk`
  processes (gdm, ~95% CPU each, need sudo) + other users' GPU jobs.

### Push-open (release, go round the door, push it further) -- NOT solved, not run on hardware

- Hand demo recorded (`microwave/demos/microwave_manual_push_open_2026-09-28.csv`, 10 Hz, `tools/record_demo.py`):
  out +y 3.6 cm off the handle -> back -x 11 cm -> right -y 26 cm -> turn hand in place +36.5° -> forward
  +x 5 cm -> push +y 31 cm along the door's arc. Encoded as `--push-style demo` (`DEMO_OPEN_POINTS`).
- **Blocked on this elbow configuration**: going round the free edge at the demo's width needs J4 at
  -146..-148° (the hand demo rode the soft limit); any tighter route puts a finger pad into the door model
  (0.1-1.6 cm, open or closed hand, back-offs 6-13 cm, even with the 2 cm clearance the user allowed for
  that leg). Options left: flip the elbow (J4 room near the base), or a route that goes OVER -- see below.
- Also written: `--push-style axes` (straight legs, straightening on the forward leg) -- fails the same way.
- Moot for now: the 80° pull opens the door fully on its own.

### New route (user's hand demo, 09-28, after the 80° open)

**Up, right, straighten (turn the hand to face in), down, left a little.** Going over the top avoids the
free-edge / J4 squeeze. User: "for this current setup should work well."

User's spec (09-28): numbers from the detection -- up to 5-10 cm above the detected microwave height,
right to about the middle of the microwave, straighten, then down.

### NEXT -- `--retreat-over` (steps 1-3 WRITTEN 09-28, not run on hardware; sim smoke-test only)
Code: detector attrs in `appliance_perception.py`; `door_push._plan_retreat_over` / `retreat_over`; flags
`--retreat-over` (with `--one-take`), `--phase retreat-over`, `--retreat-out` (0.05), `--retreat-above` (0.08).
Also: `plan_cartesian` re-solves a stalled IK step up to 3x (a single PyBullet pass stalled at ~2.3 cm on
the rotate-in-place "straighten" leg; re-solving converged). Synthetic sim poses: out/up/across/straighten
pass; failures seen were geometric (50 deg door: the "down" lands on the door; one odd 80 deg config: the
bracelet clipped the door top on "across") -- the real preflight from the real post-swing joints decides.
1. Detector (`appliance_perception.py`, add attributes only): in `detect_handle_and_placement`, next to
   `last_door_normal_base` (init ~l.636, set ~l.946 inside `if transform is not None`), transform
   `plane_points` to base and set `last_door_z_range_base` = (1st, 99th pct z) and `last_door_mid_base` =
   plane point whose `proj` is closest to (lo+hi)/2. Why: `top_of_appliance` is the image-down max, i.e.
   the door BOTTOM on this non-inverted camera (z 0.13-0.15, below the handle) -- don't use it.
2. Grasp script: average both looks, `save_door_geometry(door_z=, door_mid=)`; `add_door_model` uses
   `door["door_z"]` (+-2 cm) when present.
3. `--retreat-over` (after the one-take swing): release -> out ~5 cm along -approach FIRST (bar is fixed to
   the door at its top) -> up to door top + 0.08 -> right to door_mid y -> straighten (face -normal) ->
   down to handle height. Preflight from the predicted post-swing state like push-open (door model at the
   swing angle, 3 cm, IK/J4/J6/wrap); re-plan after the swing; run with `run_cartesian_trajectory`.
4. Test: door closed, view pose J4 ~-130, `--phase both --one-take --swing-max --retreat-over`, dry run first.

### Manual retreat-over demo, 09-28 ~18:10 (`demos/microwave_manual_retreat_over_2026-09-28.csv`, 10 Hz)
From the end of an executed `--one-take --swing-max` (90 deg, 27 wps, 0.0 cm off, J6 113.4; door file now
has door_z 0.124-0.379, door_mid y -0.314). User's rule: an immediate back-and-forth = an error, ignore it.
Clean corners (heading = approach direction in the xy plane, deg):
| # | leg | end pos (x, y, z) | heading | notes |
|---|-----|-------------------|---------|-------|
| 0 | release | 0.544, 0.048, 0.239 | -99.7 | gripper 0.814 -> 0.004 |
| 1 | up | 0.544, 0.048, 0.464 | -99.7 | **no "out" leg** -- straight up; z = door top + 8.5 cm |
| 2 | across | 0.414, -0.429, 0.464 | -99.7 | y to the closed-handle y (-0.42), NOT door_mid (-0.314); x pulled back 13 cm toward the base on the way (a y reversal -0.148 -> -0.091 at t 34 s = error) |
| 3 | straighten | ~0.414, -0.45, 0.464 | -99.7 -> -11.8 | 88 deg turn in place; **J1 crossed +-180 (176 -> -125)**, J7 147 -> 33 |
| 4 | down | 0.461, -0.357, 0.299 | -11.8 | diagonal: +4.7 cm x, +7 cm y while descending; stops 6 cm above grasp height |
| 5 | final | 0.434, -0.356, 0.284 | +4.4 | small back-and-forth adjustments (errors); rest joints -126 -43 26 -114 75 93 59 |
Differences from the planner (`_plan_retreat_over`): no out leg (the planner's out leg is what hit J6), across
ends at the handle y and further back in x, the straighten crosses the J1 wrap the planners refuse, down stops
~5 cm higher and drifts toward the middle.

### Evening 09-28 -- what ran and what changed
- **Executed** `--phase both --one-take --swing-max --execute` from a fresh view pose: 2 looks agreed, grasp 0.0 cm,
  gripper 0.814, swing-max picked **90 deg** (27 wps, 0.0 cm off, J6 113.4), door file got door_z/door_mid.
- `--retreat-over` dry runs all refused before motion; the causes, in order:
  1. The out leg from a 90-deg end drives J6 116+ (backing off the handle always raises J6 there; sim probe
     `tools/probe_j6_backoff.py`: up or a -10 deg hand turn LOWER it). The user's demo has no out leg.
  2. Bug fixed: `plan_cartesian`'s per-step jump wasn't wrapped (a free joint 179.8 -> -179.8 read as 359.6 deg).
  3. At door top + 8 cm the wrist (bracelet_link) passes 2.2-2.9 cm over the door model -> `--retreat-above 0.12` clears.
  4. Straighten then fails the +-180 WRAP gate at every swing angle 45-85 -- and the user's demo crossed J1's wrap
     on this leg too. Blocked until the wrap is tested on hardware or the start posture avoids it.
- `--swing-max` with `--retreat-over` now picks the largest angle from whose end the retreat ALSO passes.
- `plan_cartesian` step log now shows J4 and a WRAP flag.
- A latched "Emergency stop activated by user" (`CONTROL_MANUAL_STOP`) blocks every command: restart arm_server,
  re-run bulldog_bypass, AND restart joint_state_bridge (it stays stuck on the dead session -> tf incomplete ->
  detection waits 10 s per look).
- Files moved into this folder: `demos/` (all hand-demo CSV/JSON recordings, formerly `~/microwave_*`;
  `demos/camera_frames_2026-09-09/` is gitignored, 34 MB), `state_2026-09-28/` (door + last-grasp file snapshots;
  the live ones stay at `~/.microwave_door.json` / `~/.microwave_last_grasp.json`, the code reads those).

### Next
- Rewrite `_plan_retreat_over` to the demo's shape: no out leg; up; across to ~the handle's closed y while
  pulling back ~13 cm in x; straighten; down to ~6 cm above grasp height. Then deal with the J1 wrap on the
  straighten (hardware wrap test with `tools/wrap_test.py`, or a start posture that avoids it).

### State at end of 09-28 (afternoon -- superseded, see "Shutdown" below)
- Arm holding the handle, door ~80° open (J6 112 -- no scripted straight back-off from here).
- Running: arm_server, bulldog_bypass, joint_state_bridge (logs /tmp/microwave_logs/), camera, rsp,
  calibration_tf, stub_base, rqt_image_view. Button detector stopped. 17 stuck anydesk procs (need sudo).
- Nothing from 09-28 committed: all in worktree ~/feeding-deployment-microwave (branch microwave-wip,
  base bed37773)

### Shutdown, 09-28 evening
- Door open ~90 deg, arm at the demo's final pose, gripper open. Everything shut down (see commit message).
- Committed on `microwave-wip`.

## 2026-09-28 (evening) — push-close while holding a container (not yet run)

- New `--hold-offset-x` on push-close (default 0 = the validated 09-25 behaviour). The container
  sticks out ~6 in (15.2 cm) past the fingertips, so plan with **`--hold-offset-x -0.152`**: the door
  model (hinge, closed grasp/handle, door_mid) is shifted that far in arm-base x for planning only
  (door file untouched), so every leg, the swing and the push stop short by the container length.
  ```bash
  python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --door-deg 90 --hold-offset-x -0.152        # dry run
  python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --door-deg 90 --hold-offset-x -0.152 --execute
  ```
- **J4 blocks the left-at-hand-height approach** (dry runs 09-28, start J `[68.7, 34.4, -155.8, -127.9, 86.6,
  94.1, -9.3]`): every route failed ~8 cm into the move left, J4 -128 -> past the -144 guard. New
  **`--route-over`** (user's idea): up to door top + `--over-above` (8 cm), left to `--over-past` (5 cm) beyond the
  swing start, down to mid door height (mean of `door_z`; the swing and push then run there), then right + forward
  into the swing start. Swing/push unchanged. Not yet run on hardware.
- Caveats: the container itself is not in the collision model (neither was it before) — with the
  hand 15 cm back, it's the container, not the side of the hand, that will be nearest the door on the
  swing and push, so watch the swing's end and the torque-watched push. The shift is along x only; the
  closed-door normal is ~10° off x, so the offset along the push direction is ~15 cm with ~2.6 cm
  sideways. Check the dry run's printed radius/route before `--execute`.

### How to run it next time (container close)

1. Bring-up as usual (arm_server, stub_base, bulldog_bypass, joint_state_bridge, camera with the fastdds
   profile; logs in `/tmp/microwave_logs/`). Env for every microwave script, from the worktree root:
   ```bash
   source /opt/ros/humble/setup.bash
   export PYTHONPATH=$PWD/src:$PYTHONPATH ARM_RPC_HOST=127.0.0.1 HANDLE_DEPTH_CORR=0.001 CAMERA_UPSIDE_DOWN=false \
          FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_large_images.xml
   ```
   **`PYTHONPATH=$PWD/src` is required**: the pip-installed `feeding_deployment` points at `~/feeding-deployment`
   (the fridge branch), whose `AppliancePerception` has no `last_door_normal_base` -> the grasp script crashes.
   Keep the ROS entries (`PYTHONPATH=src` alone kills rclpy).
2. **Re-derive the hinge whenever the base or microwave moved** (door CLOSED, camera looking at it, gripper
   open or closed, no motion):
   ```bash
   python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --phase both --one-take   # dry run
   ```
   It prints the two looks, `door normal`, the carried-over `hinge (...)` and radius (want ~34-35 cm, matching
   the detector's `Hinge edge`). The dry run does NOT write the door file -- write it by hand with
   `microwave_common.save_door_geometry(hinge=, closed_grasp_pos=, closed_normal=, closed_handle=, door_z=, door_mid=)`.
   If the handle is out of grasp reach the script exits at `GATE FAILED at grasp` BEFORE the hinge line; then
   compute it with `_transfer_hinge(prev_door_file, handle_vertical_corrected, door_normal)` (done that way on
   09-28 after the base moved: hinge `[0.9143, -0.1629, 0.2476]`, 96 cm from the base -- only reachable with
   the container offset). Back up `~/.microwave_door.json` first.
3. Open the door ~90 deg, arm in the container-hold pose, then dry run -> check -> execute:
   ```bash
   python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --door-deg 90 --hold-offset-x -0.152 --route-over
   python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --door-deg 90 --hold-offset-x -0.152 --route-over --execute
   ```
   The only `--route-over` dry run so far refused at step 1 (bracelet 0.3 cm from the door model) because the
   user was moving the setup and the hinge was stale -- **the route has never passed a dry run yet**. If the
   container hangs low, raise `--over-above`.

### Other 09-28 evening findings

- **Kinova faulted twice** in the evening: first `INVALID_USER_SESSION_ACCESS` on every `get_state` (dead Kortex
  session), later `REACH_JOINT_ANGLES` feedback `JOINT_POSITION_LIMIT_REACHED` then `ACTION_ABORT` /
  `ROBOT_IN_FAULT` from another client's joint moves (not the push-close dry runs -- they never command).
  Recovery both times: restart arm_server + bulldog_bypass + joint_state_bridge (camera can stay up).
- **Don't `pkill -f <name>` from a Bash tool call whose own command line contains `<name>`** -- it kills the
  calling shell (exit 144) partway through. Kill by PID.
- `ArmInterfaceClient` has no `get_arm_state` (only the server-side `ArmInterface` does); check the
  arm_server log for `ROBOT_IN_FAULT` / `ACTION_ABORT` instead.
- `demos/microwave_manual_container_close_2026-09-28.csv`: 36 s recorded while the user started a hand demo
  of the container close and then stopped ("I see the problem") -- partial, NOT a full close demo.
- Door file at end of session: the 09-28 post-base-move geometry above (`door_open_deg` 90, `open_sign` -1).
  The user then moved the setup again, so **re-derive before the next run**.

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
  the session scratchpad; CSVs in `demos/microwave_manual_*_2026-09-25.csv`, push-close orientation in
  `demos/microwave_push_close_orientation_2026-09-25.json`).

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
