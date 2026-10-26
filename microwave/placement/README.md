# Microwave container placement

> **Motion is on sheppy now: `place_move.py`** (built on Part 1, `microwave/xyz_move.py`). It carries the held
> container to the placement point with cuRobo `HOLD_LEVEL` and executes every leg in **joint impedance**
> (`sheppy_arm.IMPEDANCE`, the driver's path tolerance acting as a force stop) through `SheppyArm`.
> The placement point is a **placeholder** (`place_move.placement_point`, from `--xyz` / `place_move.yaml`)
> until Part 2 (microwave perception) supplies it.
> `python3 -u microwave/placement/place_move.py --config microwave/placement/place_move.yaml [--execute]`
> after `source microwave/sheppy/env.sh`; tests: `tests/test_place_move.py`.
> **Part 2 perception (where the container goes): `perceive_placement.py`** -- camera frame(s) -> SAM 3
> "inside of open microwave" mask (`cavity_segmentation.py`) -> valid-depth cavity cloud in arm_base_link
> -> floor + cavity (`floor_target.py`, reusing `cavity_perception.estimate_cavity`/`merge_cavities`) ->
> floor contact / container centre / tool centre + a `place_move.yaml` that Part 1 reads. Camera-agnostic
> (`rgbd_source.py`: any colour + aligned depth + CameraInfo with its calibration on tf; topics in
> `perception_config.yaml`). Never moves the robot.
> `python3 -u microwave/placement/perceive_placement.py --live --frames 3` or `--recorded <npz|capture dir>`
> (`--mask` reuses a saved SAM 3 mask); tests: `tests/test_placement_perception_part2.py`.
> **Status 2026-10-26 (on the arm):** Part 1 position mode hit a hand-placed target to 0.2 mm; live SAM 3 ->
> floor (z 0.126, tilt 0.9 deg) -> the arm moved in (level, straight Cartesian leg) to the floor centre with ~5 cm
> under the hand, and back out to park. **Still to do:**
> 1. Target the CONTAINER centre, not the tool point: per the user, with the container centred in the grasp the
>    wrist camera sits 9.75 (units to confirm -- in or cm) behind the container centre; derive `grasp_offset` from that.
> 2. Impedance is broken on this arm: default gains sagged the tool 4.3 cm, stiffer gains (kq 250/70) dropped it
>    5.7 cm -- not tuning (gravity/torque-offset model or the mode switch?). `place_move.py` is impedance-only: unusable until fixed.
> 3. Final insertion leg: use the straight Cartesian path (`sheppy_arm.cartesian_knots`); cuRobo HOLD_LEVEL bows ~2 cm.
>    Level the gripper while moving out (in place at a folded pose trips the J4 guard). Fold both into `place_move.py`.
> 4. Perception: door-open check (SAM 3 segments the interior through a closed glass door); prompt "inside of
>    microwave" for close views (0.85 vs 0.50); calibrate `cloud.depth_corr_m` (measure the real 5 cm gap); measure the container.
> 5. Camera calibration: the floor reads 4.5 deg tilted with the gripper rolled +90 (0.9 deg at roll 0) -- a few deg of
>    hand-eye rotation error; perceive at roll ~0 until recalibrated.
> The motion and `bringup.sh` described below (`ArmInterfaceClient`, `arm_server`, the old
> `compliant_controller` impedance lowering, PyBullet-planned legs) are the pre-sheppy version: do not use
> them to move the arm. Their perception/cavity code is what Part 2 can build on.

Puts the OXO box the gripper is holding into the **open** microwave: real-time detection,
cavity + floor from depth, obstacle-aware level insertion, impedance lowering, release, retract.
**Perception run live on the rig 2026-10-20 (look-in works, see below); no placement motion into the microwave yet.** History of the SAM 3 version:
`../NOTES.md` (2026-10-02 section) and the 10-04 status in git (`75f508dc`).

## Pipeline

| Step | What | Code |
|---|---|---|
| Detect | **SAM 3** (default; `scripts/detect_handle_sam3.py`'s model, prompt "inside of open microwave", ~1.8 s/frame, 0.91 on the 10-20 rig frame) or YOLO26s (`detector.backend: yolo`), on the image turned upright from tf (camera mounted rolled 90 deg); >= 3 detections must agree with their median box (IoU 0.6, 25 px), 20 s timeout; SAM 3's box is grown 40 px | `microwave_detector.py` |
| Cloud | ROI pixels with aligned depth -> camera points -> `arm_base_link` (tf2) -> hand + held box removed -> 8 mm voxels -> statistical outlier removal | `point_cloud.py` |
| Cavity | level sheets from a gravity-aligned height histogram (floor, turntable, ceiling), walls by RANSAC; back wall = a wall with floor running up to it; insertion axis = -back normal; floor fit checked level (<= 6 deg -- also catches a wrong camera roll), flat, big, below the camera; opening, side walls, top. >= 3 looks must agree. No door JSON | `cavity_perception.py` |
| Target | hand re-oriented to aim the box along the axis and level it; box footprint centred between the walls, near end 3 cm inside, >= 4 cm from the back; support height measured under the footprint (turntable); refuses if something stands there, the box is too wide/long/tall or out of reach | `cavity_perception.placement_target` |
| Plan | PyBullet Gen3: existing `solve_ik` + orientation-error, joint-limit (URDF + J2/J4/J6 guards), wrap, jump checks; collision of links + held box vs cavity slabs and 3 cm scene voxels (open door, front, counter); box tilt <= 2 deg on every insertion step. Impedance: J6 pulled to -67.6 on the approach, held there inside | `placement_planner.py` |
| Execute | `JointCommand` steps (convergence-checked); lowering by task impedance (`switch_to_task_compliant_mode` + `CartesianCommand`, contact = lag + stall, abort on drift / tracking / timeout / RPC error, always leaves compliant mode) or planned position steps | `placement_workflow.py`, `impedance_lowering.py` |
| Release | re-planned from where the hand is: open (verified), lift 5 mm, straight back out (placed box is an obstacle), park if that leg passes | `placement_workflow.plan_release` |

**Impedance and J6.** `compliant_controller.py` task mode uses a 6-DOF model with J6 fixed at
**-67.6 deg** (`hack_gen3_robotiq_2f_85.urdf`; checked offline: FK exact there, 53 cm off at +67.6).
So impedance is only entered with the real J6 within 3 deg of that. The planner gets there by
self-motion on the approach -- possible only from the **J6 < 0 wrist branch**. The 09-29 container
hold (J6 +80) is on the other branch: the plan then refuses and prints the flipped joints
(J5+180, -J6, J7+180, same hand pose) to hold the box in -- flip the wrist BEFORE picking the box
up. `--lowering position` is the fallback (planned steps ending 1 cm above the support).

## Looking without a set pose, and with the container held (`placement_workflow.perceive_auto`)

The wrist camera sits 16.8 cm behind the tool frame looking along the approach (measured 10-20), so a
held container is right in front of it. Every `perceive` / `plan` / `place` looks in this order:
1. **From where the arm is.** While holding, the look is only trusted if the held box could see >= 40% of
   the cavity it measured (projected container vs cavity) -- a mostly-blocked view gave a bogus cavity.
2. **A saved earlier look** (`~/.microwave_place_look.json`, every successful look is saved; 1 h), checked
   surface by surface against what is visible now (floor/back/left/right each >= 50% of its points within
   1.5 cm): a microwave pushed 4 cm is rejected (back wall 0%). Best moment to save one: right after the
   door is opened, empty gripper.
3. **The arm finds a view** (only `place`/`all --execute`, after its own `go`): candidate viewpoints in front
   of the opening with the container kept level (<= 15 deg tilt) -- low and nose-down, so the cavity shows
   above the container -- scored by predicted visibility (incl. grazing angles), IK/joint guards, reach;
   checked moves (straight, else joint space); points fused across views; next views re-planned from the
   latest estimate until both side walls are seen. Dry runs list the views instead of moving.
The move from the look pose to pre-insert falls back to a checked joint-space move if the straight line
hits a joint guard. `view_planner.py`; closed-loop tests with the held box rendered at the gripper.

## Commands (repo root; nothing moves without `--execute` + typing `go`)

```bash
microwave/placement/bringup.sh                 # arm_server, joint bridge, stub base, bypass, speed low, rsp, calib tf, camera
source ~/microwave_place_logs/bringup/env.sh
python3 microwave/placement/tools/check_camera_roll.py               # which roll the published calibration needs
python3 microwave/placement/tools/rotate_camera_calibration.py --roll-deg <that>   # writes ..._roll<deg>.calib
microwave/placement/bringup.sh stop && microwave/placement/bringup.sh            # picks up the rolled calib

P=microwave/placement/real_gen3_ros2_place_container_microwave.py
python3 -u $P perceive                                           # detection + cavity, overlays in ~/microwave_place_logs/<t>/
python3 -u $P plan --container-drop <m>                          # + every leg in sim
python3 -u $P place --container-drop <m> --execute --stop-after pre-insert   # then insert, then lower
python3 -u $P release [--execute]
python3 -u $P plan --replay ~/microwave_place_logs/<t>           # offline re-plan of a saved run
python3 -u $P place --no-container --container-drop 0.09 --execute --stop-after insert   # empty-gripper rehearsal
python3 -u microwave/placement/tools/move_look_pose.py --back 0.06 --up 0.12 --tilt 15 [--execute]   # +tilt = nose down
# ROS 2 node (live overlay, clouds, markers, services) -- dry-run unless allow_execute:=true
python3 -u microwave/placement/microwave_place_node.py --ros-args -p container_drop:=<m>
ros2 run rqt_image_view rqt_image_view /microwave_place/overlay
ros2 service call /microwave_place/plan_place std_srvs/srv/Trigger
# tests (~1 min)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_microwave_placement_perception.py tests/test_microwave_placement_planning.py
```

Tunables: `placement_config.py` (defaults, documented), overrides in `placement_config.yaml`.
Offline scenes: `synthetic_scene.py` (ray-cast microwave, door, turntable, counter; any camera roll).

## Hardware checklist (stop at the first failure; e-stop in hand from step 4)

1. **Perception only** (gripper empty, door open, camera looking in): `bringup.sh`, `check_camera_roll.py`
   must say OK (or write + publish the rolled calibration and re-run until it does). `perceive` 3x:
   overlay box on the microwave, floor dots on the floor, yellow box on the walls; looks agree;
   floor tilt < 2 deg. Fail = refusal text + overlays in the log dir.
2. **Frames/geometry**: touch the floor and the back wall with the gripper (teleop) and compare with
   `cavity.json` (floor_z, back). > 1 cm off along the look direction -> set `cloud.depth_corr_m`.
   > 1-2 cm otherwise -> the calibration translation (roll pivot) needs a real easy_handeye2 run.
3. **Container + hold**: measure `container.drop/width/height/far_past_tool` into the YAML. Hold the
   box on the J6 < 0 branch (the flipped joints a refused `plan` prints). `plan`: all legs pass,
   clearance numbers sane, RViz `/microwave_place/markers` box sits in the cavity.
4. **Supervised approach**: `place --execute --stop-after pre-insert` -- box level, aimed into the opening,
   ~8 cm in front. Then `--stop-after insert` (re-plans from there) -- box enters without touching,
   stops 4 cm above the floor. Stop if anything rubs or tilts.
5. **Impedance lowering**: `place --execute` from a fresh plan. Expect "contact after ~4 cm"; box resting,
   arm not sagging. Abort criteria are automatic (drift 2 cm, tracking 6 cm, 20 s). If compliant mode
   misbehaves: e-stop, restart arm_server + bulldog_bypass + joint_state_bridge; use `--lowering position`.
6. **Release**: `release` (dry) then `release --execute`: gripper opens, lifts 5 mm, backs out; park may be
   refused (J4 guard, known) -- return by hand.
7. **End to end**: `all --container-drop <m> --execute` (two `go` prompts).

## Open items that need the rig

- Which roll sign the remount is (`check_camera_roll.py` decides) and the pivot translation (~cm).
- Container dimensions/drop, and whether the box + hand fit this microwave's opening height (in sim a
  20 cm cavity is too low for an 11 cm box held 8.5 cm below the tool with 4 + 3 cm clearances).
- Whether YOLO finds the OPEN microwave from the look pose (it misreads it as bus/train sometimes).
- Task-impedance gains are the feeding ones, never run on rchi-cpu-5; the 1 s gravity-compensation
  phase in `switch_out_of_compliant_mode` happens with the box resting on the floor.
- Camera frame-age gate assumes camera stamps on the system clock (1 s max age).

## 2026-10-20 first rig session (empty gripper)

- Camera calibration already carries the 90 deg roll (YOLO/SAM 3 only detect with the tf-derived upright turn;
  floor fits 0.4-2.5 deg). No re-calibration needed for the roll.
- Fixes from real frames: the back wall slopes ~12 deg (allowed up to 25); the opening comes from the side walls'
  front ends when the camera can't see the front of the floor; the insertion axis from the side walls; back wall
  measured at its near side. Real frame kept as a test: `tests/data/microwave_lookin_2026-10-20.npz`.
- Arm fell when powered off (no brakes) -- park it / support it first. After the fall J4 sat at -151.7 (past
  -147.8): every command refused until pushed back by hand.
- A held round tupperware covers the camera's view of the cavity: look BEFORE grasping (planned: plan from a saved
  look, `--cavity-from`), or look over it.
- Refusals seen (all correct, nothing moved): reach 0.932 > 0.915; box top vs a cavity top under-measured from a
  low view (top from the highest visible wall points -- use the front face above the opening); path to park blocked
  by the J4 guard from the look pose (return by hand).
- `move_look_pose.py`'s first version tilted the hand UP for +tilt; fixed and sign-checked.
