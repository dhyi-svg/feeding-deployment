# Microwave container placement

Puts the OXO box the gripper is holding into the **open** microwave: real-time detection,
cavity + floor from depth, obstacle-aware level insertion, impedance lowering, release, retract.
**Software complete and tested offline; NOT yet run on the robot.** History of the SAM 3 version:
`../NOTES.md` (2026-10-02 section) and the 10-04 status in git (`75f508dc`).

## Pipeline

| Step | What | Code |
|---|---|---|
| Detect | YOLO26s (COCO `microwave`, fallback bus/train/... as `handle_detect.py`) on the image turned upright from tf (camera is mounted rolled 90 deg); >= 3 detections must agree with their median box (IoU 0.6, 25 px), 20 s timeout | `microwave_detector.py` |
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
