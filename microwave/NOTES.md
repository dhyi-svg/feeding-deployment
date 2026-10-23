# Microwave open / close — notes (rchi-cpu-5, 2026-09-23)

Working notes for the scripts in this folder. Newest learnings first within each section.


## 2026-10-23 -- fridge: push-open option and close (code only, NOT run on hardware)

- **Close:** `real_gen3_ros2_close_microwave.py --fridge`, which is `fridge_push.close`, then `door_push.push_close`
  with fridge options. It prompts to move the wheelchair back `CHAIR_BACK_M` (0.30) and plans against
  `~/.fridge_door.json` shifted by that much. Route: back out in front of the open door's free edge, left, forward
  (the 4th of push_close's routes, `behind=True`). It swings until the door model is at `CLOSE_TO_DEG` (45,
  `stop_deg`), with no push (`--push-max 0`). Sim check against the 10-22 detection: passes from 2 of 3 start poses
  (back to x 0.34, left to y +0.06, 57-deg swing, back out 20 cm). `FRIDGE_TESTING.md`.
- **Push-open:** `--fridge --push-open` pulls 50 deg, then `demo_replay.replay` runs: off the handle, along the open
  door past its free edge, round it, then the microwave demo's keypoints 4-5. **In sim this refuses on the 10-22
  setup:** the wrist goes into the door model on the leg into the gap behind the door, at every pull (30-50 deg;
  at 20/25 deg it gets round the edge, but a finger comes 0.8/1.5 cm from the door in the narrow gap),
  orientation and clearance tried. The door points almost straight at the arm base once open, so the arm can't get
  round behind it at that distance.
- Shared code changes, with the microwave unchanged (offline planner regression byte-identical before/after):
  - The door model reads optional per-door geometry from the door file (`microwave_common.door_dims`:
    `face_behind_grasp`, `door_t`, `door_past_handle`, `body_depth`). The fridge open now writes them, plus `door_z`
    from the handle height, `hinge` = the measured pivot, and `open_sign` via `record_door_angle(door_file=)`.
  - `demo_replay.run` is split into `run` + `replay`.
  - `push_close(door_file=, door_shift=, behind=, stop_deg=)`.
- Fridge push-open pull default is now 35 deg (`PUSH_OPEN_PULL_DEG`; microwave stays 50): in the sim sweep 35 came
  closest (finger +0.1 cm from the door model; 30/40 -0.8 cm, 50 -1.3 cm). None clear the 2 cm yet.
- To measure: `DOOR_WIDTH_M` (placeholder 0.45, measure at the door's top), `DOOR_DEPTH_M` (front face to the
  innermost point of the door's inside, bins included; placeholder = the 1.25 in thickness), the door top/bottom
  vs the handle.

## 2026-10-22 (17:23) -- first SUCCESSFUL fridge open (`--fridge`, user-confirmed)

- `real_gen3_ros2_grasp_and_swing_microwave.py --fridge --execute` (depth detector over the whole image, measured
  hinge from `fridge_constants.py`, `HANDLE_CENTRE_TO_FACE_M` 0.053): grasp + 90-deg swing, door file 90.0 deg.
  Handle (0.562, -0.426, 0.636), grasp (0.514, -0.430, 0.636), door yaw 4.5 deg. Ends holding the handle (no release).
- Record: `~/microwave_detections/20261022_172303_fridge_open_SUCCESS.json` (+ that run's two detection frames).
- Earlier runs the same day missed: the white glossy handle's front face gives NO depth, only its edges, so the handle
  cluster is an edge strip (17:01-17:18: right edge; 17:03: the monitor beside the fridge, from the whole-image search).

## 2026-10-21 (evening) -- impedance code removed; full open ran again

- Compliant-swing branch + `SWING_COMPLIANT`/`SWING_STIFF_DEG`/`SWING_ARRIVE_TOL` deleted from the grasp+swing script
  (user: "just remove impedance for now"); `impedance=` plumbing in sheppy_arm/microwave_common left, unused.
- `real_gen3_ros2_grasp_and_swing_microwave.py --execute` ran end to end: tilt 10, grasp 0.0 cm, gripper 0.80, stiff
  50-deg swing 0.0 cm (path bulge 2.0 cm, fit r 38.0 vs 36.1, hinge off 2.2 cm -- the 10-07 pattern again), door file
  ~54 deg, push-open all 5 legs, ended 0.0 cm from the last keypoint.
- Then the route-over push-close (`--door-deg 90`): the default `--push-radius 0.28` was refused (swing step 7 J4 -144.1,
  just past the 144 guard -- hand comes closest to the base in front of the hinge); `--push-radius 0.24
  --push-radius-end 0.22` passed (J4 max -137.3) and RAN: route over, 22-step swing, push to the full 45 mm with no
  torque contact (max change 1.78 Nm < 3.0), backed out 20 cm. Latch not confirmed by the script.
- Door did NOT fully close. Cause: the microwave had MOVED -- re-detect for the reopen found handle x +4.6 cm, hinge
  x +6.3 cm, yaw -0.9 -> -3.7 deg vs the first open's detection (the door file the close used). The push ran out of
  its 45 mm budget against a stale door; user also said the route-over and push points were a bit low (next try
  `--over-above 0.13 --push-z 0.22`, dry run passed). Push-close should re-detect the door before closing.
- Reopen (from a view pose near the base) ran clean: tilt 0, grasp 0.0 cm, swing 0.0 cm, push-open 0.0 cm.
- **Container close (user teleop-grasped the tupperware, gripper 0.98) RAN:** `--phase push-close --route-over
  --keep-orientation --door-deg 90 --hold-offset-x -0.152 --push-radius 0.17 --push-radius-end 0.15 --over-above 0.13
  --push-z 0.22 --push-max 0.08 --execute`. Radius 0.24 and 0.20 refused on J4 (-146.4 / -144.5: the 15 cm container
  offset pulls the hand toward the base); 0.17 passed (J4 max -140.7), back-out shortened to 6 cm by J4. Torque
  contact at push 3 (+15 mm, 3.36 Nm) -> stopped. Gripper still 0.98 (holding) at the end.
  User: "almost there, a little far from closing", and the container CLIPPED the door going round (it is not in the
  collision model). Next: `--over-above 0.18` + `--push-contact-nm 6` (keep pushing through the first contact).
- One run crashed the arm driver mid route-over: WRONG_SERVOING_MODE, and the gripper read OPEN (0.009) after --
  neither our code nor the driver commands the gripper there; the arm web app was open in Firefox (suspect).
- **Container close with `--over-above 0.18 --push-contact-nm 6` (rest as above) RAN clean:** torque 3.4 Nm at +20 mm
  (first contact), dip, then 12.1 / 17.8 Nm at +30 / +35 mm -> stopped. Gripper held (0.99).
  Door CLOSED (user) -- latch at +30 mm = exactly the detected closed face (swing ends 3 cm short), so the initial
  detection was right. User wanted it to stop sooner -> push now bounded by the detection: `--push-max` default
  = `--push-stop-short` + `--push-preload` (3.5 cm, 7 x 5 mm), and the push stops on ONE step above 2 x
  `--push-contact-nm` (that run would have stopped at +30 mm, 12.1 Nm). Not yet run on hardware.
- **Ran it:** fresh open (detect from the view pose; handle x +1.7 cm vs the reopen -- the close pushes it back), user
  re-gripped the tupperware, close with no `--push-max`: contact 4.6 Nm at +10 mm, 13.0 Nm at +20 mm -> stopped on the
  single-jump rule. **Door LATCHED** (user). Close command that worked:
  `real_gen3_ros2_close_microwave.py --phase push-close --route-over --keep-orientation --door-deg 90 --hold-offset-x
  -0.152 --push-radius 0.17 --push-radius-end 0.15 --over-above 0.18 --push-z 0.22 --push-contact-nm 6 --execute`
- **Grasp tilt: 14 deg nose-down first (TEMPORARY).** User hand-placed the grasp they want (perpendicular to the door,
  bar centred in the pads): `demos/user_grasp_pose_2026-10-21.json` -- depth identical to the code's grasp (x 0.770),
  pad centre ~1 cm up / 1 cm left (noise), but 13.6 deg nose-down because THIS RIG'S ARM BASE IS TILTED. So
  `GRASP_PITCHES_DEG` now tries 14 first. **Real deployment: base will be level -> restore (0, 10, 17, 25, 30, 35).**
- **Smooth push:** on sheppy the close push is now ONE slow continuous move (`PUSH_SPEED_SCALE` 0.25, ~1.2 cm/s)
  with the J1-J4 torque change polled ~100 Hz via the new `SheppyArm.move_joints(monitor=...)`, cancelled (driver
  stops + holds, PREEMPTED) once it passes 2 x `--push-contact-nm`. Free-air cancel test passed (stopped, next goal
  accepted, back to 0.01 cm). Filmed run: open (tilt 14 chosen) + container close; push stopped at ~+15 mm, 12.0 Nm.

## 2026-10-21 -- impedance swing SCRAPPED; straight-in approach

- **Approach now lines up on the door normal, then goes straight in** (`PREGRASP_STANDOFFS` 25/20/15/10 cm, the largest
  that passes the gates; `plan_approach`). Ran once on hardware at 25 cm: grasp tracking 0.0 cm. Before, it was one
  diagonal line from the view pose (user: "approaches on an angle").
- **Impedance (compliant) swing: scrapped -- keep `SWING_COMPLIANT = False`.** Run 1 (path_tol 0.08 rad): aborted
  PATH_TOLERANCE_VIOLATED right after the stiff 10 deg, door ~16 deg. Run 2 (path_tol 0.30): got to ~51 deg but the
  gripper read 1.00 after (0.74 on the handle) -> the handle slipped out of the loose (`GRASP_FORCE` 0.25) grip; the arm
  made a wild turn, user e-stopped, arm dropped off the network. Driver's `final_error` came back all zeros (no help).
- Detection from the close-up view (hand ~23 cm from the door) is bad: YOLO says "train", hinge 5 cm off. From the
  park pose the microwave fills the frame -> no box at all. Detect from further back (all 4 edges in view).
- `--release`'s 20 cm back-off hits the J6 guard at a ~16 deg door; 12 cm + straight to park worked.
- The push-close in `real_gen3_ros2_close_microwave.py --phase push-close --route-over --keep-orientation --door-deg 90`
  ran on hardware after a push-open (torque contact at +10 mm); latch not confirmed. The door file's angle is the
  post-swing one, so after a push-open pass `--door-deg` by hand.
- Standing rule (user): the open is always the 50-deg swing + push-open.

## 2026-10-07 (late) -- STOPPED HERE: full open + close ran on hardware via `door_task.py`

**State at shutdown:** `sheppy down`, nothing running. The arm was left at the close's END pose (hand beside the
microwave's left side at the closed door, gripper open) -- move it away by teleop before anything else next time.

- `microwave/door_task.py open|close|full|retract [--execute]` (run from the REPO ROOT -- sim config path is relative):
  open = open_task (grasp + 50-deg swing) + push-open demo replay; close = close demo replay + retract (back off 15 cm,
  park, gripper untouched). Ran today: grasp + swing OK; push-open needed `--trust-demo 1` (lift-off leg starts next
  to the door face; the door model says 1.9 cm); close demo ran to 0.1 cm of its last keypoint (did the door latch? --
  ask the user); RETRACT refused: back-off step 1 puts the bracelet 2.9 cm from the microwave body (< 3 cm) -- the
  replay reached the close end in a DIFFERENT IK branch than the demo (J3 69 / J4 -34 / J7 -115 vs demo 163 / -2 / 77).
  Not yet in door_task: trust settings to match what ran (push-open trust 1), retract back-off clearance.
- **Hinge correction is suspect:** with `HINGE_*` applied the stiff swing still bulged 2.0 cm and the fit moved out
  another 2.3 cm (r 38.6) -> the 2 cm is likely the arm yielding under drag, not hinge error; circle fits of a STIFF
  path are biased. Consider removing `HINGE_BEHIND_FACE`/`HINGE_PAST_EDGE` (set 0) until measured another way.
- Door file now: `hinge` = detected edge (what demos + door model use), `hinge_pivot` = corrected pivot (swing).
- sklearn removed (open3d DBSCAN, identical clusters on a test cloud, ~3x faster); not yet run on a live detection.
- Next: decide IK branch handling for replays (seed from the demo's recorded joints, so the replay follows the
  demo's arm configuration), retract fix, view-pose preset (start J4 was past 144 twice), lateral grasp check,
  update the architecture artifact (https://claude.ai/artifact/Fpk7ieA9L26sCttRDViCfN), trim door_push.py.

## 2026-10-07 (afternoon) -- push-open from the user's demo WORKED; close demo recorded

- **Full open on hardware:** grasp -> stiff 50-deg swing -> `push_open_demo.py --trust-demo 3 --execute`: release, up,
  over to the right of the microwave, down, left into the opening, push -- ended 0.0 cm from the demo's last keypoint.
  Now generalised: `demo_replay.py encode <demo>` stores a keypoint demo in DOOR coordinates (along the door as a
  fraction of hinge-handle distance, off the face in m, height from door top / grasp, orientation vs door facing);
  `demo_replay.py run <demo>.door.json [--trust-demo N] [--execute]` maps it onto today's door, sim-gates, replays.
  `demos/push_open_2026-10-07.door.json` = the open demo with +3 cm along-door at kp2/kp3 (as run). The 50-deg door
  model puts the REAL demo poses 2-7 cm inside the door from kp3 on -> `--trust-demo 3` (body-only checks from leg 3).
- **Swing:** impedance dropped (tripped its 0.08 rad stop at the latch even with a stiff first 10 deg). Stiff swing
  measured path bulged 2.0 cm; circle fit -> real pivot 1.6 cm behind the face, 2.1 cm past the detected edge (r 36.0 vs
  33.7) -> `HINGE_BEHIND_FACE` / `HINGE_PAST_EDGE` applied to the detected hinge (not yet run with it). Grip force on the
  handle 0.25 (`GRASP_FORCE`, was 0.5) so the bar can turn in the fingers (microwave leaned forward). Detector now aims at
  the MIDDLE of the handle bar (was 4 cm from the cluster's max camera y = the bar's BOTTOM end on this upright camera),
  `VERTICAL_CORR` 0. Grasp still landed right of the bar twice (calibration, few cm, both signs seen) -> lateral touch
  check still to do. Detection frames saved to `~/microwave_detections/`.
- **Close demo** `demos/push_close_2026-10-07.json` (+ `_door.json`): kp1 START holding a container, wrist rotated 90
  (could be the opposite way), up 24 cm, LEFT over the open door to past the hinge (kp3 y +0.23, kp4 y +0.41 -- user:
  the real move left will be LESS far left), down to door height beside the microwave's left side (kp5). The push that
  shuts the door is NOT demonstrated yet. Large wrist joint changes between kp2-kp4 (J5/J7 flip) -- watch the replay.
- To do: encode + replay the close (with less-left adjust, roll-180 option), finish the push-to-shut, lateral grasp
  correction, door model from the measured hinge so --trust-demo isn't needed, drop sklearn (open3d has DBSCAN),
  trim door_push.py (old push-open planners superseded), update the architecture artifact
  (https://claude.ai/artifact/Fpk7ieA9L26sCttRDViCfN).

## 2026-10-07 END OF SESSION -- PICK UP HERE

**State at shutdown:** `sheppy down` (all nodes + sheppyd stopped, no containers), arm powered off by the user.
Door left OPEN ~50 deg, gripper open/empty. Everything below is UNCOMMITTED on `microwave-wip`.

**Bring-up next time (Jetson, sheppy-only):**
```bash
source ~/feeding-deployment/microwave/sheppy/env.sh          # every terminal
cd microwave/sheppy && sheppy up --manifest sheppy-manifest.yaml real-arm-camera   # arm + tf + calib + camera + view
#   ... camera-only = release the arm for teleop (NEVER teleop / web-app with the arm node up -> WRONG_SERVOING_MODE crash)
cd ~/feeding-deployment && python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --swing-deg 50   # dry run
```

**Done today:** sheppy hybrid bring-up (host ROS 2 + ~/ros2_ws + .venv with Jetson torch), `SheppyArm` port (all
scripts), Capybara calibration, touch-checked grasp (`GRIP_EXT` 0.088, `VERTICAL_CORR` 0.046), per-run grasp tilt
search (low handle), full grasp + 75-deg swing + release on hardware, then grasp + 50-deg swing on hardware, new park
pose, push-close fingers-left default, keypoint demo of the user's push-open.

**Next, in order:**
1. **Test the compliant (impedance) swing** -- written, NOT run: `SWING_COMPLIANT=True` in the open script ->
   `SheppyArm.IMPEDANCE` (kq 80x4/30x3, zeta 0.5, 0.08 rad lag = force stop). Prints how far the hand strayed from the
   commanded arc + a circle-fit hinge vs the detected one (fit ~1.7 cm accurate on a 50-deg arc). Start with the door
   CLOSED (close it by hand/teleop first -- it's open now), `--swing-deg 50`, e-stop in hand; watch the microwave.
   If impedance misbehaves set `SWING_COMPLIANT=False` (stiff, as run today).
2. **Encode the user's push-open demo** `demos/push_open_2026-10-07.json` (+ `_door.json` = door state then; door
   ~50 deg, hinge (0.826, 0.087), grasp at handle z 0.18, LEVEL grasp). Keypoints (tool xyz):
   0 end of swing holding (0.511,-0.040,0.182) -> 1 open + straight UP 26 cm (0.511,-0.040,0.447) -> 2 high, forward +
   RIGHT past the microwave's right side (0.760,-0.477,0.449) -> 3 DOWN to door height, wrist turning (0.716,-0.461,0.248)
   -> 4 LEFT 18 cm into the opening (0.665,-0.282,0.246) -> 5 the PUSH: left 11 / back 13 cm + ~60 deg wrist turn
   (0.537,-0.170,0.245). Express them relative to the detected hinge / door top / door angle (so they adapt to an
   unknown microwave position/height), sim-gate every leg (`tools/sim_retreat_over.py` style), dry run, then real.
   Note: it goes much wider/higher than the sim attempts (right of the microwave body, not just the door edge).
3. **Build the `microwave_task` sheppy node** (agreed: route first, then node): long-running, keeps sim/YOLO/CUDA
   warm (each script run now spends ~20-40 s on startup + re-planning even after a dry run), services
   /microwave/open|release|close (+dry-run flag), latched `/microwave/door_state` + per-run JSON records in
   `microwave/state/` (replaces the global `~/.microwave_door.json`, which carries stale keys between runs).
4. Push-close for the 50-deg door (fingers left) -- sim-check; at 75 deg nothing planned (door points at the base).
5. **The diagram** (today's goal 4, not done): the full open pipeline under sheppy -- nodes (camera, arm, tf,
   calib, view) -> detect -> tilt search -> grasp -> (compliant) swing -> release -> over-the-door -> push -> park,
   plus the door geometry (hinge, radius, door angle) it works in.
6. Minor: rclpy spin-thread traceback/segfault at every script exit; 9% RT overruns in the arm node (no faults).

## 2026-10-07 — goals for today

1. Finish the sheppy implementation for this code (started in `0138d9d8`: detector hinge, local `door_arc_waypoints`).
2. Try it on the real arm.
3. Test a complete door opening.
4. Make a diagram (of the open pipeline / door geometry).

**From now on everything runs under sheppy** (user, 10-07). The `arm_server.py` / `stub_base_server.py` /
`bulldog_bypass.py` RPC stack and the hand-started ROS 2 nodes in every section below are the OLD bring-up --
history, not procedure. Under sheppy the arm is `kinova_gen3_node` (container from `~/kinova-gen3-ros2`, fragment
`rammp-alternative.yaml`): `/joint_states`, `/ee_state`, `/gripper_state`, `/setpoint/*` streams, and the
`execute_joint_trajectory` / `go_to_ee_pose` / ... actions (the `go_to_*` ones need the cuRobo planner container
too). The scripts here still talk to `ArmInterfaceClient` -- porting them onto that interface is the sheppy work.

Order: read the arm + RealSense state first, then work out how to use sheppy (`~/.local/bin/sheppy`,
`sheppyd`, `~/rammp-deployments/december_2026/sheppy-manifest.yaml`).

**Sheppy bring-up (10-07, this box: Jetson, `eno1` 192.168.1.11, arm 192.168.1.10).** Docs:
https://rammp-org.github.io/sheppy/ and https://rammp-org.github.io/development/ (hybrid workflow = stable modules as
containers under sheppy, our code native from `~/ros2_ws`). `~/rammp-deployments/december_2026` (= origin/main) has
cameras only -- no arm node, no profiles (the docs' `real-arm-cameras` profile doesn't exist yet). So a **local copy**
lives in `microwave/sheppy/`: `sheppy-manifest.yaml` (wrist_camera D435i / mock, arm, planner), `compose.yaml` (arm +
cuRobo, published ghcr images), `profiles/{real-arm-camera,real-arm-camera-planner,mock}.yaml`.
```bash
cd microwave/sheppy
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # sheppyd hands its env to nodes; set before `up`
sheppy up --manifest sheppy-manifest.yaml real-arm-camera
sheppy status; sheppy logs -n 25 arm
# no ROS on the host yet -> read topics from inside a container:
docker exec sheppy-arm bash -c 'source /opt/ros/humble/setup.bash; source /module_ws/install/setup.bash;
  ros2 topic echo --once --qos-reliability best_effort /joint_states'
sheppy down
```
- Images: `ghcr.io/rammp-org/kinova-gen3-ros2:1.1.0`, `rammp-suite-software/realsense:main`, `rammp-curobo:1.1.0`.
  All CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` baked in); set it + `ROS_DOMAIN_ID` in every container's
  `environment:` anyway (containers don't inherit sheppyd's env).
- The realsense image's ENTRYPOINT already runs `realsense2_camera_node` with `/etc/rammp/realsense/realsense.yaml`:
  settings go in sheppy `params:`, `command:` only renames (`[--ros-args, -r, __node:=wrist_camera]`). The december
  manifest's `command: ros2 launch ... rs_launch.py` would be appended to that entrypoint -- looks wrong there.
- Plain `docker run` fails on this kernel (iptables `raw` table missing); host networking (what sheppy uses) is fine.
- **First read (10-07, arm untouched):** arm node `up (real)`, 0 faults (178 RT overruns in the first 5 s, max
  cycle 172 ms -- watch). Joints (deg) `[0.0, -11.5, -180.0, -151.7, 0.1, -23.2, 90.1]` (folded/retract pose), EE
  `(0.165, 0.001, 0.295)`, gripper 0.0087 (open), not e-stopped, arbitration disabled. **J4 -2.6477 rad is 0.7 deg
  inside its +-2.66 URDF limit** -- the arm repo's runbook (09-01) saw a run abort with J4 at -2.660; don't stream the
  measured pose back from here. Camera: colour 15.0 Hz, aligned depth ~14 Hz, D435i serial 140122079014, USB 3.2.
- **Host ROS 2 for the hybrid workflow (10-07):** installed `ros-humble-ros-base ros-dev-tools rmw-cyclonedds-cpp
  cv-bridge tf2-ros tf2-tools tf2-geometry-msgs robot-state-publisher xacro control-msgs trajectory-msgs
  diagnostic-msgs` (apt, `ros2-apt-source`), rosdep init/update. `~/ros2_ws/src/rammp-interfaces-ros2` -> symlink to
  `~/rammp-interfaces-ros2` at **v1.1.0** (= what the arm image is built against; the realsense image has 1.0.0, only
  sensor_msgs used from it). `colcon build --symlink-install`: 4 packages, 34 s. From a host shell with
  `source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` the containers' topics/actions are all visible (`/joint_states`, `/ee_state`,
  `/gripper_state`, `/setpoint/*`, `/wrist_camera/...` at 15 Hz, `/execute_joint_trajectory`, `/go_to_*`).
- **TF under sheppy** (local manifest, both real profiles): `arm_tf` = robot_state_publisher on
  `~/kinova-gen3-driver/models/gen3_7dof_2f85.urdf` with `gen3_` sed-stripped (the driver publishes `joint_1..7`, so
  the URDF must match; stripped, its frames are `base_link .. end_effector_link`, Kinova's standard flange), and
  `camera_calib_tf` = `feeding_deployment.ros2.calibration_tf` (easy_handeye2 file, `end_effector_link ->
  camera_color_optical_frame` + `base_link -> arm_base_link` alias). Checked: `tf2_echo base_link end_effector_link`
  = `/ee_state` exactly (0.165, 0.001, 0.295). The camera publishes `camera_color_optical_frame` (frame names don't
  follow the `__node:=wrist_camera` rename). **The calibration file isn't on this box yet** (copy from rchi-cpu-5).
  sheppyd hands its env to `process` nodes: start it (`sheppy up`) from a shell with ROS + ~/ros2_ws sourced, and
  `PYTHONPATH=src:$PYTHONPATH` (prepend, don't replace -- replacing lost rclpy).
- **Every terminal: `source ~/feeding-deployment/microwave/sheppy/env.sh`** (ROS + ~/ros2_ws + DDS vars + `.venv` +
  `PYTHONPATH=src` + the CUDA lib path, cd to repo root). Start `sheppy up` from such a terminal too.
- **Python deps: `~/feeding-deployment/.venv`** (`uv venv --system-site-packages`, so rclpy/cv_bridge come from
  /opt/ros): numpy 1.26 (<2 for cv_bridge), scipy, scikit-learn, open3d 0.20, pybullet (compiles ~15 min on arm64),
  pybullet_helpers (prpl-mono git), ultralytics 8.4. **torch gotchas (JetPack 6.2 = R36.4.4, CUDA 12.6 driver):**
  PyPI's torch is a cu130 build -> `cuda False` ("driver too old"); with `--extra-index-url pypi.org` uv takes PyPI's
  copy of the same version even with the Jetson index first. Install NVIDIA's wheels BY URL from
  `https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/{torch,torchvision}/` (torch 2.11.0, torchvision 0.26.0). They need
  `libcupti.so.12` + `libcudss.so.0` (not in this JetPack's apt set): pip `nvidia-cuda-cupti-cu12~=12.6` and
  `nvidia-cudss-cu12==0.8.0.10`, then REMOVE every other pip `nvidia-*` (cuDSS drags in desktop cuBLAS 12.9 ->
  `CUBLAS_STATUS_ALLOC_FAILED`) so cuBLAS/cuDNN come from JetPack (/usr/local/cuda/lib64, cuDNN 9.3); env.sh puts
  the two pip lib dirs on `LD_LIBRARY_PATH`. Verified: matmul / conv / torchvision NMS on the Orin GPU.
- **Calibration = Capybara's** (the old Jetson rig's easy_handeye2 file, pasted by the user 10-07; translation
  (0.016, 0.043, 0.053), ~180 deg about z) in `~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib`. Checked
  via tf2: image-down = base -z (-0.91), view +x tilted ~23 deg down, image-right = base -y -> orientation right for
  this upright camera (YOLO: 'microwave' 0.91 unflipped, 'train' flipped -> no flip, as handle_detect assumes).
  Live camera window: sheppy node `camera_view` (`tools/camera_view.py`, OpenCV on DISPLAY :1).
  **Releasing the arm for teleop = `sheppy up ... camera-only`** (stops `arm`/`arm_tf`/`camera_calib_tf`); back with
  `... real-arm-camera` (the node starts holding wherever the arm is).
- **Detection + touch check (10-07, `tools/touch_check.py detect|compare`):** microwave ~74 cm out, handle
  (0.737, -0.138, 0.139..0.148), hinge (0.757, +0.189) -> radius ~33 cm, door face z 0.04-0.32; two views agree to
  ~1 cm, looks within a view to 0.1 cm. `PLAUSIBLE_Z` min 0.15 -> 0.05 (microwave sits ~4 cm above the arm base here).
  Touch 1 with the rchi-cpu-5 constants: planned grasp **9.8 cm too deep** (sideways +0.1, vertical +2.1) -- those
  constants compensated that calib's depth bias; with this one the detected handle is where it is (touched tool
  4.8 cm in front of it = fingertips ~1.4 cm past the bar's centre). -> **`GRIP_EXT` -0.010 -> +0.088,
  `VERTICAL_CORR` 0.025 -> 0.046.** Touch 2 (different view): **2.3 cm total** (approach -0.4, sideways -1.7,
  vertical -1.5). Orientation diffs (14-18 deg) were the hand placement pitched ~17 deg nose-down; finger axis/roll
  matched, yaw within 5 deg -- plan's level approach kept.
- **Low handle -> J6 caps the swing; fixed by a per-run grasp TILT search (10-07).** With the handle at z ~0.14
  above the arm base a LEVEL grasp only swings 25-35 deg (J6 passes its 115 guard ~30 deg in; null-space IK only
  got ~40). Pitching the hand nose-down about the finger axis (fingers stay horizontal across the vertical bar --
  the user's touches were ~17 nose-down) frees the wrist: 0/10 -> <45, 17 -> 70, **25 -> 75 (J6 ends 92)**. Now
  `GRASP_PITCHES_DEG = (0, 10, 17, 25, 30, 35)`: plan grasp + largest swing for each, keep the smallest tilt reaching
  the largest swing (a high handle should keep 0). Tool point unchanged by the tilt (placed along the level
  approach). The microwave height is unknown in deployment (usually low) -- don't hard-code height-specific values.
  Push-open (`door_push`, `tools/sim_push_open.py`) was tried first after a 25-deg pull: every route failed on J6
  starting from the cramped end-of-pull wrist (arc also needs `--push-in` for small openings) -- moot with the tilt.
- **SheppyArm Cartesian = straight lines now** (`cartesian_knots`): each segment from the current pose cut into
  <= 2 cm / 5 deg steps (= plan_cartesian) before IK, refused if a step jumps > 20 deg or breaks the J4/J6 guards;
  timing counts rotation (1 rad = 15 cm) and never exceeds JOINT_SPEED. (The grasp is ONE waypoint; Kortex drew
  the straight line itself, the first SheppyArm version would have joint-interpolated it.) Offline execution sim
  (grasp + 75-deg swing as sent): 0.1 mm off, max J6 92, max J4 139. Tested on hardware: `tools/sheppy_motion_test.py
  cartesian` (+3 cm x and back) landed 0.3 mm / 0.02 deg -- before the densify change.
- **First full open under sheppy (10-07):** grasp (auto tilt 25 deg) + 75 deg swing WORKED on hardware (user ran it),
  then `--release --execute` (back off 20 cm, toward base, park) done -- but the park pose isn't the one the user
  wants; they're teleoping the new end pose (-> `park_pose.json`). User notes: the release sounded loud (fine,
  ignore); the run felt slow -> `SheppyArm` peaks raised to JOINT_SPEED 0.5 rad/s, CART_SPEED 9 cm/s, and
  `execute_joint_plan` now sends a whole leg as ONE smooth trajectory under SheppyArm (was one stop-and-go move per
  2 cm step). Exit of every script: harmless rclpy spin-thread traceback / segfault at shutdown -- to fix.
- **New park pose (10-07, teleoped by the user, `park_pose.json` v4):** joints [34.6, 37.7, -156.5, -128.7, 59.7,
  78.3, 74.4] deg, tool (0.484, -0.248, 0.246) -- low, in front of the microwave on the right (v3 = the high 09-25 one,
  kept as `previous`). Release to it not yet dry-run.
- **Target flow (user, 10-07): grasp -> swing -> release -> PUSH the door the rest of the way open -> park.** Ideally
  the arm finishes the open with a push. `door_push.push_open` + `tools/sim_push_open.py` exist (sim only); after the
  25-deg pull every route failed on J6 from the cramped end-of-pull wrist -- re-test from the end of the now-75-deg
  tilted swing (or after release + back-off, from a fresh posture, not chained from the pull's end).
- **Push-open route work (10-07, sim only).** User: retreat/push go UP AND OVER the door, not back toward the base;
  diagonals OK but don't head into the microwave; turn AFTER coming down. Findings (`tools/sim_retreat_over.py`, ad-hoc
  sims): at 60-75 deg the open door points at the arm base, so any descent on its right side puts the forearm over the
  door. **Swing 50-55 deg + out 5 cm, up to door top +12 cm, across to 16 cm right of the free edge, down, level the
  hand -> passes (>= 3.5 cm).** The push from there doesn't yet: facing in (+x) the wrist trails into the free edge;
  fingers pointing right (user's idea: push with the WRIST, mirror of push-close; camera side faces up then) the
  bracelet passes the free edge at 0.3-1.5 cm on the forward leg. Next: user demonstrates the push by teleop after a
  real 50-deg open (`tools/record_demo.py`), encode it door-relative. Open script: `--swing-deg N` caps the swing.
- **Push-close (10-07): fingers point LEFT by default now** (`--keep-orientation` = old as-is behaviour). From a
  75-deg door + the new park pose (hand on the door's RIGHT side) nothing plans: fingers-left puts the wrist inside the
  door at every swing start (door points at the base); as-is needs `--route-over` (passes up/over/down, 3.7 cm) but the
  swing then fails J4 (-145, guard 144) at every push z/radius tried. Closed by teleop. Re-check push-close for the
  50-deg door before the next close.
- **OPEN CONCERN (user, 10-07): the swing arc isn't accurate and could break the door.** Nothing measures the door
  during the swing: the arc is a circle about the DETECTED hinge, the hand is position-controlled, and the only
  stop is the driver's 0.15 rad tracking tolerance (a stiff arm pulling a door off its hinge won't lag that much).
  09-29 it dragged the whole microwave at 75-80 deg. To do: watch joint efforts during the swing (/joint_states
  effort, like push-close's contact rule) and stop -- or re-centre -- when they rise; or swing in IMPEDANCE mode
  (`control_mode 1` in ExecuteJointTrajectory) so the hand yields to the door's real arc; fit the hinge from the
  first few cm of travel.
- **Port: `microwave/sheppy_arm.py` (`SheppyArm`)** replaces `ArmInterfaceClient` in every script/tool (same
  `get_state()` / `execute_command()`; `ARM_RPC_HOST` gone). Joint moves = one `execute_joint_trajectory` goal;
  `CartesianTrajectoryCommand` = the poses IK'd with the scripts' own `solve_ik` (chained from the real joints) then one
  joint-trajectory goal, i.e. the arm follows the sim-checked joint path, not Kortex's Cartesian interpolation;
  gripper = `/setpoint/gripper` at 50 Hz until it settles (the 2F-85 latches it). Dense 20 ms points with velocities
  (-> the driver's cubic Hermite, not linear), min-jerk timing, peak 0.35 rad/s joint / 6 cm/s Cartesian,
  `path_tolerance` 0.15 rad per joint (driver-side tracking abort), Ctrl-C cancels the goal.
  **`ee_pos` = Kinova `tool_frame` = flange + 0.120 m along flange z** (what Kortex's tool pose was and what
  `FINGERTIP_PAST_TOOL` is measured from); `/ee_state` is the flange. Camera topics now `/wrist_camera/...`.
  Verified 10-07: sim `tool_frame` FK at the real joints == `/ee_state` + 0.12 to 0.1 mm, same quaternion -- the
  sim the scripts plan in, the old Kortex tool frame and the new driver agree.
- **First SheppyArm motion (10-07, attended):** `tools/sheppy_motion_test.py joint`: J6 +5 deg and back, both
  SUCCESSFUL, landed within 0.01 deg. `... gripper`: close 0.009 -> 0.969, open -> 0.009. (`cartesian` not run yet.)
  Driver RT telemetry: faults 0, but ~9% overrun cycles (20k / 225k) and n ~900/s, i.e. the 1 kHz loop runs late
  -- didn't affect tracking; watch it (host isolcpus=11/nohz_full=11 is set, core 11 is the driver's `--cpu`).

## 2026-09-29 (late) — open script cleaned up: current commands

**The open script's flags are gone.** `--phase`, `--one-take`, `--swing-max`, `--fast`, `--smooth`, `--hinge`,
`--target-angle-deg`, `--steps`, `--via-pregrasp`, `--swing-send-every`, `--push-open-after`, `--retreat-over`
(and the push-open args) no longer exist; commands with them in the dated sections below are history. The
default run is what `--phase both --one-take --swing-max` used to do:

```bash
source /opt/ros/humble/setup.bash
export ARM_RPC_HOST=127.0.0.1 CAMERA_UPSIDE_DOWN=false FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_large_images.xml
# from the repo root (the sim config path is relative)
python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py                      # dry run: detect + plan, no motion
python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --execute            # grasp + swing
python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --release --execute  # open gripper, back off 20 cm, park
```
- Detect (2 looks within 3 cm) → plan everything before motion (straight-line grasp squared to the door face,
  hinge carried over from `~/.microwave_door.json`, largest swing 75 → 45 deg passing the sim gates) → one
  blended grasp → close → re-check the arc from the real joints → one blended swing (every 4th waypoint sent).
- **Behaviour changes:** if the post-grasp re-check fails it now stops holding the handle (no stop-and-go
  fallback); no `--hinge` override -- edit the door file (`save_door_geometry(hinge=...)`) when the hinge is stale;
  `HANDLE_DEPTH_CORR` no longer needs exporting.
- **Detection is now `handle_detect.py`**, not `AppliancePerception` + the fake-GroundingDINO YOLO adapter: the
  handle half of `detect_handle_and_placement` copied as-is (YOLO box → depth points → RANSAC door plane →
  protruding DBSCAN cluster scored by verticality × elongation → tf2), returning handle, door normal, door_z,
  door_mid. Offline check on 19 of `demos/camera_frames_2026-09-09`: new vs old ≤ 0.85 cm handle / 0.20° normal,
  within the old detector's own run-to-run spread (≤ 1.04 cm, RANSAC). Not yet run live.
- **`PYTHONPATH=$PWD/src` is no longer needed for the open script** -- `appliance_perception.py` was the only
  `src/` difference between this branch and `detect-fridge-handle`, and the open script no longer imports it.
- Close (`real_gen3_ros2_close_microwave.py`, push-close) and `door_push.py` are unchanged.

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
   export ARM_RPC_HOST=127.0.0.1 CAMERA_UPSIDE_DOWN=false \
          FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_large_images.xml
   ```
   (09-29 late: the old `PYTHONPATH=$PWD/src` requirement is gone -- the open script uses `handle_detect.py`,
   not `AppliancePerception`. If you do set PYTHONPATH, keep the ROS entries: `PYTHONPATH=src` alone kills rclpy.)
2. **Re-derive the hinge whenever the base or microwave moved** (door CLOSED, camera looking at it, gripper
   open or closed, no motion):
   ```bash
   python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py   # dry run (was --phase both --one-take)
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
| `real_gen3_ros2_grasp_and_swing_microwave.py` | Open task (09-29 late): no flags = detect, plan, grasp, swing (dry run unless `--execute`); `--release` = release, back off, park. |
| `handle_detect.py` | Handle + door-face detection (YOLO box → depth plane → protruding cluster → tf2), copied from `AppliancePerception.detect_handle_and_placement`. |
| `real_gen3_ros2_close_microwave.py` | Close task: `--phase view` / `regrasp` / `swing` / `push` / `release` / **`push-close`** (no grasp, 09-25). |
| `door_push.py` | push-close (validated 09-25) and push-open / retreat-over (sim only, no longer reachable from the open script): side-of-gripper door pushes, torque-watched final push. |
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
`last_door_normal_base` on each detection (10 added lines, nothing else changed; 09-29 late: no longer used by
the open script, which has its own `handle_detect.py`). Deleted:
`scripts/real_gen3_ros2_yolo_grasp_microwave.py` (fully contained in the grasp-and-swing script).

## Commands that worked today

(09-23. The open script's flags below no longer exist -- see the 09-29 (late) section at the top.)

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
