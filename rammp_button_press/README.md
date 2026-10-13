# `rammp_button_press` — the microwave button press on RAMMP's arm driver

The autonomous button press from `feeding_deployment/button_press/` (branch `button-task`; read
its `README.md` for how detection and the panel frame work), ported from the Cornell arm RPC
(`ArmInterfaceClient` / `arm_server.py`) to RAMMP's `kinova_gen3_node`, running natively next
to sheppy (RAMMP's hybrid workflow). An `ament_python` package, self-contained: it needs
neither `feeding_deployment` nor its RPC stack installed.

**Status (2026-10-12):** builds; 35 offline tests pass; full dry runs against a mock driver +
synthetic camera (detect → panel frame → every leg gated). On hardware: session checks, live
detection and a dry run pass; `PREPRESS_EE_OFFSET_M` re-measured on this rig (hand-placed
pre-press spot from a 33.5 cm view), and an attended 1.0 cm push along the normal from that spot
clicked start_30s (offset measured against timer_clock, shifted by the dome layout). **The full `press_button --execute` sequence has not run yet.**

## Setup (once per box)

```bash
# the package lives in the feeding-deployment repo; ~/ros2_ws/src holds a symlink
ln -s ~/feeding-deployment/rammp_button_press ~/ros2_ws/src/rammp_button_press
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select rammp_button_press    # NOT --symlink-install (see below); rebuild after edits (~2 s)
```

- `rammp-interfaces-ros2` must be built in the same workspace at the tag the arm image uses
  (v1.1.0 for `kinova-gen3-ros2:1.1.0`). Don't modify it.
- Python deps come from a venv created with `--system-site-packages` (so `rclpy`/`cv_bridge`
  come from `/opt/ros`): numpy < 2, scipy, opencv, pybullet. On the Jetson this is
  `~/feeding-deployment/.venv`. No system-wide pip.
- Scripts get `#!/usr/bin/env python3` (`setup.cfg`), so `ros2 run` uses the activated venv.
  `--symlink-install` ignores that and writes `/usr/bin/python3`, which has no pybullet.
- The hand-eye calibration (`end_effector_link → camera_color_optical_frame`, easy_handeye2
  `.calib`) for **this** camera mount: default
  `~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib`, override with `BUTTON_CALIB`.

## A session

Every terminal: `source ~/feeding-deployment/rammp_button_press/sheppy/env.sh`

```bash
# 1. bring-up (no motion: the driver holds the arm where it is)
cd ~/feeding-deployment/rammp_button_press/sheppy
sheppy up --manifest sheppy-manifest.yaml arm-camera
sheppy status                                         # arm, arm_tf, camera_calib_tf, wrist_camera running

# 2. session-start checks: gripper + arm state, camera, tf
ros2 topic echo --once /control_status                # estopped false; arbitration / owner
ros2 topic echo --once --qos-reliability best_effort /gripper_state
ros2 topic hz /wrist_camera/aligned_depth_to_color/image_raw   # ~15 Hz
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# 3. panel in view, camera >= 22 cm from it (watch the domes)
ros2 run rammp_button_press view_detection --target timer_clock

# 4. dry run (default): detect, plan and gate every leg, move nothing
ros2 run rammp_button_press press_button --target timer_clock

# 5. ONLY with the go-ahead for this specific move, someone at the e-stop:
ros2 run rammp_button_press press_button --target timer_clock --execute

sheppy down
```

Releasing the arm for hand-guiding or teleop is `sheppy up ... camera-only`, which stops the
driver. **Never hand-guide or teleop with the driver up**: `WRONG_SERVOING_MODE` crash (2026-10-07).
`sheppy up ... arm-camera` brings it back, and it starts holding wherever the arm is.

### Re-measuring the pre-press spot (required on this rig)

`PREPRESS_EE_OFFSET_M` / `PREPRESS_EE_QUAT_PANEL` in `press_button.py` were measured on
rchi-cpu-5, in that rig's *detected* panel frame, so they absorb its hand-eye bias. With this
URDF's fingertip (tool frame + 5.96 cm) they put the fingertip **7 cm into the panel**. The
10-07 microwave port saw the same rig's grasp constants land 9.8 cm too deep on this Jetson.
`check_fingertip` refuses any pre-press spot whose fingertip is less than 1 cm in front of the
fitted plane. To re-measure:

```bash
ros2 run rammp_button_press measure_prepress_offset --target timer_clock
# detects from ~25 cm, then asks you to: sheppy -> camera-only, hand-guide the LEFT fingertip
# 2 cm in front of the button, sheppy -> arm-camera, Enter. Prints the two constants to paste.
```

## As a RAMMP module: the `/press_button` action server

`press_server` serves `/press_button` (`rammp_button_press_interfaces/action/PressButton`, its
own small interface package next to this one -- `rammp-interfaces-ros2` stays untouched):

    goal     target, presses, press_in_m (0 = default), execute, no_return
    result   success, moved, message ("done" / why it stopped; "HOLDING HERE" = holding), log_file
    feedback stage -- one line per step, exactly what press_button prints

Same code path as the script (`press_button.press_button` per goal), with the camera, arm
interface and kinematics kept loaded (~1 s per dry-run goal). **The arm moves only if the server
was started with `allow_execute:=true` and the goal asks to execute**; otherwise the goal runs as a
dry run and the result says so. Cancel = stop the trajectory in flight and hold. One goal at a
time. Sheppy runs it as node `button_press`:

```bash
sheppy up --manifest sheppy-manifest.yaml arm-camera           # server: dry runs only
sheppy up --manifest sheppy-manifest.yaml arm-camera-execute   # server may move the arm -- e-stop manned
ros2 run rammp_button_press press_client --target start_30s [--presses 3] [--execute]   # Ctrl-C = cancel
```

Don't run the `press_button` script while the server is executing a goal: they would both
command the arm. `rammp-alternative.yaml` (package root) is the fragment for adding the server to
a RAMMP manifest, with what it needs from the rest of the stack (driver, cuRobo planner, wrist
camera topics, a calibrated camera tf -- per rig, with a re-measured pre-press offset).

Build: `rammp_button_press_interfaces` and `rammp_curobo_interfaces` (link
`~/RAMMP-CuRobo/rammp_curobo_interfaces` into `~/ros2_ws/src`; v1.1.0) must be built in the same
workspace: `colcon build --packages-select rammp_curobo_interfaces rammp_button_press_interfaces rammp_button_press`.

## What changed from `button-task`

| Old (`ArmInterfaceClient`) | Now (`RammpArm`, `rammp_arm.py`) |
|---|---|
| `get_state()["position"]` | `/joint_states` (best effort; `joint_1..7`, wrapped to ±π) |
| `get_state()["ee_pos"]` (Kortex tool pose) | `/ee_state` + 0.120 m along the flange z (`/ee_state` is the flange, `end_effector_link`) |
| `get_state()["gripper_pos"]` | `/gripper_state.position` (0 open … 1 closed) |
| `get_arm_state()` (`SERVOING_READY`) | `/control_status` (e-stop, arbitration, owner) + `/diagnostics` (`Arm` task) |
| `get_speed()` low/medium | none. Each leg is timed here: min-jerk, peak 6 cm/s tool, 0.35 rad/s per joint |
| `CartesianTrajectoryCommand` (Kortex's own IK) | `execute_joint_trajectory`: **the exact joint solutions the sim gates checked**, 20 ms samples, driver-side `path_tolerance` 0.15 rad |
| full `FeedingDeploymentPyBulletSimulator` for IK | `urdf/gen3_tool_frame_kinematic.urdf` alone (same chain, same `tool_frame`; FK/IK identical to the full sim, tested); loads in 10 ms instead of 2 s |
| camera `/camera/...`, frame `arm_base_link` | `/wrist_camera/...` (`--camera-ns`), frame `base_link` |
| `feeding_deployment.ros2.calibration_tf` | `calibration_tf` in this package (RAMMP publishes no wrist-camera tf) |

Unchanged: `dome_pattern.py`, `panel_frame.py`, `geometry.py` are copies (only the import path
differs; `test/test_sync_with_feeding_deployment.py` fails if they drift). The detection, panel
frame, staging and planning in `press_button.py`, and every existing gate: IK error ≤ 5 mm,
orientation ≤ 1°, joint step ≤ 10°, reach ≤ 0.91 m, z band 0.12–0.75 m, wrist turn ≤ 45° (the 40 cm move cap was removed 2026-10-12),
press ≤ 3 cm, final position within 1 cm (approach and press; staging, bail and back-to-start within 5 cm since 2026-10-12).

**New refusals** (none of them loosen anything):
- **Joint limits:** J2/J4/J6 must stay 2° inside the driver URDF's limits. Kortex used to do
  the IK; now our solutions go to the driver as they are. Raw PyBullet IK does return
  out-of-range J4.
- **Fingertip clearance:** see above.
- **Tool-frame check at preflight:** sim FK at the live joints must match `/ee_state` + 0.12 m
  to within 5 mm. If the driver ever reports a different frame, the pre-press offset would be
  silently wrong, so the run refuses.
- **Start drift:** a leg is refused if the arm moved more than 0.5° since it was planned.
- **Arbitration:** if `arbitration_mode` is `enforced`, `--acquire-control` acquires a token
  only when nobody owns the arm. `/acquire_control` *seizes*, so it never takes the arm from
  another owner. The token is released at exit.

Never used: `go_to_ee_pose` (cuRobo-planned, not a straight line) and gripper commands.

## Open questions (for Swapnil) and what we found

1. **Does RAMMP publish a wrist-camera tf?** No calibrated one. The December manifest runs the
   camera and the driver only, so we publish `arm_tf` (robot_state_publisher on the driver
   URDF) and `camera_calib_tf` ourselves.
2. **Which tool frame does `/ee_state` report?** The flange (`end_effector_link`), checked
   2026-10-07: equal to `tf2_echo base_link end_effector_link`. The driver doc calls it "tool
   pose". The preflight check above keeps verifying it.
3. **Is arbitration enforced?** `disabled` in RAMMP's own fragment and in our manifest.
   Enforced at HERL? `--acquire-control` covers it, but only when no one owns the arm.
4. **Own rammp-org repo/module later?** Open. The package is self-contained for that. A
   `rammp-alternative.yaml` fragment would follow once the press runs as a node, not a
   one-shot script.

## Still to check on hardware

- The camera on the wrist: the local manifest says D435i (serial 140122079014, 2026-10-07),
  RAMMP's December manifest says D405. The detector's ~22 cm minimum range and depth behaviour
  were tuned on rchi-cpu-5's camera.
- The calibration on this box is Capybara's (copied 2026-10-07). Touch-checked to ~2 cm for
  the door handle, never for buttons.
- The first motion: `--presses 0 --no-return` from 23 cm, attended, before any press.
