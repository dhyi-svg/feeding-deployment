# Tonight: button detector on real hardware

Goal: **camera → detector → live topic → launched by sheppy, verified by eye.**
No arm motion. Everything below is look-only.

Time: ~2h if the reference transfers, ~2.5h if it has to be rebuilt.

---

## BEFORE YOU START — two questions that can kill the whole evening

Answer these first. Both are one message to a colleague, and guessing wrong
costs hours.

1. **Which machine has the Comfee microwave AND ROS 2?**
   Notes say the Comfee is on **Pachirisu**, which runs **ROS 1 Noetic**
   (RoboStack). **Sheppy is ROS 2.** The Jetson has ROS 2 Humble working.
   If the microwave and the ROS 2 stack are on different boxes, tonight's goal
   is not reachable as written — say so early rather than fighting it at 11pm.

2. **Which workspace?** RAMMP setup says `~/ros2_ws`. Your CLAUDE.md says
   `~/ros2_ws` is Demo-Software's and **off-limits on the shared Jetson**.
   Pick another path if that still holds. Below uses `$WS`.

```bash
export WS=~/rammp_ws          # change if you have a different workspace
export REF=~/wrist_ref        # where the camera-built reference will live
```

---

## STEP 0 — safety + state (5 min)

```bash
# whatever your usual arm state read is; confirm the gripper is not holding the door
```
`gripper_pos` **cannot** tell you whether a grasp succeeded — it saturates at
~1.0 with or without something thin between the fingers. It *can* tell you if
the arm is clamped onto something. If it is, clear that before anything else.

Do **not** start `arm_server.py` / `bulldog_bypass.py` tonight. Nothing in this
plan moves the arm, so nothing needs motion unlocked. Leaving it locked is a
free layer of safety.

**Checkpoint:** arm is idle and motion is NOT unlocked.

---

## STEP 1 — camera up (10 min)

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

`align_depth` matters: the node reads depth at the *colour* pixel, so the two
must be aligned or the depth will be sampled from the wrong place.

**Checkpoint:** both topics publishing. Colour ~30Hz, aligned depth ~14Hz is
normal on this rig.

> USB note: this camera has dropped its whole xHCI controller 4+ times in your
> logs. If it vanishes, that is the known fault, not something you did.

---

## STEP 2 — THE CRITICAL TEST: does the reference transfer? (10 min)

**Do this before writing or building anything.** The reference was built from
4K phone video. The RealSense is a different sensor at a fraction of the
resolution. SIFT *may* bridge that; it is an assumption, not a measurement.

Point the wrist camera at the microwave panel, roughly the framing the arm
would use, then:

```bash
python3 check_reference_match.py \
    --ref <repo>/scripts/scratch/button_eval/reference \
    --topic /camera/color/image_raw \
    --out /tmp/ref_match.png
```

It tries the frame as-is **and** rotated 180° (inverted mount) and prints
inliers for each.

### → LOOK AT `/tmp/ref_match.png`. Do not trust the inlier number alone.

Inliers say *the panel matched*. They do **not** say the marker is on the right
button. The marker must sit on **+30SEC** (bottom-right of the five), not on
STOP/ECO beside it.

| Result | Do this |
|---|---|
| MATCH, marker on +30SEC | skip to STEP 4 |
| MATCH, marker on the wrong button | go to STEP 3 — the reference is fine but the mark is wrong |
| NO MATCH | go to STEP 3 |

**Capture for review:** the printed output + `/tmp/ref_match.png`.

---

## STEP 3 — rebuild the reference from a camera frame (20 min, only if needed)

This is expected-ish. Phone video → RealSense is a real domain gap, and a
reference built from the actual camera will always match better.

```bash
python3 make_reference.py --grab --topic /camera/color/image_raw --out /tmp/ref_frame.png
```
It prints a sharpness number. **If it warns the frame is soft, grab another** —
a blurred reference matches badly at every distance.

Open `/tmp/ref_frame.png`, read off roughly where the +30SEC button is and a box
around the whole panel, then:

```bash
python3 make_reference.py --mark /tmp/ref_frame.png \
    --near <BTN_X> <BTN_Y> \
    --panel <X0> <Y0> <X1> <Y1> \
    --radius 14 \
    --ref-dir $REF
```

`--near` only has to be roughly on the button; a circle fit places the real
centre. **Do not hand-tune the result.** Every prediction is this one point
carried through the fitted transform, so an error here appears in every frame
forever — that is exactly what went wrong on the phone-video reference, and no
automated score caught it.

### → LOOK AT `$REF/reference_check.png`
Red marker on **+30SEC**. Green box enclosing the panel including its printed
labels (the cream panel is low-texture; the *text* carries most of the SIFT
features).

Then re-run STEP 2 against `--ref $REF`. **Do not proceed until it matches.**

---

## STEP 4 — build the package (30 min)

```bash
mkdir -p $WS/src && cd $WS/src
ros2 pkg create rammp_button_detect --build-type ament_python \
  --dependencies rclpy sensor_msgs geometry_msgs cv_bridge

P=$WS/src/rammp_button_detect
cp <repo>/scripts/scratch/button_live/button_detector_node.py        $P/rammp_button_detect/
cp <repo>/scripts/scratch/button_live/pkg_template/mock_button_detector.py $P/rammp_button_detect/
cp <repo>/src/feeding_deployment/perception/appliance_perception/reference_button_detector.py \
                                                                     $P/rammp_button_detect/
mkdir -p $P/launch
cp <repo>/scripts/scratch/button_live/pkg_template/launch/button_detect.launch.py $P/launch/
```

**Fix the import.** `button_detector_node.py` imports the detector by its
feeding-deployment path. In the new package it is a sibling:

```python
# replace:
# from feeding_deployment.perception.appliance_perception.reference_button_detector import (
#     ReferenceButtonDetector,
# )
from .reference_button_detector import ReferenceButtonDetector
```

Then edit `setup.py` per `pkg_template/setup_py_snippet.txt` (entry points +
the launch `data_files` line), and build:

```bash
cd $WS && colcon build --packages-select rammp_button_detect
source install/setup.bash
```

**Checkpoint:** build succeeds, `ros2 pkg executables rammp_button_detect`
lists `button_detector_node` and `mock_button_detector`.

---

## STEP 5 — run the node (20 min)

Mock first — proves the plumbing with nothing that can fail perceptually:

```bash
ros2 run rammp_button_detect mock_button_detector
ros2 topic echo /button_detector/button_pose --once
```

Then the real one:

```bash
ros2 launch rammp_button_detect button_detect.launch.py reference_dir:=$REF
```

In another terminal:

```bash
ros2 topic hz   /button_detector/button_pixel
ros2 topic echo /button_detector/button_pixel --once
ros2 run rqt_image_view rqt_image_view /button_detector/debug_image
```

### What you should see
- `debug_image` green quad on the panel, red crosshair **on +30SEC**
- amber `ABSTAIN` with a reason when you move the camera away — **this is
  correct behaviour, not a bug.** It declines rather than guessing.
- `button_pixel` publishing only while locked

### If `button_pose` never publishes
Expected and benign: chrome domes are close to worst-case for projected-light
depth. The log will say `button found but no valid depth at that pixel`.
The pixel output is the real deliverable tonight.

**Capture for review:** a screenshot of `debug_image` locked, and one
abstaining.

---

## STEP 6 — sheppy launches it (20 min)

Update sheppy first (your colleague's note):
```bash
curl -LsSf https://rammp-org.github.io/sheppy/install.sh | sh
```

Add the entry from `pkg_template/sheppy_node_entry.yaml` to your **local**
profile — machine `dev`, `ros_setup` pointing at `$WS/install/setup.bash`,
`reference_dir` at `$REF`. Keep it **uncommitted**; per the hybrid workflow only
the containerized alternative gets published later.

```bash
source $WS/install/setup.bash
cd ~/rammp-deployments/december_2026
sheppy                       # TUI, save the profile as `dev`
sheppy status                # every row should read `running`
sheppy restart button_detector
```

**Checkpoint — this is the actual ask:** `sheppy status` shows
`button_detector` `running`, and `debug_image` still looks right. That is
"confirm the nodes are launching correctly."

---

## STOP HERE

**Do not attempt arm motion tonight.**

`button_pose` — the pixel→3D conversion — is **completely unvalidated**. The
22/29-with-zero-wrong-buttons result is the **pixel** stage measured on phone
video with no depth and no calibration. Every line converting pixel to metres
is unexercised code.

The log already contains an e-stop, a wrong-direction arc, and an uncontrolled
~150° joint reconfiguration — all from motion attempts that looked ready. A
detector that met this camera for the first time tonight is not what you test
that on at the end of a long evening.

**Next session, with someone present:** validate the pose by touching the
gripper to the real button and comparing. That is the same
manual-touch-ground-truth method that found the 20cm handle error before.

---

## If you get stuck, capture these

1. exact command + full terminal output
2. `/tmp/ref_match.png` or `$REF/reference_check.png`
3. `ros2 topic list | grep -E "camera|button"`
4. `ros2 topic hz` for colour and aligned depth
5. a `debug_image` screenshot

Paste those in and I can work the problem directly.

---

# Motion: autonomous press (added 2026-09-21, rchi-cpu-5)

The "do not attempt arm motion" block above was written before the pixel stage was
validated on the real camera and before there was a force sensor in the loop. Both exist
now: `~/wrist_ref_red` locks the real panel, and `detect_button_press_force.py` reads the
tool wrench (0.1 N rest noise, 20-50 N on a hand push). The press is done by
`press_button_autonomous.py` -- two-dots visual servo + force stop, pure translation,
step-and-check. Read its module docstring first; this section is only the bring-up order.

**Someone stands at the physical e-stop for every `--execute`.** Killing
`bulldog_bypass.py` e-stops the arm within ~1 s; Ctrl-C in the press script stops the
next step (the in-flight <=1 cm step completes). `arm_halt.py` is unverified on hardware.

## M0. Environment (every terminal)

```bash
cd ~/feeding-deployment-button-task            # or wherever this checkout is
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_large_images.xml   # or images drop to 2-5 Hz
export ARM_RPC_HOST=127.0.0.1
export PYTHONPATH=$PWD/src:$PYTHONPATH        # PREPEND. `PYTHONPATH=src` alone drops rclpy.
mkdir -p ~/press_logs
```

## M1. Arm servers (no motion yet)

Order matters; each is its own terminal (or `setsid nohup ... &`).

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
Want: `ARMSTATE_SERVOING_READY`, speed `low`, gripper **~0.99 (closed)** -- the press uses the
closed fingertips. If you get `INVALID_USER_SESSION_ACCESS` or `ARMSTATE_IN_FAULT`: teleop
or an e-stop took the session. Stop teleop, release the e-stop, then restart the three
servers in the same order (a fresh `arm_server` re-locks motion, so the bypass must be
re-run). Seen and recovered exactly this way on 2026-09-21.

## M2. Camera + tf chain

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p align_depth.enable:=true -p camera_name:=camera \
    -p rgb_camera.auto_exposure_priority:=false            # see the fastdds memory note
ros2 run robot_state_publisher robot_state_publisher <urdf>  # as in launch/ros2/microwave_bringup.launch.py
python3 -u -m feeding_deployment.ros2.joint_state_bridge    # republishes the arm RPC as /joint_states
python3 -u -m feeding_deployment.ros2.calibration_tf --calib ~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib
```
(`ros2 launch launch/ros2/microwave_bringup.launch.py` with system python3 starts the last
three together.) **Standing rule before any motion:**
```bash
ros2 topic hz /joint_states                                   # ~50 Hz. Silent => bridge is attached to a dead arm_server; restart it
ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame   # must print a transform
```

## M3. Perception

```bash
python3 -u scripts/scratch/button_live/button_detector_node.py --ros-args \
    -p reference_dir:=$HOME/wrist_ref_red -p target_button:=timer_clock      # or start_30s
ros2 run rqt_image_view rqt_image_view /button_detector/debug_image
ros2 topic echo /button_detector/status        # want "locked <target> inliers=N" with N >= 15
```
Fewer than ~15 inliers after the arm moved => the framing drifted; add a reference view
(`add_reference_view.py`, see the memory note), do not retune.

Then the force detector, **with the arm parked and nobody touching it for the first 3 s**:
```bash
python3 -u scripts/scratch/detect_button_press_force.py --threshold 8 --publish --print-hz 0 --log ~/press_logs/force.jsonl
```
Want `rest noise` ~0.1 N and no `ARM WAS MOVING` warning. Re-run it after any teleop (the
compensated wrench is pose-dependent, so the baseline goes stale). The overlay's bottom bar
turns from grey "no data" to green `|dF| 0.0x N` when it is up.

## M4. The press -- one rung per invocation

Measure `TIP` once: ruler from the camera lens to the LEFT fingertip, in metres
(over-estimate rather than under). Then:

```bash
P=scripts/scratch/button_live/press_button_autonomous.py
python3 $P --target timer_clock                                     # DRY RUN: preflight + every planned step + gates
python3 $P --target timer_clock --execute --stage 1                 # lateral servo only; crosshair should land on the claw marker
python3 $P --target timer_clock --execute --stage 2 --tip-dist $TIP --cap-override 0.05   # approach, stops 5 cm short
python3 $P --target timer_clock --execute --stage 2 --tip-dist $TIP # approach until CONTACT (2.5 N at rest)
python3 $P --target timer_clock --execute --tip-dist $TIP --presses 1   # servo, approach, press, retract
```
After an abort the arm HOLDS. Retreat with
`python3 $P --execute --stage 4 --resume-travel <metres printed at the abort>`, or
`scripts/session/goto_preset.py` to a saved pose. Per-step JSONL lands in `~/press_logs/`.

## M5. Shutdown

Re-check `gripper_pos` and `get_arm_state()` (idle, `SERVOING_READY`), then stop in reverse:
press script -> force detector -> detector node -> bulldog bypass (arm e-stops ~1 s later,
expected) -> stub base -> arm_server -> camera/tf.
