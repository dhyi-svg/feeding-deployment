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
