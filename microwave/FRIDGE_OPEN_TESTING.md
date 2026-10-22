# Fridge open — how to test

The `--fridge` mode of `real_gen3_ros2_grasp_and_swing_microwave.py`: it detects the handle, grasps it and swings the door
open, in one run. Constants are in `fridge_constants.py`. **Not yet run on hardware.** It was only checked offline, on
the 10-21 recording (black door, white handle).

## Before the first run

- If testing at HERL, remeasure these constants:
  - `HINGE_ALONG_DOOR_M`: at handle height, from the centre line of the handle bar to the hinge pin's axis (the line
    the door turns about), parallel to the door.
  - `PIN_BEHIND_FACE_M`: from the door's front surface back to the hinge pin's axis, perpendicular to the door.
  - `HANDLE_CENTRE_TO_FACE_M`: from the middle of the handle bar (front to back, where the fingers close) to the door's
    front surface, perpendicular to the door, including any gap behind the bar. 

  A wrong hinge drags the door or the fridge during the swing.

## Stop the robot

| How | Effect |
|---|---|
| **Software e-stop in T3** (keep this typed out in T3 ahead of time) | The arm stops and **holds where it is**, powered; the gripper keeps its grip. All motion is refused until cleared. |
| **Ctrl-C in T2** | Cancels the running motion; the arm stops and holds, powered. |
| Physical e-stop button | Last resort only (driver hung or dead): cuts power and the arm drops. |

```bash
ros2 topic pub --once /estop rammp_common_interfaces/msg/EStop "{engaged: true, source: 'operator', reason: 'stop'}"
ros2 topic pub --once /estop rammp_common_interfaces/msg/EStop "{engaged: false, source: 'operator', reason: 'clear'}"   # clear, right before continuing
```

If the driver is faulted afterwards, run `sheppy restart arm` in T1. Never teleop while the arm driver is up: switch to
`camera-only` first.

## Commands

Run this first in every terminal:

```bash
source ~/feeding-deployment/microwave/sheppy/env.sh     # also cd's to the repo root
```

| Step | Terminal | Command | What it does |
|---|---|---|---|
| 0 | T1 | `cd microwave/sheppy && sheppy up --manifest sheppy-manifest.yaml camera-only`, then teleop | **Point the arm at the fridge** (door closed). The camera must face the door roughly square-on, with the handle in view and the door filling most of the image. The camera should be ~50–60 cm from the door (the 10-21 recording was 55 cm). Keep J4 around −130 to −135°. |
| 1 | T1 | `sheppy up --manifest sheppy-manifest.yaml real-arm-camera && sheppy status` | Starts the arm driver, camera and TF. Every node should show as running. |
| 2 | T2 | `python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --fridge --swing-deg 90` | **Dry run.** Detects, plans and moves nothing. Check the printed handle, hinge and grasp, and look at `~/microwave_detections/*_rgb.jpg`. |
| 3 | T2 | Same as step 2 plus `--execute` | **Open, 90°.** Grasps and swings the door 90°, then stops holding the handle. |
| 4 | T2 | `python3 -c "import sys; sys.path.insert(0, 'microwave'); from sheppy_arm import SheppyArm; SheppyArm().gripper(0.0)"` | **Let go.** Opens the gripper; the arm stays put. `--release` refuses with `--fridge`. Then switch to `camera-only` (T1) and teleop the arm away. |
| 5 | — | Close the door by hand, go back to step 0, run step 3 again | **Repeat.** For a wider open, change the number in `--swing-deg`: it is not capped at 90 (the fridge branch's runs reached ~100°). |
| 6 | T1 | `sheppy down` | Shutdown. |

## What to expect

**Prompts:** none. The run goes straight through once started.

**Open** (expected):
1. Two camera looks, then a plan.
2. Moves to 12 cm (or 10 cm) straight out from the handle, square to the door.
3. Goes straight in and closes the gripper.
4. Swings the door `--swing-deg` (90 here), exactly. There is no smaller fallback.
5. Ends **holding the handle**. There is no push-open and no release.

**Stops before any motion if:**
- detection fails, or two looks don't agree within 3 cm
- the handle is outside x 0.30–0.85, y −0.60–0.60 or z 0.25–0.75 m
- the grasp is more than 0.94 m from the arm base: move the fridge closer.
- the hinge radius is outside 25–45 cm
- the full `--swing-deg` swing doesn't pass the sim checks (reach, joint guards, IK). It refuses rather than open less.

**Stops partway if:**
- the grasp lands more than 3 cm off. It stops with the gripper untouched.
- the gripper reads open after closing. Nothing is grasped, so it doesn't swing.
- the swing re-check fails. It stops still holding the handle.

**Watch for:**
- **Where the fingers close.** The grasp offset is the microwave's `GRIP_EXT` (never tested on this handle), and nothing
  checks the grasp: `tools/touch_check.py` has no `--fridge` mode yet. If the fingers hit the door or close short of the
  bar, the fridge needs its own `GRIP_EXT` in `fridge_constants.py` (bigger = shorter).
- **Swing direction.** It should pull the handle toward the arm.
- **Dragging.** The fridge sliding or tipping, or the hinge side moving forward or back during the swing, means the
  hinge is off. Re-measure the values above.

Each run saves the door to `~/.fridge_door.json` (never the microwave's file) and its camera frames to
`~/microwave_detections/`.
