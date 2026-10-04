# `button_press` — how the autonomous microwave button press works

The arm presses a named microwave button (`timer_clock`, `start_30s`, …) by itself: it looks
at the panel with the wrist camera, works out where the button is in 3D, moves to a stored
spot just in front of it, pushes straight in along the panel's normal, comes back out, and
returns to where it started.

How to run it (bring-up, commands, recovery): **`docs/button_press_runbook.md`**.
This page explains what the code does.

## The idea in one paragraph

Find the button's 3D position and the panel's orientation, and build a **panel frame**
from them: origin at the button, z out of the panel, y up. The pre-press spot is one
constant in that frame (`PREPRESS_EE_OFFSET_M` / `PREPRESS_EE_QUAT_PANEL` in
`press_button.py`), measured once by hand-guiding the fingertip to 2 cm in front of the
button. Because the frame's origin is the target button itself, the same constant works
for every button. The press depth is pure geometry (`--press-in`, capped at 3 cm).

## How the button is found

`dome_pattern.py` looks for the panel's 5 chrome domes by their layout: 3 on top, 2 below,
offset half a step. Each candidate fit must hit all 5 domes *and* their spacing must measure
about 19 mm in metres via depth, which rules out label text and knob highlights. The 3-over-2
layout fixes "up", so a rolled camera still names the domes correctly. The panel plane is
fitted to the red pixels around the domes (not the chrome, where the depth sensor is wrong).
It needs no reference images and works from about 22 to 50+ cm. Closer than 22 cm a dome
breaks into several highlights, so it refuses to answer there.

## What `press_button` does, step by step

```
 RealSense (colour + aligned depth) ──► press_button ──arm RPC──► arm_server ──► Kinova
 tf: arm_base_link → camera ───────────┘
```

0. **Preflight** (`Run.preflight`): checks the arm is ready, the gripper is closed, the speed is `low` or `medium`, and camera info and tf are available. It saves the start joints.
1. **Detect** (`measure_panel_frame_domes`): 7 frames, at least 5 must fit all 5 domes and agree on the target pixel within 3 px. The median pixel's ray meets the fitted plane at the button; tf puts it in the arm base frame.
2. **Stage** (camera more than 25 cm from the button): far estimates read about 1.5 cm low, so the arm first goes to a spot 8 cm further out than the pre-press spot and re-detects from there. A dry run from far away stops after planning this leg.
3. **Plan**: every leg (to the pre-press spot, each press in and out, back to the start) is solved in the sim before anything moves.
4. **Go** to the pre-press spot, turning the wrist to face the panel.
5. **Press** (`Run.press_along_normal`): `--press-in` straight into the panel, hold 0.3 s, the same distance out. Repeats for `--presses N`.
6. **Return** in a straight line to the start pose (skipped with `--no-return`).

**Safety behaviour:** every move is planned in a headless PyBullet copy of the arm first (`arm.py`). A move is refused before anything is sent if the IK error is over 5 mm, any joint would move more than 10°, or it goes out of reach or out of the height band. If a move aborts on the way in, the arm goes back to the start. If a press aborts, it backs out and holds. Each leg (approach, each press stroke, return) is re-checked from the arm's actual pose just before it goes, then sent as one smooth Cartesian trajectory.

## File map

| File | Contents |
|---|---|
| `press_button.py` | **The driver**: the steps above, `Run` (preflight, gated paths, the press stroke), and the pre-press constants |
| `dome_pattern.py` | The button finder *(pure, unit-tested)* |
| `panel_frame.py` | The panel frame (origin = button, z = out, y = up) and pose interpolation *(pure, unit-tested)* |
| `geometry.py` | Panel plane fit, pixel rays, joint-angle maths *(pure, unit-tested)* |
| `perception.py` | `Perception`: the ROS 2 node holding the latest camera frames, plus tf |
| `arm.py` | `Arm`: seeded PyBullet IK, the motion gates, and sending and verifying moves |

Other places: `scripts/button_press/` (`bringup.sh`, `measure_prepress_offset.py`,
`view_detection.py`) and `tests/test_button_press_*.py`. Each constant in the code has a
comment giving the measurement or incident that set it. Read those before changing a number.
