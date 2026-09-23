# `button_press` — how the autonomous microwave button press works

The arm presses a named microwave button (e.g. `timer_clock`, `start_30s`) by itself:
it looks at the panel with the wrist camera, lines the button up with its fingertip, pushes
straight in until it *feels* the button, presses, and backs off. It worked end-to-end on
the real Comfee microwave on 2026-09-21/22.

How to run it (bring-up order, commands, recovery): **`docs/button_press_runbook.md`**.
This page explains what the code does.

## The idea in one paragraph

The camera is bolted to the wrist, so the left fingertip always shows up at the **same
pixel** in the image (the "claw pixel", `LEFT_CLAW_PIXEL`). If the button's pixel sits on
the claw pixel, the button lies on the fingertip's line of sight. Sliding the wrist along
that line, without rotating it, keeps it that way until the fingertip touches the button.
So: **camera pixels fix the sideways error, force sensing fixes the depth.** The only
calibration needed is the camera's *rotation* relative to the arm base (from tf2), and a
few degrees of error there just costs an extra correction.

## The three processes

```
 RealSense camera ──► detector_node ──► /button_detector/button_pixel, claw_pixel,
                      (which pixel is      panel_quad, status, debug_image
                       the button?)                 │
                                                    ▼
 Kinova tool wrench ─► press_detector ─► /press_detector/force_dev ─► autonomous_press ─► arm
                       (how hard is the                                (the driver: moves
                        fingertip pushing?)                             the arm step by step)
```

| Process | File | What it does |
|---|---|---|
| Button detector | `detector_node.py` | Matches the live image against saved reference photos of *this* microwave's panel (SIFT + homography, `perception/appliance_perception/reference_button_detector.py`) and carries a hand-placed mark on the reference into the live frame. Publishes the button pixel only when the match is good. Otherwise it publishes nothing and the status reads `abstain`, so it never guesses. |
| Force detector | `press_detector.py` | Reads Kinova's estimate of the external force at the tool through a separate read-only session, subtracts a resting baseline, and publishes the difference. `kill -USR1` re-zeroes it. The driver does this between phases, because the estimate drifts with arm pose. |
| Driver | `autonomous_press.py` | Runs the stages below. It is a **dry run unless `--execute`** is given. |

## What the driver does, stage by stage

0. **Preflight**: checks the arm is ready, the gripper is closed, the speed is `low`, the detector has held a lock on the requested button for 2 s, the force feed is alive and reads near zero, and tf is available. It saves the start joints so there's a way back.
1. **Line up** (`Run.servo`): computes the pixel error between the button and the claw, converts it to a sideways camera move scaled by depth, and repeats until the error is under 4 px. It uses a median of 9 frames, because about 1 frame in 30 matches the wrong reference view.
   - *Far start* (`Run.far_approach`, panel more than 23 cm away): first closes to 20 cm while keeping the button 110 px above the claw, so the panel stays in view. **Never completed on hardware.**
2. **Approach** (`Run.approach`): steps along the fingertip ray, 1 cm at a time and then 2–3 mm near the panel. After every step, with the arm still, it reads the force. The travel cap is the distance to the panel plane (fitted from depth) minus `--tip-dist`, plus 1.5 cm.
   - Contact detection only switches on within 3 cm of where the panel should be. Further out, force jumps come from arm pose, not contact.
   - A rising force makes a *candidate*. It only counts as **contact** if one more step pushes it up again, because a real press keeps loading while pose drift bounces around. The rule is in `contact.py`, with the measurements behind each number.
3. **Press** (`Run.press`): pushes a little further (`--press-travel`), holds 0.3 s, then backs off 2 cm. It repeats for `--presses N`.
4. **Retract** (`Run.retract`): reverses all the travel back to the standoff.

**Safety behaviour:** every move is planned in a headless PyBullet copy of the arm first (`arm.py`). A move is refused before anything is sent if the IK error is over 5 mm, any joint would move more than 10°, or it goes out of reach or out of the height band. Moves along the ray are split into pieces of at most 1 cm. **On any problem the arm stops and holds where it is.** Nothing retracts automatically; the script prints the command to back out.

## File map

| File | Contents |
|---|---|
| `autonomous_press.py` | `Run`: the stages above, the utility modes (`--jog`, `--goto-start`, `--resume-travel`) and the CLI |
| `perception.py` | `Perception`: the ROS 2 node the driver reads from (pixels, force, depth, tf) and the panel-plane measurement |
| `arm.py` | `Arm`: seeded PyBullet IK, the motion gates, and sending and verifying joint moves |
| `contact.py` | The contact rule and its measured thresholds *(pure Python, unit-tested)* |
| `geometry.py` | Plane fit, pixel rays, sideways correction, joint-angle maths *(pure, unit-tested)* |
| `detector_node.py` | The button detector ROS 2 node |
| `press_detector.py` | The force detector (live, `--replay` of a log, `--publish` to ROS 2) |

Other places: `launch/ros2/button_press_bringup.launch.py` (starts both detectors),
`scripts/button_press/` (build, extend and check the panel reference), and
`tests/test_button_press_*.py`. Each constant in the code has a comment giving the
measurement or incident that set it. Read those before changing a number.
