# Fridge open and close — how to test

All constants are in `fridge_constants.py`.

- **Open:** the `--fridge` mode of `real_gen3_ros2_grasp_and_swing_microwave.py`. It detects the handle, grasps it
  and swings the door open in one run. First successful run on hardware: 10-22.
- **Push-open option:** `--fridge --push-open`. It opens the microwave's way: a short pull, then let go and push.
  Not yet run on hardware.
- **Close:** `real_gen3_ros2_close_microwave.py --fridge`. It is the microwave's push-close, reached by going round
  the door instead of over it, and it stops at 45°. Not yet run on hardware.

## Before the first run

- For `--push-open` and the close, measure these and update the constants. They size the box door and cabinet that those plans check
  clearance against:
  - `DOOR_WIDTH_M`: the door's full width, hinge-side edge to free edge, **measured at the top of the door**.
    **Placeholder 0.45 m, not measured.** It decides where the plans think the free edge is.
  - `DOOR_DEPTH_M`: the door's depth, from its front face to the innermost point of its inside, bins and shelves
    included. **Placeholder: just the 1.25 in thickness, not measured.** If the inside has bins, note the height
    range they cover relative to the handle too. It decides how much room there is behind the open door.
  - `DOOR_ABOVE_HANDLE_M` / `DOOR_BELOW_HANDLE_M`: door top / bottom above / below the middle of the handle bar
    (0.50 / 0.50, generous).

- If testing at HERL, remeasure these constants:
  - `HINGE_ALONG_DOOR_M`: at handle height, from the centre line of the handle bar to the hinge pin's axis (the line
    the door turns about), parallel to the door.
  - `PIN_BEHIND_FACE_M`: from the door's front surface back to the hinge pin's axis, perpendicular to the door.
  - `HANDLE_CENTRE_TO_FACE_M`: from the middle of the handle bar (front to back, where the fingers close) to the door's
    front surface, perpendicular to the door, including any gap behind the bar. 

  A wrong hinge drags the door or the fridge during the swing.

- For the close: `CHAIR_BACK_DIR` assumes the arm faces the wheelchair's forward direction. If backing up the chair
  moves the fridge some other way in the arm's frame, change it.

## How to stop the robot

| How | Effect |
|---|---|
| **Software e-stop in T3** (first choice) | The driver cancels the motion and the arm **holds where it is**, powered (it does not drop); the gripper keeps its grip. All motion is refused until it is cleared. |
| **Ctrl-C in T2** | Cancels the running motion; the arm stops and holds, powered. |
| Physical e-stop button | Last resort only (the driver process is hung or dead): it cuts power and the arm drops. |

**This one is the e-stop.** Keep it typed out in T3 ahead of time and send it to stop the robot:

```bash
ros2 topic pub --once /estop rammp_common_interfaces/msg/EStop "{engaged: true, source: 'operator', reason: 'stop'}"
```

**This one does not stop anything: it clears the e-stop afterwards.** Send it only right before you continue (only a
fresh message counts):

```bash
ros2 topic pub --once /estop rammp_common_interfaces/msg/EStop "{engaged: false, source: 'operator', reason: 'clear'}"
```

If the driver is faulted afterwards, run `sheppy restart arm` in T1.

Never teleop while the arm driver is up: it crashes the driver. Run `sheppy down` in T1 first, and bring it back up
with step 1 after.

## Open: commands

Run this first in every terminal:

```bash
source ~/feeding-deployment/microwave/sheppy/env.sh     # also cd's to the repo root
```

| Step | Terminal | Command | What it does |
|---|---|---|---|
| 0 | — | Teleop, with sheppy down (nothing running) | **Point the arm at the fridge** (door closed). The camera must face the door roughly square-on, with the handle in view and the door filling most of the image. The camera should be ~50–60 cm from the door (the 10-21 recording was 55 cm). Keep J4 around −130 to −135°. |
| 1 | T1 | `sheppy up --manifest microwave/sheppy/sheppy-manifest.yaml real-arm-camera && sheppy status` | Starts the arm driver, camera, TF and the camera window (teleop is off from here on). Every node should show as running. |
| 2 | T2 | `python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --fridge --swing-deg 90 --execute` | **Open, 90°.** Grasps, swings the door 90°, then lets go and backs 12 cm straight out of the handle. If you want a dry run first, don't add the execute flag.|
| 2b | T2 | `python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --fridge --push-open --execute` | **Open the microwave's way instead (option).** Pull 35° holding the handle, then let go and push the door further open. See "Push-open option" below. |
| 3 | T2 |  — |  Teleop to grab the tupperware (need to run `sheppy down` first) |
| 4 | — | Close the door by hand, go back to step 0, run step 2 again | **Repeat.** For a wider open, change the number in `--swing-deg`: it is not capped at 90 (the fridge branch's runs reached ~100°). |
| 5 | T1 | `sheppy down` | Shutdown. |

## Open: what to expect

**Prompts:** none. The run goes straight through once started.

**Open** (expected):
1. Two camera looks, then a plan.
2. Moves to 12 cm (or 10 cm) straight out from the handle, square to the door.
3. Goes straight in and closes the gripper.
4. Swings the door `--swing-deg` (90 here), exactly. There is no smaller fallback.
5. Lets go and backs 12 cm straight out of the handle (checked in sim first, from where the swing ended). No push-open, no park.

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
- the back-off fails its sim check after the swing. It doesn't let go: the hand stays on the handle. Run step 2's
  command with `--release` instead of `--swing-deg 90` to retry, or `sheppy down` and teleop.

**Watch for:**
- **Where the fingers close.** The grasp offset is the microwave's `GRIP_EXT` (never tested on this handle), and nothing
  checks the grasp: `tools/touch_check.py` has no `--fridge` mode yet. If the fingers hit the door or close short of the
  bar, the fridge needs its own `GRIP_EXT` in `fridge_constants.py` (bigger = shorter).
- **Swing direction.** It should pull the handle toward the arm.
- **Dragging.** The fridge sliding or tipping, or the hinge side moving forward or back during the swing, means the
  hinge is off. Re-measure the values above.

Each run saves the door to `~/.fridge_door.json` (never the microwave's file) and its camera frames to
`~/microwave_detections/`.

## Push-open option (`--push-open`)

This is the microwave's open on the fridge (`fridge_push.push_open`). It pulls `PUSH_OPEN_PULL_DEG` (35°, or
`--swing-deg`) holding the handle, then lets go and replays the end of the microwave's push-open demo
(`demos/push_open_2026-10-07.door.json`) against the fridge door. The microwave demo starts by going up and over the
door. A fridge door is too tall for that, so this goes round the door's free edge at hand height instead:

1. Straight out 12 cm off the handle.
2. Along the open door to 16 cm past its free edge.
3. Round the edge.
4. The demo's keypoint 4, into the gap behind the door.
5. The demo's keypoint 5, the push.

It runs as one smooth trajectory, planned and checked in sim from where the pull really ended, before anything
moves. Only the push leg is allowed to touch the door model.

**Expect it to refuse on the 10-22 setup.** In an offline sim check of that detection (fridge handle 0.56 m out,
door width and depth guessed), nothing clears the door on step 4 at any pull tried (20–50°). 35° came closest (a
finger just touching the door model), which is why it is the default; 30° and 40° go 0.8 cm into it, 50° 1.3 cm.
Below 35° the gap behind the door is too narrow for the open fingers. Above it the fridge is so close that the open
door swings into the wrist's path. Measure the door's width and depth first: the margins are 1–2 cm. If it
refuses, nothing moves and the hand stays on the handle. Then run step 2's command with `--release` instead of
`--push-open` to let go and back out.

After a push-open, the door's angle isn't measured, so give the close `--door-deg`.

`python3 -u microwave/fridge_push.py` runs the push-open alone, from the end of a pull (still holding the handle).
Without `--execute` it plans and checks without moving.

## Close (`real_gen3_ros2_close_microwave.py --fridge`)

This is the microwave's push-close (`door_push.push_close`, the close that latched the microwave on 10-21), with three
changes for a fridge (`fridge_push.close`):

1. **You move the wheelchair back ~30 cm first.** The script asks for this and waits for `y`. The plan uses the
   fridge recorded by the last `--fridge` open, shifted 30 cm further away (`CHAIR_BACK_M`, `CHAIR_BACK_DIR`).
2. **No going over the door.** The hand backs out in front of the open door's free edge, goes left past the door,
   then forward to the swing start behind it.
3. **It stops at 45°.** The swing about the hinge (the microwave's) ends when the door is at `CLOSE_TO_DEG` (45°),
   and the door swings shut by itself from there. There is no torque push.

The gripper is never touched (it may be holding a container). It has only been checked in sim, against the 10-22
fridge detection: the plan passed from two of three test start poses.

### Close: commands

| Step | Terminal | Command | What it does |
|---|---|---|---|
| 1 | T2 | Open step 2 above (90° pull) | Saves the door to `~/.fridge_door.json`, which the close plans from. |
| 2 | T1 | `sheppy down`, teleop to grab the container, then open step 1 again | **Grip the container.** |
| 3 | T2 | `python3 -u microwave/real_gen3_ros2_close_microwave.py --fridge` | **Dry run.** No prompt. Plans as if the chair has moved back, and moves nothing. |
| 4 | T2 | `python3 -u microwave/real_gen3_ros2_close_microwave.py --fridge --execute` | **Close.** Asks you to move the chair back, then runs. |
| 5 | — | Move the wheelchair forward again | Before the next open. The open re-detects the door. |

Options (the defaults are in `fridge_constants.py`):
- `--door-deg N`: how far the door is open now (default: the door file's angle, i.e. the last pull). Give it after
  a push-open, or if the door was moved by hand.
- `--chair-back M`: how far the chair is moved back (default 0.30). `0` means it wasn't moved, and there is no prompt.
- `--close-to-deg N`: where the swing stops (default 45).
- `--push-radius` / `--push-radius-end`: the hand's distance from the hinge during the swing (default 0.30 → 0.28).
- `--push-z Z`: the height of the swing (default: where the hand is now).
- `--hold-offset-x`, `--side-past-hinge`, `--retreat-dist`: as for the microwave close.

### Close: what to expect

**Prompt:** with `--execute`, one prompt: move the wheelchair straight back ~30 cm and type `y`. Anything else stops
the run, arm untouched. If the chair is still back from an earlier close, leave it there and type `y`.

1. Plans everything first. If any leg fails a check, it refuses with the arm untouched.
2. Backs out to in front of the open door's free edge, goes left past the door, then forward to the swing start
   behind it.
3. Swings the door shut about the hinge until it is at ~45°.
4. Backs out up to 20 cm.
5. Prints `PUSH-CLOSE DONE`, then a reminder to move the wheelchair forward again.

**Watch for:**
- The container is not in the collision model.
- If the chair didn't move straight back, or moved much more or less than 30 cm, the plan is off by that much.
- Whether the door swings shut from 45° by itself.
