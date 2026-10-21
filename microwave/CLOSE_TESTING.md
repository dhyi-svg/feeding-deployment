# Microwave close — real-robot testing plan (Jetson, sheppy)

Written 2026-10-21. Nothing in here has been run on the robot yet: every command is either
read-only, a dry run, or motion you start yourself with `--execute`.

What this covers:

- **Part A**: record a teleop demo (joint angles + RGB-D frames) to work out the elbow-push
  joint plan by hand.
- **Part B**: test the close code at low speed.

What the close does: `door_task.py close` replays the 10-07 close demo relative to the door the
open task detected (`~/.microwave_door.json`). By default it runs exactly as on 10-07 (plain IK,
the wrist ends left of the elbow); flags select the wrist-right variant and/or a push 10 cm
further toward the handle. The push is done by the forearm/wrist sweeping across the door.

All commands run from the repo root unless they `cd` somewhere else. Every terminal needs:

```bash
source ~/feeding-deployment/microwave/sheppy/env.sh      # ROS 2 + ~/ros2_ws + .venv; also cd's to the repo root
```

## 0. Stop levers — read before any motion

| # | What | When |
|---|---|---|
| 1 | **Physical e-stop button** | Anything about to hit something. Always the first choice. |
| 2 | **Ctrl-C** in the terminal running `door_task.py` | Wrong but not urgent. `SheppyArm` cancels the running trajectory goal (the arm stops) and the run log is still written. |
| 3 | **Software e-stop** from a spare terminal (keep it typed out, ready to send) | `door_task.py` hung or isn't the thing moving. The driver stops the arm and refuses motion until cleared. |
| 4 | **Stop the arm driver:** `cd ~/feeding-deployment/microwave/sheppy && sheppy up --manifest sheppy-manifest.yaml camera-only` | Take the driver away entirely (this also frees the arm for teleop). `sheppy down` stops everything. |

Software e-stop (#3):

```bash
# engage (never refused, even if stale)
ros2 topic pub --once /estop rammp_common_interfaces/msg/EStop "{engaged: true, source: 'operator', reason: 'stop'}"
# clear (only a FRESH message is honoured -- run it right before you continue)
ros2 topic pub --once /estop rammp_common_interfaces/msg/EStop "{engaged: false, source: 'operator', reason: 'clear'}"
```

If the driver is faulted after a stop: `cd ~/feeding-deployment/microwave/sheppy && sheppy restart arm`.
Check `sheppy status` before moving again.

**Never teleop (Kinova joystick / web app) while the arm driver is up.** It takes the arm out
of low-level servoing and crashes the driver. Switch to `camera-only` or `teleop-record` first.

Suggested terminal layout:

- **T1:** sheppy (`up`, `status`, `logs`)
- **T2:** the commands
- **T3:** the e-stop command typed out, ready to send

---

## Part A — teleop recording (joint angles + occasional camera frames)

Script: `scripts/session/record_teleop_demo.py`, ported to sheppy on 10-21 (it previously
needed the old `microwave_bringup.launch.py` stack).

How it works:

- **Read-only.** It never commands the arm.
- **Arm data.** It reads the arm through its own read-only Kortex session, so it works while
  the arm driver is down and you teleop. It publishes `/joint_states` from that session, so
  sheppy's `robot_state_publisher` and hand-eye calibration give every frame its camera pose.
- **What it records:**
  - every arm sample at 30 Hz: joints, velocity, torque, tool pose, gripper
  - RGB-D frames at 2 Hz while the arm moves, plus a short settle afterwards; none while it's parked
- **Tagging keypoints.** Type a label and press Enter to tag the current instant (e.g.
  `behind_door`, `push_start`, `shut`). Each tag stores the arm state and forces a frame.
  `q` + Enter (or Ctrl-C) stops.
- **Output:** `~/captures/teleop_<time>/`
  - `state.jsonl`: all samples
  - `events.jsonl`: your tags, with joints
  - `frames/NNNNNN/`: rgb jpg, uint16 depth png, camera info, base←camera matrix, arm state
  - `meta.json`
- **Size:** about 16 MB/min while moving.

**A1. One-time: `kortex_api` in its own folder (the sheppy venv is NOT touched).**

The recorder needs `kortex_api` to read the arm while the driver is down. It goes into
`~/kortex_isolated` and is only put on `PYTHONPATH` for the recorder's own command (A3/A4),
so nothing else ever imports it. `--no-deps`: it uses the protobuf already on the system
(3.12.4) instead of the wheel's broken 3.5.1 pin, and installs nothing else. Tested this
way on 10-21 (imports, plus the recorder's Kortex module).

```bash
cp ~/.cache/uv/git-v0/checkouts/00bab5f3d33091f1/36265a0c/prpl-tidybot/kortex_wheels/kortex_api-2.6.0.post3-py3-none-any.whl ~/
uv pip install --python /usr/bin/python3 --no-deps --target ~/kortex_isolated ~/kortex_api-2.6.0.post3-py3-none-any.whl
```

To remove it completely: `rm -rf ~/kortex_isolated`.

If the cached wheel is gone, Kinova hosts the same file:
`https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl`

**A2. Bring up the camera + TF with the arm driver DOWN** (T1):

```bash
cd ~/feeding-deployment/microwave/sheppy
sheppy up --manifest sheppy-manifest.yaml teleop-record     # wrist camera, camera window, robot_state_publisher, hand-eye calib
sheppy status
```

**A3. Prove it works while you teleop** (T2). Move the arm with the joystick while this runs.
The numbers must change, nothing may error, and the TF line must print a camera position:

```bash
PYTHONPATH=~/kortex_isolated:$PYTHONPATH python3 -u scripts/session/record_teleop_demo.py --check --tag elbow_close
```

**A4. Record** (T2). Teleop the whole motion and tag the keypoints as you go:

```bash
PYTHONPATH=~/kortex_isolated:$PYTHONPATH python3 -u scripts/session/record_teleop_demo.py --tag elbow_close --note "door open ~90, microwave at <where>"
#   start<Enter>  behind_door<Enter>  push_start<Enter>  shut<Enter>  ...  q<Enter>
```

Useful options:

- `--frame-hz 1` for fewer frames
- `--all-frames` to also record while the arm is still
- `--no-camera` for joints only
- `--out DIR`
- `--camera-ns /camera` only for the old launch stack

**A5. Give the arm back to the driver** (T1):

```bash
cd ~/feeding-deployment/microwave/sheppy && sheppy up --manifest sheppy-manifest.yaml real-arm-camera
```

Send me the `~/captures/teleop_<time>/` folder name; the tagged joint configurations in
`events.jsonl` are what the elbow-push plan will be built from.

**Untested under sheppy:** `--check` (A3) is the test. If the TF line fails, check
`ros2 topic echo --once /joint_states` shows `joint_1`..`joint_7`, and check
`sheppy logs arm_tf`.

---

## Part B — testing the close code (low speed)

### B1. Bring-up (T1)

```bash
source ~/feeding-deployment/microwave/sheppy/env.sh
cd microwave/sheppy && sheppy up --manifest sheppy-manifest.yaml real-arm-camera   # camera + arm driver + TF + calib + view
sheppy status                                                                         # every node running
```

### B2. Read-only checks (T2, nothing moves)

```bash
ros2 topic hz /joint_states                                    # ~ steady rate; Ctrl-C after a few lines
ros2 topic hz /wrist_camera/color/image_raw                    # ~15 Hz
ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame   # must resolve (the calib chain is up)
ros2 topic echo --once /gripper_state                          # note whether the gripper is open
```

### B3. Get the arm to the close's start pose

The close starts where the push-open ends, and refuses unless the hand is within 6 cm of that
point. The reliable way there is to run the open first (door CLOSED at the start), slow:

```bash
python3 -u microwave/door_task.py open --speed 0.3                 # dry run: detect, plan, print -- nothing moves
python3 -u microwave/door_task.py open --speed 0.3 --execute       # grasp, 50-deg swing, release + push it further open
```

### B4. Close: dry run first (nothing moves)

```bash
python3 -u microwave/door_task.py close --speed 0.3                                   # as on 10-07 (default)
python3 -u microwave/door_task.py close --speed 0.3 --push-offset                     # push 10 cm further right
python3 -u microwave/door_task.py close --speed 0.3 --close-wrist-side right          # wrist right of the elbow
```

Check the output:

- `hand now ..., X cm from the demo's start`: X must be ≤ 6.
- `all 5 legs pass`. With `--close-wrist-side right` the planning takes ~30 s.
- It ends with `DRY RUN -- nothing commanded` and `CLOSE OK (dry run)`.

In sim (10-21), `--close-wrist-side right --push-offset` together is refused at the end of the
push: no wrist-right configuration reaches it. The other three combinations plan.

Known refusal: the close's start pose sits at the J4 guard (144°). If it refuses on J4, note
the printed joints and stop. Don't loosen guards on the robot.

### B5. Close: execute slowly (hand on the e-stop)

```bash
python3 -u microwave/door_task.py close --speed 0.3 --execute                         # add the same flags as the dry run
```

What happens:

1. It re-plans from where the arm actually is.
2. It moves at 30% speed: up, left over the door, down beside the microwave, push shut.
3. It retracts: 15 cm back off the door, then to the park pose.

Watch:

- the elbow and wrist passing over the open door
- the bracelet meeting the door at the end
- whether the door **latches** (write it down: there's no sensor for it)

Go up in speed only after a clean run: `--speed 0.5`, then `1.0`.

### B6. After each run

- Run log: `ls -t ~/microwave_runs/ | head -1`. It holds:
  - `run.json`: events, door used, every trajectory goal (commanded joints, ok or not)
  - `arm.npz`: joints/torques at 25 Hz
  - RGB-D frames at the start and after the close
- Note the latch result and anything odd. Send me the run folder name.
- Move the arm somewhere safe (or `door_task.py retract --speed 0.3 --execute`).

### B7. Shutdown (T1)

```bash
cd ~/feeding-deployment/microwave/sheppy && sheppy down
```

---

## Option flags

`microwave/door_task.py {open|close|full|retract}`

| Flag | Values | Default | What it does |
|---|---|---|---|
| `--execute` | — | off (dry run) | Command the arm. Without it everything is planned and printed only. |
| `--speed` | 0–1 | 1.0 | Scales every arm motion's speed (Cartesian 9 cm/s and joint 29°/s peaks at 1.0). |
| `--close-wrist-side` | `left`, `right` | `left` | `left`: the close as run on 10-07 (plain IK from the current joints; the wrist ends left of the elbow). `right`: a planned joint path that keeps the wrist right of the elbow (~30 s planning). |
| `--push-offset` | — | off | Use `demos/push_close_2026-10-07_right10.door.json`: the push (kp5, END) 10 cm further along the door toward the handle. Off = the original 10-07 encoding. |

The close always uses the door the open task detected (`~/.microwave_door.json`), so run the
open first in the same session (or at least without moving the wheelchair or the microwave).

---

## Known issues to watch

- **J4 guard at the close start.** Every IK solution for the 10-07 close's start pose has
  |J4| ≈ 144–146°, right at the guard, so planning can refuse there.
- **Clearance with `--push-offset`** was 2.2 cm in sim (minimum 2 cm), against the demo's
  recorded door. The dry run re-checks it against the real door file.
- **The door angle isn't measured.** The close assumes the door is open no more than 110°.
- **The retract was refused on 10-07** (2.9 cm clearance at its first step). If it refuses
  again, the arm stays where the close ended. Teleop it away (driver down first).
