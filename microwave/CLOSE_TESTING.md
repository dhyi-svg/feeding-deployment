# Microwave close — how to test

This is the close that latched the door on 10-21: the route-over push-close in
`real_gen3_ros2_close_microwave.py --phase push-close` (code: `door_push.push_close`). It runs after the open.

## Stop the robot

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

## Commands

Run this first in every terminal:

```bash
source ~/feeding-deployment/microwave/sheppy/env.sh     # also cd's to the repo root
```

| Step | Terminal | Command | What it does |
|---|---|---|---|
| 0 | — | Teleop, with sheppy down (nothing running) | **Point the arm at the microwave** (door closed). Nothing in the code does this; it is always placed by hand. Every open on 10-21/22 started with the hand ~45 cm out, ~20 cm right of the arm's centre line, ~25–30 cm up, and the camera facing the microwave square-on, tilted ~15° down. The whole door (all four edges) must be in the camera window. Keep J4 around −130 to −135°: a start at −146.6° made the open refuse every grasp. |
| 1 | T1 | `sheppy up --manifest microwave/sheppy/sheppy-manifest.yaml real-arm-camera && sheppy status` | Starts the arm driver, camera, TF and the camera window (teleop is off from here on). Every node should show as running. |
| 2 | T2 | `python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --execute` | **Open, once.** Detects from where the arm is, so it must still be at the step 0 pose. Saves the door to `~/.microwave_door.json`, then grasps, swings 50°, lets go and pushes the door to about 90°. |
| 3 | T1 | `sheppy down`, then teleop to grab the container, then the step 1 command again | **Grip the container.** |
| 4 | T2 | `python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --route-over --keep-orientation --door-deg 90 --hold-offset-x -0.152 --push-radius 0.17 --push-radius-end 0.15 --over-above 0.18 --push-z 0.22 --push-contact-nm 6 --execute` | **Close** while holding the container (the 10-21 command that latched). Drop `--execute` for a dry run that plans and moves nothing. |
| 5 | — | Open the door by hand to ~90°, then run step 4 again. | **Repeat the close** as often as you like. The arm starts where the last close left it, and the container stays in the gripper. |
| 6 | T1 | `sheppy down` | Shutdown. |

Every close uses the door saved by the open in step 2; `--door-deg 90` tells it how far the door is open. If the
microwave or the wheelchair moves, run step 2 again first: on 10-21 a moved microwave left the door unclosed.

## What to expect

**Prompts:** none. Neither script asks for input; each runs straight through once started.

**Open** (expected):
1. Two camera looks, then a plan.
2. Moves to 25 cm in front of the handle, goes straight in and closes the gripper.
3. Swings the door 50°.
4. Lets go, goes up, over to the right, down and in, and pushes the door further open.
5. Ends beside the door with the gripper open.

**Open stops before any motion if:**
- detection fails, or two looks don't agree within 3 cm
- the handle position or the door size is implausible
- no grasp + swing passes the sim checks

**Open stops partway if:**
- the grasp lands more than 3 cm off. It stops with the gripper untouched.
- the gripper reads open after closing. Nothing is grasped, so it doesn't swing.
- the swing re-check fails. It stops still holding the handle.

**Close** (expected):
1. Plans everything first.
2. Goes up to 18 cm above the door top, left past the hinge, down to z 0.22, then right and forward behind the door.
3. Swings the door shut, stopping 3 cm short of the detected closed position.
4. Pushes in one slow continuous move (~1.2 cm/s), at most 3.5 cm.
5. Stops the push when the joint torque jumps by more than 12 Nm. It prints `door shut ... push stopped`.
6. Backs off the door, then backs out up to 20 cm.
7. Prints `PUSH-CLOSE DONE`.

The gripper is never touched.

**Close stops before any motion** if no plan passes every check (`no plan passes every gate -- refusing, arm
untouched`). It is usually J4: lower `--push-radius` / `--push-radius-end` by 2 cm and dry-run again.

**Close stops partway, leaving the arm where it is, if:**
- a leg doesn't reach its planned joints (`... -- stopping here`)
- any joint lags its path by more than ~9° (the driver aborts)

**Watch for:**
- `reached the planned maximum without a clear contact`: the door may not be shut. Check it by eye.
- The container is not in the collision model. Watch it as it passes over the door top.
- Whether the door latched. Nothing measures it.

Each run prints everything it does. The open also saves its camera frames to `~/microwave_detections/`.
