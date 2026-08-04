# Pachirisu bring-up runbook (microwave-door task)

Practical step-by-step commands for running this repo's microwave-door-opening stack
on `Pachirisu` (RTX desktop, Ubuntu 24.04 host) against the real Kinova Gen3 at
`192.168.1.10`. This is a runbook, not a narrative — for *how we got here* and
*what's proven vs. not*, see `CLAUDE.md`'s "CURRENT STATE / NEXT STEP" block and
`TESTING_LOG.md`. For the (separate, Jetson-only) single-venv rig, see
`LOCAL_DEPLOYMENT.md`.

Last written: 2026-08-04.

---

## 0. Before you start

 - Check `nvidia-smi` before assuming the GPU is
  free — another lab member's long-running jobs (e.g. `serve_policy.py`,
  `kinova_infra.control_node.*`) may already be using it. Don't kill other users'
  processes.
- **Only one process can hold the arm** (`/tmp/kinova.lock` inside the container). A
  stale lock from a dead process is auto-cleared on the next `arm_server.py`/`kinova.py`
  start.
- **Physical e-stop is the only real stop** — `bulldog_bypass.py` has no software
  e-stop; if it dies, the arm faults within ~1s (heartbeat lost). That's intentional
  and is the one retained safety property.

---

## 1. Start the container

The container (`feed-noetic`, from image `feed-noetic-snapshot3`) already exists with
`ros_env` (RoboStack conda, Python 3.11), the Kortex SDK, the perception deps
(torch/GroundingDINO/etc.), and `/opt/msgs_ws` all pre-built into its writable layer.
Normally you just **start the existing container** — do not `docker run` a fresh one,
that would lose all of the above.

```bash
docker start feed-noetic
docker exec -it feed-noetic bash
```

<details>
<summary>Fallback: recreating the container from scratch (only if it's gone/corrupted)</summary>

```bash
docker run -it --network host --privileged --gpus all \
  -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/deployment_ws:/root/deployment_ws \
  --name feed-noetic feed-noetic-snapshot3 bash
```

Flags matter and were each added after a real failure (see `TESTING_LOG.md`,
"native install blocker" / 2026-07-21 entry):
- `--privileged` — camera/USB access (without it, `/dev/video*` is invisible).
- `--gpus all` — attaches the RTX 5070; easy to forget on a copy-paste recreate, check
  with `nvidia-smi` inside the container immediately after.
- `-v /dev:/dev` — `--privileged` alone snapshots `/dev` at container start; without
  this live bind-mount, a USB replug/re-enumeration (which happens) leaves the
  container serving a stale, dead device node.
- `-v ~/deployment_ws:/root/deployment_ws` — bind-mounts the repo (and the
  host-persisted calibration/detection-trial dirs alongside it) so nothing there lives
  only in the container's writable layer.

If you do recreate, re-run `docker commit feed-noetic feed-noetic-snapshotN` afterward
so the rebuilt env/packages survive a future `docker stop`.
</details>

---

## 2. Inside the container: activate the environment

Every shell you open in the container needs **both** of these, in this order — the
second is a real, previously-undocumented-until-2026-07-31 gap: `ros_env` alone is
missing `feeding_deployment_msgs` (a separately-built catkin message package), which
any HLA import (`feeding_deployment.actions.base` etc.) needs transitively via
`arm_client.py` → `safety/collision_threshold.py`.

```bash
conda activate ros_env
source /opt/msgs_ws/devel/setup.bash
export ARM_RPC_HOST=127.0.0.1
cd /root/deployment_ws/src/feeding-deployment
```

Sanity check (optional, cheap):
```bash
python -c "import rospy, feeding_deployment_msgs, feeding_deployment; print('ok')"
```

---

## 3. Bring up the stack (one terminal/`docker exec` per step, in order)

Each of these blocks and should run in its own terminal (`docker exec -it feed-noetic
bash`, then repeat step 2's env setup in that shell too).

| # | What | Command | Notes |
|---|---|---|---|
| 1 | ROS master | `roscore` | needed for camera + any rospy-based code |
| 2 | Camera (optional — only if you need vision) | `roslaunch realsense2_camera rs_camera.launch align_depth:=true` | `align_depth:=true` is needed for `RealSenseInterface`/`detect_handle_and_placement`'s 3D pipeline. Has intermittently crashed the USB controller in past sessions (`TESTING_LOG.md` — "recurring USB controller failures"); not fully root-caused, watch `dmesg` for `xhci_hcd ... assume dead` if the feed dies. |
| 3 | Arm RPC server | `python -u src/feeding_deployment/control/robot_controller/arm_server.py` | connects to the real arm, clears faults, holds position — **no motion yet**. Binds `127.0.0.1:5000`. `-u` avoids a stdout-buffering blind spot. |
| 4 | Stub base server | `python scripts/stub_base_server.py` | no-op base on `127.0.0.1:5001` so bulldog's handshake passes (there's no real base on this platform). |
| 5 | Bulldog bypass (**unlocks motion**) | `python scripts/bulldog_bypass.py` | flips `bulldog_ready`, heartbeats `is_alive()`. Real `bulldog`/rospy watchdog is untested on this box — bypass is still what's used. |

Steps 1–2 are only needed if you're doing live perception. Steps 3–5 are needed for
any real arm motion; skip 4–5 (arm server only) for read-only telemetry.

### After any e-stop / Xbox teleop takeover
The Kortex session faults on the next call. Recover by:
1. Restart `arm_server.py` (reconnects, clears faults, holds — no motion).
2. Re-run `bulldog_bypass.py` (a fresh server re-locks motion, so the bypass must be
   re-run too, not just the server).
3. Verify with a `get_state()` read — expect `ARMSTATE_SERVOING_READY`.

---

## 4. Useful one-off scripts (`scripts/scratch/`)

Not part of the bring-up sequence — run standalone, after step 3 (and 1–2 if they need
the camera), from a shell with step 2's env active.

| Script | What it does |
|---|---|
| `sim_open_microwave_hla_dryrun.py` | Sim-only, zero-hardware-risk: ticks the *real* `OpenDoorHLA`/`open_microwave.yaml` behavior tree end-to-end with `robot_interface=None` + `NullSimulator` + a stub perception adapter. Validates control flow, not real poses. Safe to run any time steps 1–5 are *not* even up. |
| `test_detect_handle_and_placement.py` | Real `detect_handle_and_placement()` against the live camera, monkeypatching the camera→arm-base transform from the saved calibration (no tf2). Needs steps 1–3 (camera + arm server; motion not required). |
| `capture_calib_sample.py` / `run_calibrate_hand_eye.py` | Re-run the eye-in-hand calibration (ArUco board) if it ever needs redoing. Needs steps 1–3 and the physical 12-marker board in place of the microwave. |
| `run_door_arc_step.py` | One-waypoint-per-call door-arc script (seeded IK + joint-space-delta guard). **Real arm motion** — needs steps 1–5, hand on e-stop, verify each waypoint in sim first. |

Saved artifacts these depend on (host-persisted, outside the container, under
`~/deployment_ws/`):
- `pachirisu_wrist_camera_calib/wrist_camera_calib_v2.json` — current eye-in-hand
  calibration (PARK method; validated to 5.3cm on real hardware).
- `pachirisu_detection_trials/`, `pachirisu_handle_detection/` — saved detection
  images from past sessions, for reference/debugging only.

---

## 5. Shutdown (every session — do this before walking away)

Reverse order of bring-up, SIGINT (`Ctrl-C`) each, then the container:

```text
bulldog_bypass.py  →  stub_base_server.py  →  arm_server.py  →  realsense2_camera  →  roscore
```
then, from the host:
```bash
docker stop feed-noetic
```

`arm_server.py`'s SIGINT does a graceful Kortex disconnect and releases
`/tmp/kinova.lock` — don't `kill -9` it if avoidable. The container itself doesn't need
to be stopped for a same-day break (it's cheap to leave idle — nothing runs
automatically inside it), but **do** stop it before leaving the machine unattended for
longer, and always if the arm is left gripping/mid-motion (physically resolve that
first — see `CLAUDE.md` for how to reclaim control after e-stop).

---

## Next steps (as of 2026-08-04, from `CLAUDE.md`)

1. **Wire real live detection into the proven duck-typed HLA adapter.** Still
   sim-only/no-motion: replace `sim_open_microwave_hla_dryrun.py`'s stub's hardcoded
   poses with a real `detect_handle_and_placement()` call, using the same
   monkeypatched-transform pattern already proven in `test_detect_handle_and_placement.py`.
2. **Model this microwave's real hinge kinematics.** The door visibly rises as it
   opens (not a pure single-vertical-axis pivot — a demonstrated real waypoint was off
   by +10.5cm y / +5.8cm z from the current formula). Either have the user demonstrate
   a few more real waypoints by hand and interpolate/fit directly (low-risk, working
   well so far), or investigate the physical hinge mechanism for a correct parametric
   model.
3. **Root-cause the 07-30 wrong-direction door-arc incident** (leading, unconfirmed
   hypothesis: a stale hinge-offset transferred across a manual teleop nudge that
   changed orientation) before attempting the arc again.
4. Only once 1–3 are solid: attempt a full live detect → grasp → open through the real
   `OpenDoorHLA` path (not just the scratch scripts) with motion enabled.
5. **Separately, on the Jetson rig:** finish the arc-open with a solver that tries
   seeded IK first and falls back to unseeded when `ik_err > 2cm` (guards against the
   big-joint-jump/wrist-flip failure mode already seen there).
