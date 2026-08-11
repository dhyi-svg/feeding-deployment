# Jetson setup — every command, cold box → autonomous microwave open

Single-machine rig: **Jetson Orin Nano + Kinova Gen3 plugged in directly**, no NUC,
no base, no lab safety wiring. This is the *operational* file — the ordered list of
commands to bring the rig up and run the microwave task.

Companions (do not duplicate them here):
- `LOCAL_DEPLOYMENT.md` — what works / what's broken, and why (the reference).
- `docs/microwave_ros2_runbook.md` — the live run **checklist** (tick rungs off during a run).
- `TESTING_LOG.md` — the blow-by-blow hardware history.
- `CLAUDE.md` — current state / next step.
- `PACHIRISU_SETUP.md` — the same thing for the **other** rig (RTX/RoboStack container).
  Different box, different env, same arm — don't cross the commands over.

Last updated: 2026-08-04. Everything below has been run on this box; the sequence in
§6 completed a full autonomous door open (~91°) on 2026-07-29.

---

## 0. Facts about this box

| Thing | Value |
|---|---|
| Box | Jetson Orin Nano 8 GB, JetPack 6.2 / L4T R36.4.7, CUDA 12.6, aarch64 |
| System Python | 3.10.12 (`/usr/bin/python3`) — owns `ros2`/`lark` |
| Project env | `~/feeding-deployment/.venv` — owns the repo, torch, GroundingDINO, `rclpy` |
| Kortex SDK | **not** in `.venv` — user site, `~/.local/lib/python3.10/site-packages` |
| ROS | ROS 2 **Humble** (distro install only; `~/ros2_ws` belongs to other projects — off-limits) |
| Arm | Kinova Gen3 7-DOF, `192.168.1.10`; this box `192.168.1.18` on `enP8p1s0` |
| Camera | Intel RealSense D435i, eye-in-hand, via `realsense2_camera` |
| Hand-eye calib | `~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib` |

Two interpreters, on purpose: `ros2 launch` needs `lark` (system python3), the repo
needs the venv. The launch file starts repo modules with the venv itself, so both
sides get what they need — nothing to install to make that work.

---

## 1. One-time install (already done on this box)

```bash
cd ~/feeding-deployment
python3 -m venv .venv
.venv/bin/pip install -e ".[robot, develop]"
./run_ci_checks.sh          # all-green in 5-10 s
```

Perception deps (`torch` 2.11 cu126 Jetson wheels, GroundingDINO, `open3d`,
`supervision`, `pyrealsense2`) are hand-installed into `.venv`; the GroundingDINO
weights live in `~/Grounded-Segment-Anything/`. **A `torch` reinstall wipes the
hand-copied `libcudss.so.0` shim in `torch/lib/`** — re-copy it if imports break.

---

## 2. Session environment

Paste this into every terminal that talks to the arm:

```bash
cd ~/feeding-deployment
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$HOME/feeding-deployment/src:$PYTHONPATH"
export ARM_RPC_HOST=127.0.0.1        # else arm_interface.py aims at the lab NUC
export CUDA_VISIBLE_DEVICES=""       # GroundingDINO: CPU only (see §8)
PY=$HOME/feeding-deployment/.venv/bin/python
```

---

## 3. Hardware present (READ-ONLY)

The two most common blockers, both invisible from software until you look:

```bash
cat /sys/class/net/enP8p1s0/carrier     # want 1
ping -c1 192.168.1.10                   # want a reply
lsusb | grep -i intel                   # want an Intel device (the RealSense)
ps aux | grep -E "[h]ome_launch|[a]rm_driver"   # want nothing: only one process may hold Kortex
                                               # (brackets stop grep matching itself)
```

---

## 4. Arm servers (READ-ONLY — no motion yet)

Order matters. Each in its own terminal (or `tmux` pane) with §2's env.

```bash
pkill -f arm_server.py ; pkill -f bulldog_bypass.py ; pkill -f stub_base_server.py
rm -f /tmp/kinova.lock          # only if no live process holds the arm

$PY src/feeding_deployment/control/robot_controller/arm_server.py   # 1st: connect, clear faults, hold
$PY scripts/stub_base_server.py                                     # 2nd: no-op base for bulldog's handshake
$PY scripts/bulldog_bypass.py                                       # 3rd: unlocks motion, heartbeats
```

- **Restart, never reuse, a long-lived `arm_server.py`** — one had wedged at 99.7 % CPU
  with a dead Kortex session (2026-07-29).
- **The bypass has no software e-stop. The physical e-stop is the only stop.** If the
  bypass process dies the arm e-stops within ~1 s (the one retained safety property).
- Check `gripper_pos` before anything moves (0.009 = open, ~1.0 = closed). Note it
  saturates at ~1.0 whether or not something is between the fingers — it cannot tell
  you a grasp succeeded.

Zeroing torque offsets (`robot_controller/kinova.py`) is **interactive** and only
needed before inside-mouth transfer — not for the microwave task.

---

## 5. ROS 2 bring-up (READ-ONLY)

```bash
# system python3 — NOT the venv (needs lark)
ros2 launch launch/ros2/microwave_bringup.launch.py
```

Starts `robot_state_publisher`, the `/joint_states` bridge off the arm RPC, the static
hand-eye calibration TF, and `realsense2_camera` with `align_depth.enable:=true` at
640x480x15 (the driver default stalls the colour stream on this rig).

Verify **before** any motion:

```bash
ros2 topic hz /joint_states                                  # ~50 Hz
ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame
ros2 topic hz /camera/aligned_depth_to_color/image_raw        # depth alive, ~14 Hz
```

A wrist-mounted camera should read on the order of `[0.255, 0.018, 0.565]` at a
typical pose. Wildly different ⇒ calibration or joint bridge is wrong — stop.

**Frames on this rig (`arm_base_link`): +x forward/away from the arm, +y left, +z up.**
Any approach standoff must be at *smaller* x than the target.

---

## 6. The microwave task

Corrections are **off by default** and every motion script refuses to run without them:

```bash
export HANDLE_DEPTH_CORR=0.094      # fixed offset, measured on the tf2 path 2026-07-29
export HANDLE_LAT_CORR=0.0          # lateral error is detection VARIANCE, not bias
```

> Not `0.16` — that belongs to the old calib+FK path and undershoots by ~6 cm here.
> Re-measure after any recalibration, and re-touch the handle first: the microwave
> gets nudged a few cm by every door swing.

### 6a. Detection only (READ-ONLY)

```bash
$PY -m feeding_deployment.perception.appliance_perception.appliance_perception \
    --handle_type "microwave handle"
```

Expect the handle at ~0.55–0.68 confidence, ~34–85 s/frame on CPU. Two detections of
a stationary handle should agree to ≲1 cm.

### 6b. Approach — dry run, then move (MOTION)

```bash
$PY scripts/real_gen3_ros2_approach_microwave.py              # dry run: computes + gates, no motion
$PY scripts/real_gen3_ros2_approach_microwave.py --execute    # one joint-space move to the standoff
```

Never closes the gripper, never touches the door.

### 6c. Grasp (MOTION)

```bash
$PY scripts/real_gen3_ros2_grasp_microwave.py                 # dry run
$PY scripts/real_gen3_ros2_grasp_microwave.py --execute       # detect -> pre-grasp -> grasp -> close
```

Detects **once, from ~39 cm** and caches the pose — at ~2 cm the wrist camera cannot
see the microwave and the plane fit returns a handle ~7 cm off. Gated on two
detections agreeing within 3 cm plus a loose plausibility box (no ground truth: it
goes stale as the appliance drifts).

**Then pause and check the grip by hand before opening.** `gripper_pos` cannot
confirm it.

### 6d. Door arc (MOTION — this one moves immediately, no `--execute`)

```bash
$PY scripts/real_gen3_ros2_open_arc_microwave.py
```

Repo geometry (`_generate_door_arc_waypoints`, `arc_length_m=0.55`, spacing `0.05`,
`direction=-1`, `rotate_orientation=True`) about the **assumed** `+0.32 m` hinge —
validated to 3.7 mm radius preservation over a 90.8° sweep. The *perceived* hinge is
wrong-side at multiple viewpoints; do not switch to it.

Plans every waypoint first (seeded IK, falling back to unseeded when it stalls), then
executes with a settle-wait and a 3 cm per-waypoint tracking abort. A stop at the last
waypoint is the door's own limit, not a fault.

Throughout: `set_speed("low")`, hand on the physical e-stop, sweep forward/away from
the base, keep z raised if another project's camera rig is under the wrist.

---

## 7. Shutdown and recovery

```bash
kill -INT <arm_server pid>     # graceful close -> Kortex disconnect -> /tmp/kinova.lock released
# then stop the bypass and the stub base
```

**After an e-stop or Xbox teleop:** the Kortex session faults
(`ERROR_PROTOCOL_SERVER` / `INVALID_USER_SESSION_ACCESS` on the next call). Restart
`arm_server.py`, then **re-run `bulldog_bypass.py`** — a fresh server re-locks motion.

---

## 8. Known walls on this box

| What | Symptom | Live with it by |
|---|---|---|
| GroundingDINO on GPU | NVML assert *or* genuine OOM — they trade off | `CUDA_VISIBLE_DEVICES=""`, CPU |
| IKFast / `plan_to_ee_pose` | hangs / never converges | PyBullet native IK + `set_joint_position` |
| Cartesian `set_ee_pose` | aborts at extended configs | joint-space moves everywhere |
| `rospy` / real `bulldog` | Noetic vs Ubuntu 22.04 | perception ported to ROS 2; bulldog bypassed |
| ViT-H (SAM) | CUDA OOM on 8 GB shared RAM | lazy-loaded; the appliance path never needs it |
| RealSense at default profile | colour stream stalls after minutes | 640x480x15, set in the launch file |

Offline validation, no arm and no camera:

```bash
$PY scripts/generate_microwave_poses_offline.py --log_dir <dir>   # real geometry, detector stubbed
$PY scripts/validate_open_microwave_hla_sim.py  --log_dir <dir>   # real HLA; stops at the IKFast wall
```

---

## 9. Next steps

In priority order.

1. **Release and retract without loading the door.** The one gap left in the proven
   sequence: release *first*, then clear laterally, then back out — never withdraw
   while gripping. Currently done by hand after the arc.
2. **Fold the sequence into `OpenDoorHLA.open_microwave()`** so the HLA runs
   unmodified instead of three standalone scripts. Two known obstacles:
   the hinge heuristic (the repo perceives it; only the assumed `+0.32 m` works here),
   and `move_to_ee_pose_trajectory`'s cartesian path (cartesian aborts at extended
   configs on this arm — the scripts use per-waypoint joint moves).
   Start from Pachirisu's 2026-07-31 result: the real `OpenDoorHLA` ticks end-to-end
   against a ~15-line duck-typed `perceive_handle_opening_poses` stub with
   `robot_interface=None` + `NullSimulator` — no `PerceptionInterface`, no
   `netft_rdt_driver`. That adapter shape is the way in; its *poses* are placeholders,
   not values to reuse. (`scripts/scratch/sim_open_microwave_hla_dryrun.py`.)
3. **Re-measure `HANDLE_DEPTH_CORR` against a fresh touched ground truth**, and check
   whether the microwave has drifted, at the start of any session that will grasp.
   The appliance moves a few cm per door swing; absolute references go stale fast.
4. **Detection speed.** ~85 s/frame on CPU is the pipeline's floor. Two untried
   levers: downscale the frame before detection (~4x less activation memory, faster
   too, but coarsens the handle centroid — validate against a touched truth), or a
   Swin-T checkpoint instead of Swin-B (not on disk; a download).
5. **`press_start_button` on hardware.** The local GroundingDINO backend
   (`BUTTON_BACKEND=auto|grounding_dino|molmo`) removes the Molmo/lab-network
   dependency but has only been exercised in `tests/test_button_detection.py`. The
   wrist camera is upside down — the row/side rule runs in the flipped frame.
6. **Staging joint configs are lab values.** `left_back_retract_pos`,
   `fridge_door_staging_pos`, `left_retract_pos` are mounting-specific; verify each on
   this rig before the HLA path trusts them.
7. **FastDownward (`FD_EXEC_PATH`)** for a real PDDL planner solve — the domain and
   problem already serialize as valid PDDL.
