# Local (single-machine, arm-only) deployment — what works

Running the EmPRISE feeding stack on **one Jetson Orin Nano with a Kinova Gen3 plugged
in directly** — no NUC, no base, no lab safety wiring, no ROS. This documents what's
been verified to work on this box, the exact commands, and what's broken/blocked.

> Reference lab setup (compute box + NUC + Cornell cluster) is in `README.md`. This
> file is the *deviations* for the single-machine rig. Contains machine-specific
> values (IPs, user-site paths) — not intended for `upstream`.

Last updated: 2026-07-14.

---

## Environment

| Thing | Value |
|---|---|
| Box | Jetson Orin Nano, 8 GB, JetPack 6.2 / L4T R36.4.7, CUDA 12.6, aarch64, Python 3.10 |
| Main env | `~/feeding-deployment/.venv` |
| Arm | Kinova Gen3 7-DOF at `192.168.1.10` (Kortex API port 10000). Box is `192.168.1.18`. |
| Kortex SDK | **not** in `.venv` — installed in user site (`~/.local/lib/python3.10/site-packages`) |
| Perception deps | `.venv` has GroundingDINO + SAM checkpoints (`~/Grounded-Segment-Anything/`), `open3d`, `pyrealsense2`, `sklearn` |
| Camera | Intel RealSense **D435I** (color+depth) — grabbed directly via `pyrealsense2`, **no ROS** |
| YOLO (compare) | `~/yolo_env` (bottle project) — ultralytics + **Jetson-native torch, GPU works** + COCO `.pt`. Read-only use only. |

**Command prefix** for anything that talks to the arm (pulls in the Kortex SDK and
points the RPC host at localhost instead of the hardcoded lab NUC):

```bash
E="PYTHONPATH=$HOME/.local/lib/python3.10/site-packages ARM_RPC_HOST=127.0.0.1"
PY=$HOME/feeding-deployment/.venv/bin/python
```

> `ARM_RPC_HOST` requires the local edit to `arm_interface.py` (env-var override of
> `NUC_HOSTNAME`, default still the lab NUC). **Uncommitted** as of 2026-07-14.

---

## ✅ Works

### Perception / sim (no arm needed)
| Step | Command | Verified | Notes |
|---|---|---|---|
| torch + CUDA on GPU | `$PY -c "import torch; print(torch.cuda.is_available())"` → `True` | 2026-07-09 | torch **2.11.0 (cu126)** + torchvision 0.26 Jetson wheels + hand-copied `libcudss.so.0` into `torch/lib/`. A torch reinstall wipes the shim. |
| SAM loads lazily | (import `GroundedSAM`) | 2026-07-08 | ViT-H deferred to first use; appliance/microwave path uses GroundingDINO boxes only. Commit `88d49475`. |
| draw arc in sim | `$PY scripts/draw_microwave_waypoints.py --mode direct` | 2026-07-08 | door-opening arc geometry, screenshot + pkl template. Commit `67e3bd09`. |
| drive Gen3 arc in sim | `$PY scripts/sim_gen3_open_microwave.py --mode direct` | 2026-07-08 | forward-facing grasp + arc, 0.1 mm tracking, gripper close. Uses **PyBullet native IK** (not the repo's IKFast). |

### Real arm — setup (no motion)
| Step | Command | Verified | Notes |
|---|---|---|---|
| zero torque offsets | `cd .../robot_controller && $E $PY kinova.py` | 2026-07-09 | **interactive** (Enter prompts + physical positioning) — run it yourself in a terminal. Single-instance lock `/tmp/kinova.lock`. |
| arm RPC server | `$E $PY src/feeding_deployment/control/robot_controller/arm_server.py` | 2026-07-09 | serves on `127.0.0.1:5000`; connects + clears faults + holds position (no motion). |
| stub base server | `$E $PY scripts/stub_base_server.py` | 2026-07-09 | no-op base on `127.0.0.1:5001` so bulldog's handshake passes. Commit `a8657982`. |
| bypass (unlock motion) | `$E $PY scripts/bulldog_bypass.py` | 2026-07-09 | flips `bulldog_ready`, heartbeats `is_alive()`. **No software e-stop** — physical only. Commit `a8657982`. |

### Real arm — motion ladder (low speed, hand on e-stop)
| Rung | What | Verified | Notes |
|---|---|---|---|
| 1 | connect + fault-clear (`get_state()`) | 2026-07-09 | reads joints/EE over RPC |
| 2 | read-only telemetry | 2026-07-09 | `position`/`velocity`/`ee_pos`/`effort`, radians |
| 3 | joint move (+5° on J4) | 2026-07-09 | `set_joint_position`, exact, other joints ~0 |
| 4 | cartesian move (+5 cm z) | 2026-07-09 | `set_ee_pose(pos, quat)` works for **small** moves |
| 5 | door-opening arc (real arm) | ✅ 2026-07-14 | **joint-space** (sim-IK → `set_joint_position`) is reliable; forward + lateral swing completed. Cartesian aborts at extended configs (see gotchas). |

### Perception — live, rospy-free (2026-07-14)
| Step | How | Notes |
|---|---|---|
| grab color+depth | `pyrealsense2` direct (`rs.align`) | no ROS camera interface needed |
| detect microwave | GroundedSAM / GroundingDINO (**CPU**) | `microwave` box 0.59–0.81; open-vocab. ~34 s CPU. |
| box → 3D | deproject depth + intrinsics | control panel → valid 3D; glass door → no depth |
| **handle in arm-base frame** | faithful mimic of `detect_handle_and_placement` | box → point cloud → `segment_plane` → protruding cluster (DBSCAN) → centroid → **camera→arm_base via static extrinsic + live EE pose** (replaces `tf2`). Hinge-consistent result, no rospy. |

> **YOLO doesn't work for the microwave here:** nano COCO models (`yolov8n`→"bus", `yolo11n-seg`→missed, `yolo26n-seg`→"train") mis-ID or miss it. `-seg` models need a warmup pass. GroundingDINO is the one that gets it. Seg is **not needed** anyway (appliance path is boxes only).

---

## ❌ Broken / blocked

| What | Symptom | Root cause | Workaround |
|---|---|---|---|
| IKFast (repo sim IK) | hangs indefinitely | IKFast build fails on this custom Gen3 URDF (aarch64) | PyBullet native `calculateInverseKinematics` |
| `plan_to_ee_pose` (sim cartesian) | never converges | needs a stepping/real-time loop + IKFast | native IK + `resetJointState` for kinematic playback |
| `rospy` (bulldog, PerceptionInterface, tf2) | not installed | **ROS 1 Noetic repo vs ROS 2 Humble / Ubuntu 22.04 box** — Noetic only targets 20.04 | bulldog → **bypass**; camera → `pyrealsense2`; **tf2 → static extrinsic + arm FK** |
| bulldog | won't start | needs arm **and** base RPC servers up | stub base server + (real bulldog still needs rospy) |
| ViT-H on GPU | CUDA OOM (`NvMap error 12`) | 2.5 GB model + double-copy on 8 GB shared RAM | lazy SAM; use lighter SAM / bigger Jetson for the food path |
| GroundingDINO on GPU | `NVML_SUCCESS==r ASSERT` in torch allocator | Tegra iGPU lacks NVML/PCI interface torch expects | run detection on **CPU** (`CUDA_VISIBLE_DEVICES=""`), ~34 s/frame. (YOLO via `yolo_env`'s Jetson torch runs GPU fine.) |
| real-arm motion (all `set_*`) | `AssertionError: Bulldog is not running` | every motion method calls `_require_bulldog()` | bypass unlocks it |
| door arc rung 5 | swept inward toward base → e-stopped | arc was generated sweeping toward the base | **sweep forward/away from base**; verify reachability first |

---

## ⚠️ Gotchas & safety

- **Joint control >> cartesian on the real arm:** `set_ee_pose` (Kortex cartesian) aborts (`returns False`, no motion) at **extended/near-singular configs**; `set_joint_position` is reliable. For arcs, compute joint targets via **sim IK** (sim↔real joints aligned to 0.2 cm) and command joints. Also: a move can **return `False`/`True` before it settles and complete late** — always **wait for velocity≈0 then re-check the EE**, don't trust the return value.
- **Camera→arm frame without ROS:** the RealSense is eye-in-hand — static `arm_end_effector_link→camera_link` in `launch/sensors.launch:110` `xyz(-0.046,0.084,0.11) quat(0.707,0,0,0.707)`. Chain it with the live EE pose (`get_state`) to convert camera-frame 3D → arm-base, replacing `tf2`. (Exact accuracy needs hand-eye calibration: verify the EE frame + the ~1.5 cm color-sensor offset.)
- **Physical obstacle:** a camera rig for another project was mounted under the arm/wrist (now removable). When present, real-arm trajectories must stay **clear of the base/low region** — sweep forward and keep z up.
- **After any e-stop:** the Kortex session faults (`ERROR_PROTOCOL_SERVER` on the next call). Recover by restarting `arm_server.py` (reconnects + clears faults + holds — no motion). Then re-run the bypass (fresh server = motion re-locked).
- **Bypass has no software e-stop.** Physical e-stop is the only stop. If the bypass process dies, the arm e-stops within ~1 s (heartbeat lost) — that's the one retained safety property.
- **Single-instance lock** `/tmp/kinova.lock`: only one process can hold the arm. A stale lock (dead PID) is auto-cleared on the next `arm_server`/`kinova.py` start.
- **Clean shutdown:** `kill -INT` the arm server (graceful `close()` → Kortex disconnect → lock released), then stop stub base / bypass.

---

## Commits

- `88d49475` — SAM lazy-load (appliance path runs without ViT-H)
- `67e3bd09` — sim door-opening scripts + IKFast-wall note in `CLAUDE.md`
- `a8657982` — stub base server + bulldog bypass
- *uncommitted:* `arm_interface.py` `ARM_RPC_HOST` override

## TODO / untested

- **Hand-eye calibration** — verify the EE frame + camera offset so the perception→arm-base target is accurate enough to grasp (pipeline is done; numbers need trust).
- Confirm the protruding cluster is the actual **latch** vs the door bezel (this microwave has a subtle latch, not a fridge handle).
- Turn the rospy-free perception result into a real `handle_opening_pos.pkl` (feed the arc geometry the HLA expects).
- Full `open_microwave` HLA on hardware — needs the rviz `None`-guard fix at `open_door.py:202` (still **not** fixed) + wiring perception in without `PerceptionInterface`/rospy.
- FastDownward (`FD_EXEC_PATH`) for actual PDDL **planner solve** (domain/problem already serialize as valid PDDL).
- rospy/roscore path (or an x86 ROS 1 box) to run real bulldog + the full executive.
