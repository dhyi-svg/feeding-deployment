# Local (single-machine, arm-only) deployment — what works

Running the EmPRISE feeding stack on **one Jetson Orin Nano with a Kinova Gen3 plugged
in directly** — no NUC, no base, no lab safety wiring, no ROS. This documents what's
been verified to work on this box, the exact commands, and what's broken/blocked.

> Reference lab setup (compute box + NUC + Cornell cluster) is in `README.md`. This
> file is the *deviations* for the single-machine rig. Contains machine-specific
> values (IPs, user-site paths) — not intended for `upstream`.

> **Just want the commands?** `JETSON_SETUP.md` is the ordered bring-up
> (cold box → arm servers → ROS 2 → autonomous open) plus next steps. This file
> is the reference behind it: what works, what's broken, and why.

Last updated: 2026-07-21.

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

> `ARM_RPC_HOST` overrides `NUC_HOSTNAME` in `arm_interface.py` (default still the lab
> NUC). **Committed** as of 2026-07-21 (`2c0e3498`) — was an uncommitted local edit here
> before that.

---

## Alternate box: Pachirisu (RTX desktop, Ubuntu 24.04) — RoboStack + kortex_api

A second, separate rig from the Jetson above: `Pachirisu` connects to the **same physical
Gen3 arm** at `192.168.1.10`, but through a Docker container (`feed-noetic`,
`osrf/ros:noetic-desktop-full`, `--network host`) running a
[RoboStack](https://robostack.github.io/) conda env instead of a venv — this gets **real
`rospy` and the PRPL/py3.10+ deps into the same interpreter** (Python 3.11), which was
impossible on the Jetson (no ROS at all there). See the native-install-blocker entry in
`TESTING_LOG.md` (2026-07-21) for how the env was built, and the Pachirisu bring-up entry
(same date) for the read-only arm verification below.

| Thing | Value |
|---|---|
| Box | Pachirisu, RTX GPU, Ubuntu 24.04 host |
| Container | `feed-noetic` (`osrf/ros:noetic-desktop-full`, Ubuntu 20.04), `--network host` |
| ROS env | Miniforge `ros_env`, **Python 3.11**, `ros-noetic-ros-base` (robostack-staging) |
| Arm | same Kinova Gen3 at `192.168.1.10` as the Jetson rig. Box's ethernet (`enp4s0`) is `192.168.1.11/24` — already on-subnet, no static IP needed. |
| Kortex SDK | `kortex_api-2.8.0.post5-py3-none-any.whl` (pure-Python wheel, no version/platform tag) pip-installed into `ros_env` from Kinova's Artifactory — **pins `protobuf==3.20.0`**, a latent (benign so far) conflict with the repo's `google-generativeai`/`anthropic` deps, which want newer protobuf. |
| Extra apt dep | `iputils-ping` — `KinovaArm.__init__` shells out to `ping` as a pre-flight; missing from the base ROS image. |

**Command prefix** (inside the container, `conda activate ros_env` first):
```bash
export PYTHONPATH=/opt/msgs_ws/devel/lib/python3.11/site-packages:$PYTHONPATH  # feeding_deployment_msgs, built standalone -- see TESTING_LOG.md
export ARM_RPC_HOST=127.0.0.1
python -u src/feeding_deployment/control/robot_controller/arm_server.py  # -u / PYTHONUNBUFFERED=1: avoid the stdout-buffering blind spot, see TESTING_LOG.md
```

**Verified (read-only only, no motion this session):** network reachable (ping,
dashboard, raw TCP to 10000), `arm_server.py` starts and binds `127.0.0.1:5000`, a direct
`ArmManager` client (bypassing `ArmInterfaceClient`'s `rospy.wait_for_message` watchdog
gate, mirroring `scripts/real_gen3_*.py`) reads `get_state()` cleanly — 7 finite joint
angles, unit-norm EE quaternion, gripper at the documented "0.009 = open" reading, no
fault. Clean `SIGINT` shutdown releases `/tmp/kinova.lock` both times tested.

**Not yet done here:** any motion rungs (this box has only done Jetson's rung-1/2
equivalent), the real `bulldog`/watchdog (rospy is finally available, so the bypass may
no longer be necessary — untested), and the full perception stack (`torch`,
`groundingdino`, `supervision`, and the lab's `netft_rdt_driver` F/T sensor ROS package
are all still missing from `ros_env` — same gap as the Jetson's hand-installed
perception deps, `netft_rdt_driver` has no public distribution at all).

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
| **handle in arm-base frame** | faithful mimic of `detect_handle_and_placement` | box → point cloud → `segment_plane` → protruding cluster (DBSCAN) → centroid → **camera→arm_base via the hand-eye calib + live EE pose** (replaces `tf2`). No rospy. |
| gripper | `open_gripper()`/`close_gripper()` over RPC | works: 0.009 open ↔ 1.0 closed |

> **YOLO doesn't work for the microwave here:** nano COCO models (`yolov8n`→"bus", `yolo11n-seg`→missed, `yolo26n-seg`→"train") mis-ID or miss it. `-seg` models need a warmup pass. GroundingDINO is the one that gets it. Seg is **not needed** anyway (appliance path is boxes only).

> **Camera→arm calibration — use the saved easy_handeye2 one:** `~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib` (eye_in_hand, `end_effector_link`→`camera_color_optical_frame`, ~180° about Z), chained with the live `get_state` EE pose. **Do NOT use the lab `sensors.launch` extrinsic** — it's for a different mount and drives the arm the *wrong way*.

> **⚠️ Depth bias (~16 cm):** perception **overestimates depth by ~16 cm** (lateral accurate ~5 mm, height ~4 cm). Teleop ground-truth: real handle `[0.713,-0.099,0.465]` (0.86 m, **reachable**) vs perceived `[0.874,-0.104,0.428]` (0.98 m). This one bias caused grasp overshoot (pushed the microwave) *and* false "out of reach" aborts. **Correct depth before trusting close grasps.** Reach note: handle sits near the Gen3 ~0.9 m limit — keep the microwave ~0.6 m from the base.

---

### Frame convention on this rig (`arm_base_link`)

| axis | direction |
|---|---|
| **+x** | **out / forward**, away from the robot toward the appliance |
| **+y** | left |
| **+z** | **up** |

So a larger x means *further from the arm*, and any approach standoff must be at
**smaller x than the target** — in front of the appliance, on the arm's side.

> **Watch the grasp-offset sign.** `GRASP_QUAT = (-0.5, 0.5, 0.5, -0.5)` (fixed in
> `detect_handle_and_placement`) maps **local +z to base −x**. So the repo's
> `pre_grasp = handle_transform @ trans(0, 0, -0.12)` moves **+0.12 in base x —
> away from the arm, behind the microwave**, which is both unreachable
> (0.98 m vs the Gen3's ~0.90 m) and physically impossible. On this rig the
> standoff offsets must be applied along **+z local** so they land at smaller x.
> Caught on 2026-07-29 by the approach script's reach gate.

### Perception — ROS 2 (Humble), the repo's own code path (2026-07-28)

The rospy-free pipeline above proved the geometry on hardware, but it ran
*alongside* the repo rather than through it. This box does have ROS 2 Humble, so
the repo's perception path is now **ported to ROS 2** instead of bypassed —
`OpenDoorHLA.open_microwave()` and `PressMicrowaveButtonHLA` run unmodified.

Only the client library and two message spellings differ, so the port adapts at
the boundary and leaves every detector's math untouched:

| ROS 1 (lab) | ROS 2 (here) |
|---|---|
| `CameraInfo.K/D/R/P` | `CameraInfo.k/d/r/p` |
| `stamp.secs/.nsecs` | `stamp.sec/.nanosec` |
| implicit global node (`rospy.init_node`) | explicit `Node` + an executor that must be spun |

`feeding_deployment/ros2/` holds the ROS 2 side:

| Module | Role |
|---|---|
| `node.py` | one process-wide rclpy node, spun on a daemon thread (stands in for rospy's implicit global) |
| `compat.py` | `CameraInfoCompat` — exposes ROS 1 field names over a live ROS 2 message, so `camera_info.K[0]` keeps working downstream |
| `realsense_ros2_interface.py` | `realsense2_camera` consumer with the **same `get_camera_data()` contract** as the ROS 1 class |
| `joint_state_bridge.py` | publishes `/joint_states` from the repo's own arm RPC (read-only) — there is no ROS arm driver on this rig |
| `calibration_tf.py` | latches the saved easy_handeye2 calib as static TF, plus an identity `base_link → arm_base_link` alias |

**Why the joint-state bridge:** tf2 can only answer
`arm_base_link → camera_color_optical_frame` if something publishes the arm's
joints. The lab gets that from its Kinova ROS driver; here the arm is behind
`arm_server.py`'s RPC, so the bridge republishes `get_state()` as `JointState`
and `robot_state_publisher` builds the rest of the tree:

```
base_link --(robot_state_publisher + joint_state_bridge)--> end_effector_link
end_effector_link --(easy_handeye2 calib, static)--------> camera_color_optical_frame
base_link --(identity, static)---------------------------> arm_base_link
```

`kortex_description`'s `gen3.xacro dof:=7 gripper:=robotiq_2f_85` emits exactly
`base_link` / `end_effector_link` — the frame names the saved calibration refers
to, so no renaming is needed.

**Bring-up** (arm servers first, as always — see the ladder above):

```bash
ros2 launch launch/ros2/microwave_bringup.launch.py
# verify BEFORE moving the arm:
ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame
```

> `ros2 launch` must run under **system python3** (it needs `lark`, which the
> project venv lacks). The launch file starts the repo's own modules with the
> venv interpreter, so both sides get what they need — nothing to install.

> `align_depth.enable:=true` is not optional: the handle pipeline reads depth at
> colour pixel coordinates.

**Button detection without Molmo.** `detect_start_button` reached the button only
through a remote Molmo VLM behind an ngrok tunnel on the lab network. There is
now a local GroundingDINO backend that applies the rule Molmo's own prompt
states — the *right* one of the two rectangular buttons on the *bottom* row of
the control panel. Selected with `BUTTON_BACKEND`:

| value | behaviour |
|---|---|
| `auto` (default) | try Molmo, fall back to GroundingDINO — identical to the old code whenever Molmo is reachable |
| `grounding_dino` | local only, no network |
| `molmo` | the lab's original behaviour |

> **Orientation is the trap:** the wrist camera is mounted upside down, so
> "bottom row" and "rightmost" are evaluated in the *flipped* view while
> `detect_items` returns raw original-image coordinates. Getting it backwards
> selects the diagonally opposite button — which looks perfectly plausible in a
> log. Covered by `tests/test_button_detection.py`.

**Offline validation (no arm, no camera):**

```bash
python scripts/generate_microwave_poses_offline.py --log_dir <dir>   # real geometry
python scripts/validate_open_microwave_hla_sim.py --log_dir <dir>    # real HLA
```

The first runs the genuine `perceive_handle_opening_poses("microwave")` with the
detector stubbed. The second replays it through the real behaviour tree — it
gets through staging, perception and the RViz guard, then stops at the first
Cartesian move because `plan_to_ee_pose` is broken on this box (see below), not
because of anything in the skill.

---

## ❌ Broken / blocked

| What | Symptom | Root cause | Workaround |
|---|---|---|---|
| IKFast (repo sim IK) | hangs indefinitely | IKFast build fails on this custom Gen3 URDF (aarch64) | PyBullet native `calculateInverseKinematics` |
| `plan_to_ee_pose` (sim cartesian) | never converges | needs a stepping/real-time loop + IKFast | native IK + `resetJointState` for kinematic playback |
| `rospy` (bulldog, PerceptionInterface, tf2) | not installed | **ROS 1 Noetic repo vs ROS 2 Humble / Ubuntu 22.04 box** — Noetic only targets 20.04 | **the perception path is now ported to ROS 2** (see below); bulldog still → **bypass** |
| bulldog | won't start | needs arm **and** base RPC servers up | stub base server + (real bulldog still needs rospy) |
| ViT-H on GPU | CUDA OOM (`NvMap error 12`) | 2.5 GB model + double-copy on 8 GB shared RAM | lazy SAM; use lighter SAM / bigger Jetson for the food path |
| GroundingDINO on GPU | two modes, see below | **NVML assert AND genuine OOM — they trade off** | run detection on **CPU** (`CUDA_VISIBLE_DEVICES=""`). (YOLO via `yolo_env`'s Jetson torch runs GPU fine.) |
| real-arm motion (all `set_*`) | `AssertionError: Bulldog is not running` | every motion method calls `_require_bulldog()` | bypass unlocks it |
| door arc rung 5 | swept inward toward base → e-stopped | arc was generated sweeping toward the base | **sweep forward/away from base**; verify reachability first |

---

> **GroundingDINO on GPU — re-tested 2026-07-29 with torch 2.11, still not viable.**
> CUDA itself is fine now: `torch.cuda.init()` succeeds and a real GPU matmul runs
> (the old blanket "Tegra can't do CUDA" reading was wrong). But Swin-B inference
> hits one of two walls depending on the allocator, and they trade off:
>
> | config | model load | inference |
> |---|---|---|
> | default | NvMap OOM (`error 12`) | — |
> | `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` | OK, 25.8 s, 1601 MB peak | `NVML_SUCCESS == r` assert, `CUDACachingAllocator.cpp:1165` |
> | + `PYTORCH_NO_CUDA_MEMORY_CACHING=1` | OK, 106.9 s | CUDA OOM |
>
> With caching you need the NVML interface the iGPU does not expose; without it you
> run out of memory for real. ~4.3 GB was free, so it is the Tegra GPU carveout, not
> total RAM. Not fixable by configuration.
>
> **Two untried levers, if CPU's ~85 s/frame becomes the bottleneck:** downscale the
> frame before detection (1280x720 -> 640x360 cuts activation memory ~4x and would
> speed up CPU too, but coarsens the handle centroid that feeds the grasp, so it
> needs validating against a touched ground truth); or a Swin-T checkpoint instead of
> Swin-B (smaller, but it is not on disk — a download).

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
