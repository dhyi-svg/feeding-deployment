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
| 4 | cartesian move (+5 cm z) | 2026-07-09 | `set_ee_pose(pos, quat)`, dz=+5 cm no drift |
| 5 | forward door-opening arc | ⚠️ partial | first 3 waypoints tracked 0–1 cm; **aborted** — see below |

---

## ❌ Broken / blocked

| What | Symptom | Root cause | Workaround |
|---|---|---|---|
| IKFast (repo sim IK) | hangs indefinitely | IKFast build fails on this custom Gen3 URDF (aarch64) | PyBullet native `calculateInverseKinematics` |
| `plan_to_ee_pose` (sim cartesian) | never converges | needs a stepping/real-time loop + IKFast | native IK + `resetJointState` for kinematic playback |
| `rospy` (bulldog, PerceptionInterface) | not installed | ROS2 Humble on box vs ROS1 Noetic repo | bulldog → **bypass**; perception → recorded images / future x86 box |
| bulldog | won't start | needs arm **and** base RPC servers up | stub base server + (real bulldog still needs rospy) |
| ViT-H on GPU | CUDA OOM (`NvMap error 12`) | 2.5 GB model + double-copy on 8 GB shared RAM | lazy SAM; use lighter SAM / bigger Jetson for the food path |
| real-arm motion (all `set_*`) | `AssertionError: Bulldog is not running` | every motion method calls `_require_bulldog()` | bypass unlocks it |
| door arc rung 5 | swept inward toward base → e-stopped | arc was generated sweeping toward the base | **sweep forward/away from base**; verify reachability first |

---

## ⚠️ Gotchas & safety

- **Physical obstacle:** a camera rig for another project is mounted under the arm base. Real-arm trajectories must stay **in front of / clear of the base** — never sweep inward.
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

- Rung 5: a **forward-sweeping** door arc that completes (last attempt swept inward)
- Real handle/hinge perception → a genuine `handle_opening_pos.pkl` (needs camera + rospy)
- Full `open_microwave` HLA on hardware (needs perception + the rviz `None`-guard that is **not** yet fixed at `open_door.py:202`)
- rospy/roscore path (or migrate to an x86 ROS1 box) to run real bulldog + perception
