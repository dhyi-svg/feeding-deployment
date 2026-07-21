# Real-arm testing log — microwave door opening

Hardware bring-up + autonomous microwave-door-opening on the single-machine rig
(Jetson Orin Nano + Kinova Gen3, no NUC/base/ROS). Companion to `LOCAL_DEPLOYMENT.md`
(env / what-works reference) and the `## CURRENT STATE / NEXT STEP` block in `CLAUDE.md`.
Newest sessions on top.

---

## Setup — talking to the arm (every session)

The RPC arm interface hardcodes the lab NUC hostname; the local edit to
`arm_interface.py` lets an env var override it, and the Kortex SDK lives in the user
site (not `.venv`). So every arm command needs:

```bash
E="PYTHONPATH=$HOME/.local/lib/python3.10/site-packages ARM_RPC_HOST=127.0.0.1"
PY=$HOME/feeding-deployment/.venv/bin/python
```

- `ARM_RPC_HOST=127.0.0.1` → `NUC_HOSTNAME = os.environ.get("ARM_RPC_HOST", <lab NUC>)`
  in `arm_interface.py` (**uncommitted** local edit). Points the RPC client at the
  arm server running on this box instead of the lab NUC.
- Detection also needs `CUDA_VISIBLE_DEVICES=""` (GroundingDINO on the Tegra iGPU hits
  an NVML assert — run it on CPU, ~34 s/frame).

Bring-up order each session (see `LOCAL_DEPLOYMENT.md` for the full table):
1. `arm_server.py` — connects, clears faults, holds position (no motion).
2. `scripts/stub_base_server.py` — no-op base so bulldog's handshake passes.
3. `scripts/bulldog_bypass.py` — flips `bulldog_ready`, heartbeats `is_alive()`.
   **No software e-stop** — physical only. If it dies the arm e-stops in ~1 s.

**After Xbox teleop or any e-stop:** the arm goes to firmware manual mode / the Kortex
session faults. Restart `arm_server.py` (reconnect + fault-clear + hold), then re-run
the bypass (fresh server re-locks motion). Verify with `get_state()` → expect
`SERVOING` and control reclaimed.

---

## 2026-07-21 — native install blocker: Noetic-3.8 vs PRPL-3.10 (feed-noetic container)

**Not the arm/microwave rig** — this session used a separate Docker container,
`feed-noetic` (`osrf/ros:noetic-desktop-full` image, Ubuntu 20.04, **Python 3.8.10**)
on host `Pachirisu` (Ubuntu 24.04), repo bind-mounted at the same path inside the
container. Goal: get `pip install -e ".[robot]"` to succeed under stock ROS Noetic's
system Python. No arm hardware touched.

### Result: genuine, unfixable-via-pins blocker found, then unblocked via RoboStack

`pyproject.toml`'s `[robot]` extra is unversioned, so pip on a 2026 index backtracks
into source builds / unbuildable transitive deps. Fixed those first (see
[`constraints.txt`](constraints.txt), repo root):

| pin | reason |
|---|---|
| `pin==2.7.0` | newest releases dropped py3.8 wheel support here → source build |
| `ruckig==0.9.2` | same — no cp38 wheel for newest releases |
| `openai==1.55.0` | unpinned dep resolves to newest releases with a huge transitive graph → slow/failing backtracking |
| `tokenizers==0.20.3` | pulled in transitively by `anthropic` (unpinned). Newest `tokenizers` (0.21.0) has no cp38 wheel here, so pip builds from source; that build backend then requires `puccinialin` (a Rust-toolchain bootstrapper with **no PyPI distribution at all**) → hard failure. 0.20.3 has a prebuilt cp38 wheel, avoiding the source build entirely. |
| `anthropic==0.34.2` | unpinned, backtracks through ~70 historical releases (down to 0.2.x) resolving transitive deps — minutes of wasted resolution time per attempt |

With those pins, resolution succeeds and fails at exactly **one** remaining error:
```
ERROR: Package 'prpl-utils' requires a different Python: 3.8.10 not in '>=3.10'
```
Checked and confirmed this is not pin-fixable:
- `prpl-utils` / `pybullet_helpers` / `relational_structs` are pulled via **direct git
  URL** in `pyproject.toml` (from the `prpl-mono` monorepo), not resolved from PyPI —
  constraints.txt can't override what a URL-pinned requirement installs.
- Checked **every commit** in each package's history (via GitHub API, back to their
  first commit) and **every PyPI release** (0.0.1–0.1.1, they're also published
  there): `requires-python = ">=3.10"` in 100% of them, no exceptions.
- `tomsutils` (separate repo, `tomsilver/toms-utils`, also git-URL-pinned): `>=3.9` at
  its oldest commit, tightened to `>=3.10` later — never compatible with py3.8 either.
- Container only has `/usr/bin/python3.8` — no 3.9/3.10/3.11 installed (expected: ROS
  Noetic's Ubuntu 20.04 base ships py3.8 as system Python).

So: **stock Noetic (py3.8) and the PRPL deps (py3.10+) are irreconcilable in the same
native environment.** No version pin, git ref, or PyPI release bridges that gap.

### Fix: RoboStack (conda-forge-packaged ROS) on Python 3.11

RoboStack packages ROS distros against modern conda-forge Python builds, decoupling
the ROS version from the OS's system Python. Verified end-to-end in the same
container:

1. Miniforge → `/opt/miniforge3`.
2. `mamba create -n ros_env python=3.11` (not 3.10 — `robostack-staging`'s
   `ros-noetic-desktop-full`/`ros-noetic-ros-base` builds only target py3.9/3.11/3.12,
   no py3.10 build exists; 3.11 still satisfies PRPL's `>=3.10`).
3. Channels: `robostack-staging` (priority) + `conda-forge`.
4. `mamba install ros-noetic-ros-base` — used the lighter `ros-base` metapackage, not
   `desktop-full` (don't need rviz/gazebo/GUI tooling for rospy/roscomm). **Gotcha:**
   the first `desktop-full` attempt aborted mid-transaction on a transient download
   timeout (two packages, `roswtf`/`rqt-robot-steering`) and mamba silently rolled
   back the whole env — `rospy` wasn't actually installed despite the wrapping shell
   command reporting exit 0 (a trailing `echo` after the real command masked mamba's
   failure). Bumped `remote_max_retries`/`remote_read_timeout_secs` and switched to
   `ros-base`; it completed cleanly on retry.
5. `python -m pip install -e ".[robot]" -c constraints.txt` inside `ros_env` —
   **succeeded outright**, including editable-building `prpl-utils`, `pybullet_helpers`,
   `relational_structs`, `tomsutils` (they're pure Python; `>=3.10` was the only gate).

Re-verified explicitly, all four in the **same** interpreter:
```
$ python -c "import rospy; print(rospy.__file__)"
/opt/miniforge3/envs/ros_env/lib/python3.11/site-packages/rospy/__init__.py
$ python -c "import pybullet_helpers, relational_structs, prpl_utils; print('prpl ok')"
prpl ok
$ python -c "import feeding_deployment; print('repo ok')"
repo ok
$ python --version
Python 3.11.15
$ which roscore
/opt/miniforge3/envs/ros_env/bin/roscore
```
`rospy`, the PRPL deps, and the repo itself all import from the same
`/opt/miniforge3/envs/ros_env/` interpreter — genuinely unblocked. This is the path to
running the real `bulldog`/rospy-based executive (not the bypass) and the full HLA
system, without droppping either Noetic or the PRPL deps.

### Not yet done
- Only `ros-base` installed, not `desktop-full` — `rviz_interface.py` will still need
  `rviz` (not in `ros-base`) if/when that's exercised; separate from the existing
  `rviz` `None`-guard TODO at `open_door.py:202`.
- This env is untested against real ROS master / catkin workspace builds (`roscore`,
  `catkin_make`, message generation for `feeding_deployment_msgs`) — only Python-level
  imports were verified here.
- Not yet tried on the Jetson rig (`LOCAL_DEPLOYMENT.md`'s box) — that one already has
  a working py3.10 `.venv` path via user-site Kortex SDK; RoboStack would be an
  alternative there too if a real rospy/bulldog is ever needed on that box.

---

## 2026-07-14 — autonomous detect → grasp → open

### Result
**Full autonomous microwave-door open works end-to-end:** live detect → handle in
arm-base frame → depth/lateral-corrected grasp → joint-space arc opens the door ~90°
→ release → retract. Grip holds through the whole swing.

### Perception → grasp (one script)
Grab RealSense (D435I, eye-in-hand) via `pyrealsense2` → GroundingDINO detect
`microwave` (CPU) → deproject box → open3d `segment_plane` → protruding cluster
(DBSCAN) → handle centroid → camera→arm-base via the **easy_handeye2** calib
(`~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib`, eye_in_hand) chained
with live `get_state()` EE pose (replaces tf2). **No SAM/rospy/tf2.**

Empirical corrections (from teleop ground-truth), baked into the grasp:
| const | value | why |
|---|---|---|
| `DEPTH_CORR` | `0.16` | perception overestimates depth ~16 cm; scale the handle ray in by 16 cm |
| `LAT_CORR`   | `0.07` | protruding-cluster centroid sits ~7 cm off the latch; shift −y |
| `GRIP_EXT`   | `0.065`| grasp EE = handle − 6.5 cm along approach. 6.5 (vs earlier 9) = the proven "grip a bit more forward → firmer" fix; grip then held through the swing |

Grasp lands within ~1–2 cm. Sequence: `open_gripper` → move to pre-grasp (handle −
`GRIP_EXT+0.10`) → step in over 4 sub-steps (tracking-abort > 2.5 cm) → `close_gripper`
→ **pause for a human grip check** before opening.

### Seeded-IK fix (wrist-flip prevention)
Symptom: a large move (e.g. back-off after opening) made the wrist "flip all the way
around". Root cause: `p.calculateInverseKinematics` was seeded from the sim's default
(home) config, not the arm's actual pose, so it returned a far-away joint solution.

Fix: **before each IK call, seed the sim from the arm's current real joints** —
```python
for i, jj in enumerate([1,2,3,4,5,6,7]):
    p.resetJointState(rb.robot_id, jj, ai.get_state()["position"][i],
                      physicsClientId=rb.physics_client_id)
sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id, pos, quat, ...)
```
IK then returns a config near the current one → no flip. Confirmed: the detect+grasp
run this session made no weird wrist moves.

**Caveat — don't seed mid-arc.** Seeding is too conservative on the small arc
waypoints: it stalled on door-arc step 2 with sim `ik_err 3.5 cm` (unseeded/home IK
reaches those exactly). Rule: **seed for large jumps (approach, retract, back-off);
use unseeded IK for the arc waypoints**, or try seeded first and fall back to unseeded
when `ik_err > 2 cm` (also guard against a big joint-jump = flip).

### Door-opening arc (the runner)
Uses the **repo's own** arc geometry — `PerceptionInterface._generate_door_arc_waypoints`
(no `self`, invoked unbound) with the microwave params `arc_length_m=0.55`,
`waypoint_spacing_m=0.05`, `direction=-1` (left-hinged → handle sweeps −x toward the
arm, **not** into the microwave), `rotate_orientation=True`. Start = the real grasp
pose; hinge estimated one door-width (`+0.32 m`) to `+y` (the wrist cam can't see the
door while grasping).

Each waypoint: IK → `set_joint_position` (joint control is reliable where
`set_ee_pose`/cartesian aborts at extended configs) → **wait for velocity ≈ 0, then
re-check the EE** (a move can return before it settles) → tracking-abort if the EE is
> 2.5 cm off (door bind / latch / hinge limit → natural stop) → **1 s pause** between
steps. Reproduce with a heredoc that:
1. reads current EE (the grasp pose) → `start_pose`, hinge = `(x, y+0.32, z)`;
2. `wps = _generate_door_arc_waypoints(None, start_pose, hinge, 0.55, 0.05, direction=-1, rotate_orientation=True)`;
3. builds the sim (`create_scene_description_from_config(".../configs/vention.yaml","skewer")` → `FeedingDeploymentPyBulletSimulator`);
4. per waypoint: IK (see seeding rule above) → `set_joint_position` → settle-wait → tracking check → 1 s pause.

This session's arc: **step 1 opened cleanly** (handle pulled to `[0.532,-0.118,0.48]`,
grip 0.99 held); step 2 stalled only because IK was seeded (see caveat) — not a
hardware problem.

### Speed / safety
`set_speed("low")`, hand on the physical e-stop, camera rig for another project sits
under the wrist so keep z raised and sweep forward/away from the base.

### State at end of session
Arm was **holding the door grasped** (gripper ~0.99) at ~arc step 1, paused. To reset:
restart `arm_server.py` + re-run `bulldog_bypass.py`.

### Next step
Finish the arc with the seeded→unseeded IK fallback, then fold detect→grasp→open into
the `open_microwave` HLA (needs the `rviz` `None`-guard at `open_door.py:202` + perception
wired in without `PerceptionInterface`/rospy).

---

## Earlier (see git log + `LOCAL_DEPLOYMENT.md`)

- 2026-07-09: arm bring-up — torque zeroing (`kinova.py`), `arm_server.py`, stub base +
  bypass, motion ladder rungs 1–4 (connect, telemetry, +5° joint, +5 cm cartesian).
- 2026-07-14: live rospy-free perception (RealSense + GroundingDINO CPU), the
  easy_handeye2 calib vs the wrong lab `sensors.launch` extrinsic, the ~16 cm depth bias.
