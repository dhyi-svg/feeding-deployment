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
