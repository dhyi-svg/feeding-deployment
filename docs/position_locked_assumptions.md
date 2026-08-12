# Position-locked assumptions in the microwave path

Everything here breaks, or silently degrades, when the **microwave moves or rotates** —
or in a few cases when the arm starts somewhere new. The goal is a system that finds and
grasps the handle wherever the appliance is; this file tracks how far the current code is
from that, and who introduced each gap.

**Origin** is against `upstream/main` (the lab's `empriselab/feeding-deployment`), the
fork point for `microwave-task`:

- **UPSTREAM** — already in the lab code. Not introduced here, and fixing it changes
  shared behaviour for the fridge path too.
- **BRANCH** — added by this fork, almost always to work around an upstream limitation
  on this rig.

Written 2026-08-11.

---

## The big one: RANSAC's plane equation is computed and thrown away

`appliance_perception.py` fits a plane to the appliance face and keeps only the inlier
indices — `plane_model` (the `ax+by+cz+d` coefficients) is never referenced again. That
single discarded variable is behind **three** separate position-locked behaviours below.
Upstream does exactly the same (`plane_model` assigned at upstream line 384, unused).

| Use it for | Fixes |
|---|---|
| signed distance to the plane, instead of a scalar median depth | tilt-dependent protrusion test |
| the plane **normal** | hardcoded approach direction |
| the plane's in-face axes | hinge-side assumption |

---

## Inventory

| # | Thing | Where | Origin | Assumption | Symptom when violated |
|---|---|---|---|---|---|
| 1 | Scalar-median protrusion test | `appliance_perception.py` (`plane_depth = np.median(...z)`, then `plane_depth-0.07 < z < plane_depth`) | **UPSTREAM** | appliance face is roughly fronto-parallel | measured **11.6 cm** depth span across a flat face on 2026-08-11; bodywork and the table both qualified as "protruding", handle cluster lost the vote, **20 cm** error |
| 2 | Fixed grasp quaternion `(-0.5, 0.5, 0.5, -0.5)` stamped on every pose | `appliance_perception.py` | **UPSTREAM** | one fixed appliance orientation | gripper approaches from a fixed world direction regardless of which way the door faces |
| 3 | `GRASP_QUAT_FIX = R.from_euler("y", π)` | `real_gen3_ros2_approach_microwave.py`, `..._grasp_...` | **BRANCH** | microwave is along base **+x** | rig-specific patch for #2; wrong for any other appliance bearing |
| 4 | `DOOR_W = 0.32`, `hinge = handle_y + DOOR_W` | `real_gen3_ros2_open_arc_microwave.py` | **BRANCH** | hinge is exactly 0.32 m in base **+y** | arc pivots about the wrong point if the appliance is rotated or a different width |
| 5 | `DIRECTION = -1` | same | **BRANCH** (mirrors upstream) | door swings toward the arm | wrong-direction swing; caused a real e-stop on the other rig 2026-07-30 |
| 6 | `direction = 1 if "bottom textured fridge door" else -1`; `arc_length_m = 0.55 if "microwave" else 0.35` | `perception_interface.py` `perceive_handle_opening_poses` | **UPSTREAM** | swing direction and arc length are properties of the *prompt string* | never detected; a mirrored or differently-sized appliance is simply wrong |
| 7 | Hinge = extreme edge of the plane cloud (`np.max`/`np.min` per appliance) | `appliance_perception.py` | **UPSTREAM** | hinge side follows from the appliance name | perceived hinge came out **wrong-side** at multiple viewpoints; all scripts route around it |
| 8 | `hinge_3d[1] = handle_centroid_3d[1]`, labelled `# Hack` | `appliance_perception.py` | **UPSTREAM** | hinge axis is vertical and at handle height | door that rises as it opens is unmodelled (+5.8 cm z error measured on the other rig) |
| 9 | `top_of_appliance[1] = np.max(plane_cloud[:,1])` | `appliance_perception.py` | **UPSTREAM** | — | **sign bug**: camera-frame +y is *down*, so this finds the BOTTOM. Reported z=0.32 for a handle at z=0.52. Unused by the scripts; on the HLA path `post_release_pose` would lift *into the table* |
| 10 | `handle_centroid[1] = top_most_y - 0.04` | `appliance_perception.py` | **UPSTREAM** | handle centre is 4 cm from the cluster edge | magic offset per appliance; interacts with #9's frame confusion |
| 11 | `PLAUSIBLE_X/Y/Z` box in base frame | `real_gen3_ros2_grasp_microwave.py` | **BRANCH** | handle lives in a fixed absolute box | would reject a legitimately relocated microwave; too loose to catch the 20 cm error anyway |
| 12 | `TRUTH` / `STALE_TRUTH` constants | approach / grasp scripts | **BRANCH** | one fixed handle position | reporting only, not gated — but prints meaningless "27.5 cm from truth" lines |
| 13 | `microwave_view_pos`, `microwave_approach_start_pos`, Kortex `MICROWAVE_HOME` | `config/local_arm_presets.yaml`, arm | **BRANCH** | one viewing pose | convenience only; nothing depends on it |
| 14 | Largest-cluster wins (DBSCAN) | `appliance_perception.py` | **UPSTREAM** | handle is the biggest protruding thing in the box | background clutter out-voted the handle on 2026-08-11 |
| 15 | Detection box ≈ whole frame | GroundingDINO + prompt | **UPSTREAM** | — | box covered 89% of the image, so all localisation falls to the 3D stage and to #1/#14 |

### Not position-locked (verified)

- **`HANDLE_DEPTH_CORR = 0.094`** (BRANCH). A *camera* bias, not a position: 5 mm drift
  over 13.7 cm of range, and re-verified at +0.1 cm against a touched pose on 2026-08-11
  after the microwave had moved. Keep it.
- **`HANDLE_LAT_CORR = 0.0`** (BRANCH). Deliberately zero — the observed lateral error is
  variance, not bias, and the sign has flipped between sessions.
- **`MAX_REACH` / `MIN_Z` / `MAX_Z` / `MAX_JOINT_JUMP`** — arm limits, independent of the
  appliance.
- **The detection server** (`detection_service.py`, BRANCH). Caches model weights only;
  every `detect()` takes a live camera frame and a live tf lookup, and refuses a frame
  older than 5 s.

---

## What this means for the split

Nine of the fifteen are **upstream**, and they are the deep ones — the protrusion test,
the fixed grasp quaternion, the hinge heuristics, largest-cluster selection. The lab code
was written for one appliance in one place, and generality was never a requirement.

The **branch** entries are almost all *workarounds for upstream entries*: #3 patches #2,
#4 and #5 replace the unusable output of #7 and #8. That is worth noting before "fixing"
a branch constant — several of them are load-bearing precisely because the upstream
behaviour underneath them is wrong.

## Suggested order

1. **#1 — use the plane equation.** Tilt-invariant by construction, and would have
   rejected the table cluster outright. Roughly five lines, upstream file.
2. **#14 — select clusters by shape, not size.** A thin vertical bar reads the same from
   any angle; "biggest blob" depends on what else is in frame.
3. **#2/#3 — derive the approach direction from the plane normal**, retiring the fixed
   quaternion and its 180° patch together.
4. **#15 — two-stage detection** (appliance → crop → handle) so the background stops
   competing at all.
5. **#4/#5/#7/#8 — real hinge geometry.** The hardest, and the one that most needs a
   corpus spanning several appliance orientations before it can be evaluated honestly.
6. **#9 — the `top_of_appliance` sign bug.** Small, isolated, and blocks the HLA path.

Validate every one of these against a labelled corpus spanning **multiple appliance
positions and orientations** (see `scripts/session/record_ground_truth.py` and
`scripts/session/replay_detection.py`) — not against a single setup, which is how these
assumptions survived this long.
