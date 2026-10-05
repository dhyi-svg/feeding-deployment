# Microwave container placement

Puts a container the gripper is already holding into the **open** microwave. The arm has NOT been
moved by this code yet. Older open items: `../NOTES.md`, 2026-10-02 section.

## Status (2026-10-04, first lab session)

Confirmed on the rig (rchi-cpu-5, gripper empty and closed, door open 90 deg):
- Node starts, SAM 3 loads; live overlay on `/microwave_place/interior_overlay` segments the open
  microwave's interior with the prompt "inside of open microwave" (user: box accurate to ~2 cm).
- Depth -> 3D -> `arm_base_link` -> cavity bounds + placement point runs live; `/microwave_place/placement_point`
  published (e.g. x 0.958, y 0.234, z 0.035).
- `plan_place`: two looks agreed within 3 cm; pre-insert/above/place poses passed every pose check
  (level, aimed in, fits, reach, under the top); IK of the first planned step solved (0.01 cm).

Not confirmed / blocking:
- **Door collision model is stale**: `plan_place` refused at step 1 of "to pre-insert" (gripper 3.5 cm
  inside the modelled door). The microwave moved ~16 cm left (+y) since the 10-03 door file, and the
  door slab is placed from `closed_grasp_pos`, which disagrees with the 10-03 refit hinge by ~13 cm.
  Measured: the real 90-deg door's inner face is ~8 in (0.20 m) left of the gripper's left finger
  (y ~0.43); the model has it at y ~0.07. Fix next: refresh the door file (re-detect handle/hinge),
  or a placement-only door-model shift (planned, not written).
- Rest of the IK/collision planning (all three legs), and **any arm motion inward** (execute_place),
  lowering, release, park.
- Floor z 0.035 looks low (door bottom was z ~0.10 on 10-03): touch the floor and compare before
  lowering; maybe a depth correction (`PLACE_DEPTH_CORR`).
- `~/.microwave_door.json` `door_open_deg` was hand-set to 90 (backup `.bak_2026-10-04`).

| File | What |
|---|---|
| `microwave_place_node.py` | ROS 2 node: live SAM 3 overlay + plan/execute services. Start here to build on it. |
| `real_gen3_ros2_place_container_microwave.py` | The steps as functions (`plan_placement`, `execute_placement`, `plan_release`, `execute_release`, `look_inside`) + a CLI. |
| `microwave_cavity.py` | Pure numpy: interior points -> cavity bounds + placement point. Tests: `tests/test_microwave_cavity.py`. |

Pipeline: wrist camera colour image -> **SAM 3** (off the shelf, prompt "inside of open microwave")
-> interior mask -> aligned depth -> 3D points -> tf to `arm_base_link` -> cavity box -> placement
point (two looks must agree within 3 cm) -> pre-insert / above / place tool poses (hand orientation
held fixed) -> PyBullet plan with IK / joint-limit / door-clearance checks -> joint commands over the
arm RPC. The final lowering is position-controlled; impedance control is a TODO (`lower_container`).

## Run (from the repo root, bring-up already up)

```bash
python3 -u microwave/placement/microwave_place_node.py --ros-args -p container_drop:=0.05
ros2 run rqt_image_view rqt_image_view /microwave_place/interior_overlay     # watch SAM 3 live

ros2 service call /microwave_place/plan_place std_srvs/srv/Trigger           # look + plan, no motion
ros2 param set /microwave_place allow_execute true                           # only when ready to move
ros2 service call /microwave_place/execute_place std_srvs/srv/Trigger
ros2 service call /microwave_place/plan_release std_srvs/srv/Trigger
ros2 service call /microwave_place/execute_release std_srvs/srv/Trigger
```

Topics, services and parameters are listed in the node's docstring.
