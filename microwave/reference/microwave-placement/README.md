# Reference only -- not active code

Files here are the `microwave-placement` branch's versions of files that conflicted with the sheppy line
(`microwave-wip`) in the 10-23 merge. The active versions are the ones in `microwave/`. Nothing imports these.

- `real_gen3_ros2_grasp_and_swing_microwave.py` (from `7b55fb33`): adds `--light-grip [RELAX]` and the
  `HINGE_DEPTH_RANGE` hinge-depth gate (commit `d15bdd51`), on the pre-sheppy open (door-file hinge carry-over,
  `ArmInterfaceClient`). See `microwave/NOTES.md`, 2026-10-23 merge entry and 2026-10-03.
