# Repo audit — hardcoding & robustness (everything except the microwave path)

Audit date: 2026-07-30. Branch `microwave-task`. Scope: the whole repo **excluding**
the microwave door-opening / button-pressing work (that is tracked in `CLAUDE.md`
and `TESTING_LOG.md`).

Verified by running the test suite (`165 passed, 1 failed`) and probing imports in
`.venv` on this Jetson.

---

## Category A — Fully hardcoded, dead on this rig (rewrite or delete)

| Thing | What's hardcoded | Verdict |
|---|---|---|
| `actions/flair/inference_class.py:21-23` | `/home/isacc/Grounded-Segment-Anything`, `/home/isacc/Depth-Anything`, `.../spaghetti_checkpoints` | Paths don't exist here; `spaghetti_checkpoints/` isn't in the repo at all, `depth_anything` not installed. FLAIR food-manipulation is **non-functional**. |
| `perception/head_perception/deca_perception.py` | `from DECA.decalib...` — the `DECA/` submodule was never vendored (only `config/*.npy` survives) | Whole head-perception stack is import-dead. Everything downstream of it (`feel_the_bite/*`, mouth-open gestures, `get_head_perception_data`) can't run. |
| `perception/head_perception/ros_wrapper.py:163-167` | `file:////home/isacc/deployment_ws/src/kortex_description/tools/*.stl` | RViz markers point at a nonexistent workspace. |
| `misc/visualize_log.py:5`, `misc/grab_*_from_rosbag.py` | Absolute paths to a specific user's `.bag` files from 2024-09-07 | One-off scratch scripts. Delete or convert to argparse. |
| `safety/bulldog.py:79-81` | `remote_execution_log_path` + `hostname="192.168.1.2"` + `username="isacc"` — SCPs logs to the lab compute box | Only relevant in the two-machine lab wiring. On a single box this is pure dead weight (and `bulldog` also hard-requires the base RPC server — which is why `scripts/bulldog_bypass.py` exists). |
| `integration/launch_robot.sh`, `run_bulldog*.sh`, `launch/arm.launch`, `arm_sensors.launch`, `sim.launch`, `cartographer_mapping.launch`, `sensors.launch`, `config/maps/vention_map.yaml` | `/home/isacc/...`, `/home/tom/...`, `/home/isacc/miniconda3/envs/feed/bin/python`, TLS certs at `/home/isacc/certs/192.168.1.2*.pem` | Every ROS 1 launch file is pinned to one machine's filesystem. |
| `control/robot_controller/checking.py`, `motioneye_cam.py` | `http://192.168.1.5:8083/` MotionEye camera | Lab-only device. |
| `webapp/src/config/parameterConfig.js` | `ROS_HOST='192.168.1.2'`, and `USER = 'Hi Aimee'` — a **participant's name baked into the frontend** | Needs to come from a runtime config/query param. |

---

## Category B — "Seems pretty hardcoded" but is actually the intended design

Keep these. They look like magic numbers but are deliberately externalized
calibration. Don't rewrite — **re-measure**.

- **`simulation/configs/vention.yaml` / `wheelchair.yaml`** (288 / 94 lines, ~62
  entries). Every named joint config and EE pose (`retract_pos`,
  `utensil_inside_mount_pose`, `fridge_door_gaze_pos`, `plate_staging_pos`, …)
  lives here and is loaded by `create_scene_description_from_config()`. This is
  the good pattern. The values are for the lab's Vention stand — worthless on
  your mount, but the *mechanism* is fine. You already started the replacement:
  `config/local_arm_presets.yaml` has 2 of ~62.
- **`simulation/scene_description.py:158-166`** — `tool_frame_to_finger_tip`
  (5.955 cm) and `camera_pose` (EE→`camera_color_optical_frame`) are dataclass
  defaults, not YAML. These *are* rig-specific and *should* be moved into the
  YAML. Same for `tool_frame_to_utensil_tip` / `_drink_tip` / `_wipe_tip`
  (lines 283-310) — those are tool-geometry, valid if you have the lab's tools,
  wrong otherwise.
- **`config/nav_named_locations.yaml`** + `_load_target_pose()` with
  `FEEDING_NAV_LOCATIONS_FILE` override, captured by
  `scripts/capture_named_locations.py`. Properly externalized.
- **`config/nav/*.yaml`**, `navigate.py`'s `_REFINEMENT_DEFAULTS` (read from
  `custom_param.yaml` at runtime), `_scripted_speeds()` re-reading
  `teb_local_planner.yaml` each leg. Well-engineered.
- **`safety/watchdog.py:36-49`** frequency thresholds — each has a documented
  "expected is X Hz" rationale. Good. But `EXPECTED_CAMERA_RESOLUTION =
  (1280, 720)` will trip on a 640×480 stream.

---

## Category C — Genuinely not robust (real hardcoding inside logic)

Magic numbers buried in algorithms, with no config path.

1. **`perception/appliance_perception/appliance_perception.py:905`** — sink
   placement point is `((x1+x2)//2 + 140, y1 - 50)`. A **+140 px** offset in
   image space, unscaled by resolution or distance. Explicitly labeled
   `# Hack`. Breaks at any resolution ≠ the lab cam. Same file,
   `detect_table_placement` follows the same pattern.
2. **`appliance_perception.py:741-744`** — handle centroid = `top_most_y - 0.07`
   (fridge) / `- 0.04` (else). Two magic offsets chosen per appliance string.
3. **`appliance_perception.py:70-75`** — `_SWING_SWEEP_DEG = 43.0` and a
   `_SWING_KY` dict keyed on the literal detector prompt string
   (`"bottom textured fridge door"`). Presentation-only (overlay arrows), so low
   risk, but the string-keyed dispatch pattern recurs in the real geometry above.
4. **`interfaces/perception_interface.py:456-471`** —
   `get_tool_tip_pose_at_staging()` returns three literal 4×4 poses per tool.
   Carries its own `# Rajat ToDo: Fix these hardcoded values`.
5. **`perception_interface.py:1441-1490` and `1645-1690`** — drink and plate
   pickup are **entirely** hardcoded transform chains off an ArUco marker:
   `[0.09, -0.02, 0.1]`, then `tf[2,3]=0.017`, `tf[0,3]=0.14`, `tf[0,3]=0.32`.
   Plus `get_aruco_relative_pose(..., override_angles="drink"|"plate")` which
   throws away the detected roll/pitch and substitutes `π/2` / `π`. This is
   teach-by-numbers with a marker for the origin — it works, but only for that
   exact bottle, plate, and marker mounting.
6. **`perception_interface.py:1693`** — `time.sleep(3)` labeled
   `# Rajat Hack: Wait one second for the aruco mean to be correct, does this
   actually help though?`. Comment says 1s, code says 3.
7. **`actions/feel_the_bite/inside_mouth_transfer.py:15-26`** — 11 module-level
   constants controlling motion *inside a user's mouth*
   (`INSIDE_DISTANCE_LOOKAHEAD_Z = 0.04`, `MOVE_OUTSIDE_DISTANCE = 0.12`, …),
   none user-configurable, several with commented-out alternates.
   Safety-relevant, and tuned to the lab's fork geometry.
8. **`control/robot_controller/kinova.py:105`** — `self.fix_joint_hack = True`,
   unconditionally freezing joint 6 to make the 7-DOF arm behave as 6-DOF for
   compliant control. Threaded through `compliant_controller.py` in five places.
   Not a flag, not a config.
9. **`kinova.py:54, 116`** — arm IP `192.168.1.10` in two places (one a literal
   inside a `ping`), credentials `("admin","admin")`. Contrast
   `arm_interface.py:20`, which *does* honor `ARM_RPC_HOST`.
   `base_interface.py:22` has no such override.
10. **`integration/run.py:415-455`** — ~40 lines of
    `if hla_name == "..." and obj_combo[0].name != "..."` filtering,
    self-labeled `# Major hack. The proper way to do this would be to define
    subtypes`. Plus `# Super hack: the drink and wipe are always prepared`
    (line 1137) unconditionally injecting `ToolPrepared` atoms into the planner
    state each step, and a `# Super Hack: skip the plate transfer behavior
    tree`. The PDDL type lattice exists — these bypass it.
11. **`interfaces/perception_interface.py:18`** — LED serial port is a full
    `/dev/serial/by-id/usb-UnexpectedMaker_FeatherS2_Neo_84722E753121-if00`,
    i.e. keyed to one physical device's serial number. Same pattern in
    `base_controller/vention_arduino_control.py:44`.

---

## Category D — What actually works right now

- **Preference learning** (`preference_learning/`,
  `integration/preference_session.py`, `apply_preferences.py`,
  `checkpoint.py`) — pure Python + Anthropic API, no hardware. Cleanly
  structured: `config/` holds the meal catalog and bundle schema, `methods/`
  the memory models. ~140 of the 165 passing tests are here.
  **This is the healthiest part of the repo.**
- **Behavior-tree layer** — the `!hla` / `!scene_description` YAML loader,
  `is_user_editable` params, `apply_bundle_to_behavior_trees`, live LLM-driven
  edits. Data-driven and rig-agnostic.
- **`_generate_door_arc_waypoints`** (`perception_interface.py:516`) — clean,
  general, no magic numbers, works for any hinged door.
- **ROS 2 shim** (`ros2/`) — `CameraInfoCompat`, joint-state bridge,
  calibration TF. Written locally, no lab assumptions.
- **`navigate.py`** — despite being the largest file (1880 lines), it's the most
  carefully written: every constant has a documented empirical justification,
  config is re-read at runtime, fallbacks are explicit. Blocked only by the base
  being unpowered.
- **`utils/llm_config.py`** — well-documented model choice, effort level, and
  the empirical sweep that justified them.

### Dependency status (probed in `.venv`)

Available: `rclpy`, `torch`, `groundingdino`, `segment_anything`, `supervision`,
`open3d`, `pybullet`, `pybullet_helpers`, `relational_structs`, `tomsutils`,
`anthropic`, `openai`, `cv2`, `sklearn`, `paramiko`, `serial`.
`kortex_api` is in system python / `~/.local`, **not** `.venv`.

**Hard blockers:**

| Missing | Kills |
|---|---|
| `netft_rdt_driver` | F/T sensing — no public distribution at all. `UserTransferDoneSignal="sense"` can't work. |
| `dynamixel_sdk` + `wrist_driver_interfaces` | Wrist controller (utensil rotation) |
| `rospy` | All ROS 1 launch files and nodes |
| `DECA` | Head perception → both `feel_the_bite` transfers |
| `depth_anything` + missing checkpoints | FLAIR food manipulation |

---

## Category E — Quick wins vs. rewrites

### Directly implementable (< 1 hour each)

- **Finish `config/local_arm_presets.yaml`** — record the remaining ~60 poses via
  teleop. Everything in Category B / `vention.yaml` unlocks at once.
  **Single highest-leverage item.**
- `ARM_RPC_HOST`-style env override for `base_interface.py:22`,
  `kinova.py:54/116`, webapp `ROS_HOST`, `USER`.
- Move `camera_pose` + `tool_frame_to_finger_tip` from `scene_description.py`
  defaults into the scene YAML.
- Fix or delete the stale test: `tests/test_apply_preferences.py:220` expects
  `near → 0.07`, `apply_preferences.py:55` maps `near → 0.05`. One of the two is
  wrong; it's the only failing test in the suite.
- `launch/*.launch`: `$(find feeding_deployment)` / `$(env HOME)` instead of
  `/home/isacc`.
- Scale the sink/table `+140, -50` pixel offsets by `image_width / 1280` — a
  3-line change that at least makes them resolution-invariant.

### Quick (a session)

- Make `fix_joint_hack` a constructor arg defaulting to current behavior, so
  7-DOF compliant control becomes testable.
- Replace the `by-id` serial constants with a glob + first-match, or an env
  override.
- Delete `misc/visualize_log.py`, `grab_*_from_rosbag.py`, `checking.py`,
  `motioneye_cam.py` — pure lab scratch.

### Needs real rewrite

- **Drink and plate pickup** (C#5). The ArUco-relative transform chain should
  become the same detect → plane-fit → cluster pipeline
  `detect_handle_and_placement` already uses — that code works, it just isn't
  wired to these skills.
- **Sink/table placement detection** (C#1) — currently a bounding box plus two
  magic pixel offsets, no 3D reasoning beyond a single `pixel2World` call.
- **`run.py`'s planner grounding filter** (C#10) — should be PDDL subtypes.

### Can't be saved without new hardware/deps

Head perception + both `feel_the_bite` transfers (DECA), FLAIR food manipulation
(Depth-Anything + missing checkpoints), F/T-based bite detection
(`netft_rdt_driver`), wrist rotation (`dynamixel_sdk`).

---

## Two things worth flagging

1. A **participant's name is in the shipped frontend**
   (`webapp/src/config/parameterConfig.js:9`).
2. The **in-mouth transfer constants** (C#7) are safety-relevant, not
   configurable, and tuned to a fork you may not have.
