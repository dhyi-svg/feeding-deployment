# Navigation Stack — Verified Current State (2026-07-15 evening)

Every claim below was checked against the **files on disk at commit `9653969`
("zed vio teardown") + working tree** AND, where marked ✅live, against the
**running ROS master** on the evening of 2026-07-15 (post-relaunch with the
IMU-only ZED config). This supersedes the architecture/param sections of
`navigation_stack.md` for "what is true right now"; that file keeps the
history and failure-analysis depth.

---

## 1. Big picture

```
                       lidar_l/scan ─┬─► scan_gate ─► /lidar_{l,r}/scan_gated ─► cartographer_node ─► map→odom TF
                       lidar_r/scan ─┘   (coverage gate)                          (pure localization,      │ (+50 ms lookahead)
                                                                                   frozen pbstream)        ▼
 ZED Mini (IMU-only) ─► imu/data ─► gyro_bias_estimator ─► imu/data_debiased ─┐                        /map grid
                     └─► SVO2 recording (raw stereo+IMU, per bringup)          │ vyaw
                                                                               ▼
 base encoders (NUC RPC) ─► wheel_odom_publisher ─► /wheel_odom ─────► ekf_fused_imu_wheel ─► odom→vention_base_link TF
                                                     vx (+advisory vyaw)        │
                                                                                ▼
                                                                    /odometry/fused_imu_wheel (20 Hz)
                                                                    consumed by: cartographer (odom input),
                                                                    TEB (odom_topic), drift tracer, nav_diag_logger
 move_base (GlobalPlanner + TEB) ─► /cmd_vel ─► cmd_vel_bridge_basicmicro ─► set_speeds RPC ─► NUC base_server ─► Arduino v7
 Xbox teleop (deadman) ──────────► /cmd_vel_teleop ─┘ (priority, hold-exempt)
```

**TF ownership** (✅live, no fights):
| Transform | Owner |
|---|---|
| `map → odom` | cartographer_node (frozen-state localization; publishes **50 ms into the future**, §3) |
| `odom → vention_base_link` | `ekf_fused_imu_wheel` (20 Hz) |
| `vention_base_link → zed_mini_base_link` | ZED wrapper's own state publisher (via `general/base_frame`; deliberately NOT in combined.urdf — last-writer-wins fight, 2026-07-13) |
| `vention_base_link → lidar_{l,r}` | static publishers in sensors.launch (x 0.2575, y ±0.135, z 0.415, roll π) |
| `arm_end_effector_link → camera_link` | static (arm RealSense mount) |

**Hosts**: compute (.2) runs everything above; NUC (.3) runs `base_server.py`
(owns the Arduino; lost-command stop lives THERE, not in the bridge) +
`arm_server.py` + bulldog/e-stop; Mac (.13) sends the e-stop heartbeat.

---

## 2. Sensors bringup — `launch/sensors.launch`

| Node | Key params | Lifecycle |
|---|---|---|
| 2× rplidarNode | port-pinned by USB path, 115200, `angle_compensate` | plain |
| zed_node (zedm) | see below | **required=true** (a ZED death exits the whole launch — deliberate, no coded recovery) |
| zed_svo_recorder | output `integration/log/svo/`, one file per bringup | plain |
| wheel_odom_publisher | `counts_per_meter 4874`, `track_width 0.85`, 20 Hz poll | respawn 2 s |
| gyro_bias_estimator | stillness gates `vx<0.01 m/s` AND `wz<0.01 rad/s` for 1 s; EMA τ=30 s; "calibrated" after 10 s accumulated stillness | respawn 2 s |
| ekf_fused_imu_wheel | see §4 | **NO respawn** (restart = origin teleport into carto; loud death preferred) |
| RealSense D435i | rs_camera.launch, 1280×720 @ **15 fps** color+depth, align_depth, pointcloud filter | plain — **ACTIVE** (the "DISABLED [2026-07-10]" comment above it is stale, see Concern C1) |
| FT sensor | netft 192.168.1.7 @ 1000 Hz | plain |
| rosbridge | TLS :9090 | plain |

**ZED Mini — IMU-only since 2026-07-15** (✅live: `depth_mode=NONE`,
`pos_tracking_enabled=false`, zero depth/odom/point-cloud topics advertised):
- Why: 4 SDK-internal segfaults since Jun 21 (SDK 5.0.7 tracking/depth
  internals); the Jul 15 16:23 one killed a feeding run mid-drive.
- `depth_mode=NONE` hard-blocks tracking from ever starting (verified wrapper
  source: `mDepthDisabled` short-circuits `mPosTrackingRequired`), so stray
  subscribers to old VIO topics are silent, not warn-spam.
- Only live products: `imu/data` (~200 Hz) → debiased → EKF vyaw; RGB topics
  publish only if subscribed (RViz has one image panel).
- **SVO2 recording** replaces the VIO witness + gives the nav dataset: raw
  stereo + IMU, H.265 (`svo_compression: 2`), grabbed at HD720@30 regardless
  of ROS pub rate. ✅live: `zed_20260715_180106.svo2` growing at **~3.3 GB/h**
  (in the 2–4 GB/h budget). Depth/VIO/other methods recompute offline from
  replay. Files live OUTSIDE the pruned session bundles.
- All `pos_tracking/*` params (publish_tf=false, area_memory=false,
  two_d_mode=true, …) are inert while tracking is off; kept as the safe
  config if VIO ever returns.

---

## 3. Localization — `cartographer_localization.launch` + `vention_2lidar_localization.lua`

**scan_gate (NEW, committed, ✅live)** — sits between the lidars and
Cartographer only (costmaps keep raw scans):
- Computes per-scan **spatial coverage** (occupied 0.2 m cells among valid
  returns) — raw return count doesn't work, an occluding torso still returns
  ~850 points piled into a few cells.
- Gate closes on a single scan below `min_cells=25` (healthy 36–114; feeding
  occlusion ~10–20) or partner staleness; **drops BOTH lidars together**
  (dropping one desyncs Cartographer's ordered multi-queue); reopens only
  after both are healthy continuously (hysteresis).
- Effect: starved Cartographer freezes `map→odom` on the last good pose and
  the robot rides wheel+IMU odom — fixes the jul14_2 table-stay yank storms
  (10-point scans matching submaps 4–7 m away at 90% score, 2–7 m yanks).
- Threshold is provisional: read the 30 s stats lines from the first gated
  table session and retune (launch arg `gate_min_cells`).

**Cartographer** (pure localization, `load_frozen_state=true`,
`pure_localization_trimmer.max_submaps_to_keep=3`):
| Param | Value | Note |
|---|---|---|
| tracking_frame / published_frame | `vention_base_link` / `odom` | provide_odom_frame=false |
| use_odometry | true (→ `/odometry/fused_imu_wheel` remap) | use_imu_data=false (IMU attempt reverted Jul 12 — colocation CHECK) |
| scans | `scan_1→/lidar_r/scan_gated`, `scan_2→/lidar_l/scan_gated` | via scan_gate |
| min/max range | 0.15 / 12.0 m | |
| motion filter | 0.5 s / 5 cm / 0.5° | gates node creation |
| ceres weights | translation 10, rotation 40 | |
| `POSE_GRAPH.optimize_every_n_nodes` | **25** (was 90) | correction every ~6 s driving / ~12 s parked |
| `tf_publish_lookahead_sec` | **0.05** | post-dates map→odom 50 ms so TEB's now-stamped lookups never future-extrapolate (the self-locking TF-stamp race, session_20260713_173433). **Requires the custom cartographer_ws fork** — see Concern C3 |

Occupancy grid: 0.05 m, 1 s publish.

---

## 4. Odometry fusion — `ekf_fused_imu_wheel` (robot_localization EKF)

Base config `config/nav/ekf_zed_wheel.yaml` + sensors.launch overrides
(✅live: `imu0=/zed_mini/zed_node/imu/data_debiased`, `publish_tf=true`):
| Input | Fused fields | Source notes |
|---|---|---|
| odom0 | (disabled) | `/odometry/_zed_vio_disabled`, never published — VIO retired |
| odom1 `/wheel_odom` | **vx** (twist cov 1e-4) + advisory **vyaw** (cov 0.1) | encoder odometry; rotation slips ~20% hence the weak vyaw |
| imu0 `imu/data_debiased` | **vyaw only** | debiased ZED gyro; per-boot bias lottery handled by the estimator — wait for "first calibration complete" (≥15 s stillness) before driving |

frequency 20 Hz, two_d_mode true, world_frame odom, sensor_timeout 0.2 s,
process noise vx 0.025 / vyaw 0.02. Output `/odometry/fused_imu_wheel`
(✅live, flowing). **No respawn** by design.

---

## 5. Planning — move_base (`launch/navigation.launch` + `config/nav/*`)

**Planners**: global = `global_planner/GlobalPlanner` (switched off silent
navfn default 2026-07-13; Dijkstra+quadratic = navfn-equivalent search, tuned
cost model `cost_factor 0.55 / neutral_cost 66 / lethal 253`); local = **TEB**
(DWA yaml exists but is not loaded).

**Recovery — the Jul 13 violent-spin fix (✅live)**:
- `clearing_rotation_allowed: false` strips both rotate stages from the
  default recovery list → recovery is `conservative_reset → aggressive_reset
  → abort` only.
- Belt-and-suspenders: `TrajectoryPlannerROS/{max_rotational_vel 0.1667,
  min_in_place_rotational_vel 0.05, acc_lim_th 1.5}` are pinned anyway, so a
  re-enabled rotate_recovery could never out-drive supervised teleop caps.
- Third layer: the bridge's `max_speed_units 920` choke (below).

**move_base**: `planner_frequency 1.0` (⚠ its own comment argues for 0.2 —
Concern C2), `controller_frequency 10`, patience 15/15 s.

**Costmaps** (both fed by RAW lidar scans — gating is carto-only):
| | global | local |
|---|---|---|
| frame | map (static map) | odom (rolling 8×8 m, 0.05 m) |
| update / publish | 2 / 1 Hz | 10 / 5 Hz |
| inflation | radius 1.2, scale 2.0 | radius 0.5, scale 5.0 |
| transform_tolerance | 0.5 s | 0.5 s |
| footprint | 0.74×0.59 m rectangle (matches TEB footprint_model exactly) | same |
| obstacle/raytrace range | 8 / 10 m | same |

**TEB** (`teb_local_planner.yaml`, ✅live odom_topic + max_vel confirmed):
| Param | Value | Note |
|---|---|---|
| odom_topic | `/odometry/fused_imu_wheel` | |
| max_vel_x / backwards | **0.065 / 0.065 m/s** | deliberately throttled deployment speed (bridge scales are now truthful; speed governed HERE) |
| max_vel_theta / acc_lim_theta | **0.125 rad/s** / 0.15 | |
| acc_lim_x | 0.7 | |
| goal tolerance | 5 cm / 0.05 rad | |
| max_global_plan_lookahead_dist | 1.0 m | averages over map→odom yaw jitter |
| global_plan_viapoint_sep / weight_viapoint | 0.3 / 5.0 | hunting-tuning history in comments |
| min_obstacle_dist | **0.01 m** | ⚠ contradicts its own comment ("must be ≥ resolution 0.05") — Concern C6 |
| inflation_dist | 0.30 | |
| feasibility_check_no_poses | 1 | |
| homotopy planning | off | |
| oscillation recovery | on (10 s windows) | |

---

## 6. Command path & teleop

**cmd_vel_bridge_basicmicro** (compute): physically calibrated 2026-07-09 —
`linear_scale 4874` (== counts_per_meter, so commanded m/s == actual),
`angular_scale 2600`. `max_speed_units 920` = per-wheel ceiling sized to the
teleop worst case (0.10·4874 + 0.1667·2600); anything hotter (like the old
1.0 rad/s recovery spin) clips at the choke point. Stiction floor
`min_move_units 100` (ratio-preserving). Safety-hold gate present but
**dormant** (no publisher, §8). Teleop `/cmd_vel_teleop` has priority and is
hold-exempt.

**shared_autonomy**: manager (goal ownership, takeover/done/resume) + Xbox
teleop `max_vel_x 0.10 / max_vel_theta 0.1667`, deadman X.

**NUC side**: `base_server.py` owns the Arduino (fw v7 — encoder streaming
mangles ~5 inbound command bytes/s; mitigations live NUC-side), lost-command
stop in BaseInterface, bulldog + Mac heartbeat e-stop.

**Scripted / logged navigation** (`run.py --logged-navigation`, navigate.py):
scripted kitchen legs at speeds read from the TEB yaml; a human teleop input
mid-segment voids the leg → autonomous fallback; **yaw-slip fallback
(2026-07-14)**: on straight segments, |fused yaw − origin yaw| > **15° for 3
consecutive 10 Hz ticks** aborts the leg (`stop_reason="yaw_drift"` →
`slip_fallback` → autonomous navigation). Gyro-informed fused yaw sees the
rotation the encoders miss; threshold ~3× the residual-bias floor; rotate
segments exempt.

---

## 7. Diagnostics, logging, dataset

- **watchdog.py** (arm-side E-stop): FT (1 kHz, force/torque limits),
  RealSense color+aligned-depth rates (threshold 2 Hz) + 1280×720 resolution
  check, collision-free rate, lidar rates, robot state rates. ZED odom check
  retired. ⚠ RealSense checks and the sensors.launch include must stay
  enabled/disabled **together** (Concern C1).
- **sensor_diag_logger** (logger tmux session): 4 streams — zed_imu 200 Hz,
  lidars 8 Hz, realsense caminfo 30 Hz — + kernel/USB/GPU capture.
- **nav_diag_logger** (logger session, every bringup): odom.csv + jump/gap/
  stamp-regression/origin-reset events now watch **`/odometry/fused_imu_wheel`**
  (an EKF-respawn origin teleport trips `odom_origin_reset`); wheel/TF/costmap/
  goals/rosout/trigger-bags as before; sanitizer/ZED-status channels removed.
- **Drift traces** (on-demand, pre-typed in the `trace` tmux window):
  `drift_traces.launch record:=true` → carto (green) / wheel (blue) /
  fused_imu_wheel (gold) + anchor; lock via `drift_lock.py`. ✅live now.
- **Dataset per meal** (NEW `recording` tmux window, pre-typed):
  `record_meal.sh` → `core.bag` (F/T, arm/wrist, odom, IMU, lidar, safety,
  move_base/carto, tf; lz4) + `arm_vision.bag` (RealSense compressed RGB +
  compressedDepth) under `integration/log/bags/meal_<stamp>/` — outside
  SESSION_KEEP pruning, like `log/svo/`. `preflight_check.sh` (run BEFORE
  run.py — LED serial contention): disk ≥150 GB, NUC clock offset, topic
  rates, **SVO growth check**, then interactive peripherals self-test.
  Stop recorders BEFORE killing tmux (SIGHUP'd rosbag leaves .bag.active).
- **Bringup harness** (`feeding-compute.sh`): feeding session panes 1–8
  (roscore, sensors, app, utensil, watchdog, carto-localize, shared_autonomy,
  run.py), `prefix+r` restarts panes 5–8 only; logger session (health monitor
  + sensor logger + nav logger); per-run session bundles with teardown-only
  NUC rsync.

---

## 8. Concerns

**C1 — Stale "DISABLED" comment on an ACTIVE RealSense include**
(sensors.launch ~line 228). The include runs (✅live) and watchdog camera
checks are correspondingly active, so the *system* is consistent — but the
comment says the opposite, and it documents a hard coupling ("if disabled,
watchdog checks MUST be disabled or it E-stops the arm"). Whoever next
toggles either side based on that comment gets burned. Fix: rewrite the
comment to "ACTIVE; toggle together with watchdog camera checks".

**C2 — `planner_frequency` value contradicts its own rationale**
(navigation.launch:49-52). The comment explains 0.2 Hz (1.0 Hz global replans
re-injected carto yaw jitter → ~1 Hz hunting, navlog_20260706) but the value
is `1`, and ✅live it is 1.0. Either the hunting fix was deliberately walked
back (then the comment is wrong) or the value never landed. Decide and align.

**C3 — Silent dependency on the Cartographer fork.**
`tf_publish_lookahead_sec = 0.05` only exists in the local cartographer_ws
fork; upstream Cartographer ignores unknown lua options **silently**... or
errors — either way a rebuild/reinstall on a fresh machine reverts to the
self-locking TF-stamp race with no obvious symptom at bringup. Document the
fork requirement in the workspace README / pin the fork commit.

**C4 — Untracked scripts wired into the harness.** `record_meal.sh` and
`preflight_check.sh` are referenced by feeding-compute.sh's recording window
but not committed. A fresh checkout builds a recording window that pre-types
paths to nonexistent scripts. Commit them (scan_gate.py et al. were committed
in `75cd402`/`9653969`; these two are the stragglers).

**C5 — Harness map default vs newest maps.** `MAP_FILE` in feeding-compute.sh
points at `aimee-7-1.pbstream`, while `aimee-7-15.pbstream` and
`aimee-7-15-2.pbstream` (built today) exist, and zed_drift_test defaults to
`aimee-7-3`. If 7-15 is the current house state, panes launched by the
harness localize against a two-week-old map. Confirm which map is canonical
and align the defaults.

**C6 — TEB `min_obstacle_dist 0.01` violates its own stated rule.** The
comment above it says it must be ≥ the 0.05 m costmap resolution or the
optimizer grazes cells the feasibility check calls collision → the known
"trajectory not feasible / Resetting planner" stall loop (a jul13 incident
ingredient). The `weight_viapoint` comment even names `0.01 → 0.05` as the
companion fix if resets increase. Currently living with the contradiction —
if infeasible-reset spam reappears in navlogs, this is the first knob.

**C7 — No relocalization/alias interlock (accepted gap, Phase-3).**
`/nav_safety_hold` has no publisher since zed_health_monitor was deleted; the
180° table alias has no hard stop. Partially mitigated now by: scan_gate
(kills the occlusion-poisoning that caused most yank storms), faster small
corrections (optimize_every_n_nodes 25), no recovery rotations, and 920-unit
command choke. Residual: a genuine alias jump still moves goals/costmaps with
nothing to halt the base. The subscriber hooks (bridge, manager, navigate.py,
hold.csv) are all kept dormant, so a Phase-3 yank-channel publisher re-arms
everything without code changes.

**C8 — Single points of failure are load-bearing and deliberate.** zed_node
`required=true` (any SDK death = full sensors exit, by RJ rule: fail loud,
restart by hand — the SVO recorder starts a fresh file on relaunch) and the
EKF has no respawn (origin-teleport hazard; `odom_origin_reset` event now
watches for exactly that). Operational discipline, not code, covers both:
restart sensors, wait for gyro "first calibration complete" before driving.

**C9 — Minor stale comments** (cosmetic, batch-fix when convenient):
costmap_common says "expand by 0.35 m" above `inflation_radius: 0.05`;
watchdog says "expected 30 Hz" for a camera running 15 fps (threshold 2 Hz,
so harmless); `custom_param.yaml` in config/nav is unreferenced by any launch.

---

## 9. Live verification snapshot (2026-07-15 ~18:05)

- `depth_mode=NONE`, `pos_tracking_enabled=false`, EKF `imu0=data_debiased`,
  `publish_tf=true` — all live-confirmed.
- Node set matches design exactly: single EKF (no ablation stragglers),
  scan_gate, zed_svo_recorder, drift tracer trio, nav/sensor diag loggers,
  move_base + bridge + shared autonomy, cartographer + grid.
- `/odometry/fused_imu_wheel` flowing; **zero** ZED depth/odom/point-cloud
  topics advertised; `/odometry/_zed_vio_disabled` exists only as the EKF's
  dead subscription.
- SVO: `zed_20260715_180106.svo2` at 229 MB after ~4 min (~3.3 GB/h).
- move_base live: `recovery_behaviors` unset + `clearing_rotation_allowed
  false` (rotate recovery gone), TEB odom_topic/max_vels as in §5,
  `planner_frequency 1.0` (see C2).
