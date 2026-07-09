# Navigation Stack — Architecture, Hardware, Parameters & Diagnosis

> Companion to `docs/navigation_debug.md` (ZED failure taxonomy) and
> `docs/wheel_odom_bringup.md` (encoder/odom bring-up). This document is the
> single reference for how autonomous base navigation works end to end — from
> the lidar photons and motor encoders up through Cartographer, move_base/TEB,
> and the cmd_vel→Arduino→RoboClaw command path — plus a parameter catalog, the
> known bugs (grounded in run-log evidence), and a phased tuning runbook.
>
> Every non-obvious claim carries a `file:line` reference so it can be verified
> with `grep`. Values are current as of the branch this doc was written on;
> re-check anything you are about to tune.

---

## Table of contents
1. [System overview & host split](#1-system-overview--host-split)
2. [Hardware wiring: sensors → motors](#2-hardware-wiring-sensors--motors)
3. [TF tree & frames](#3-tf-tree--frames)
4. [Perception → localization pipeline](#4-perception--localization-pipeline)
5. [Planning: move_base + TEB](#5-planning-move_base--teb)
6. [Execution: cmd_vel → wheels, and the latency budget](#6-execution-cmd_vel--wheels-and-the-latency-budget)
7. [Parameter catalog (every knob, with the ones to tune ★)](#7-parameter-catalog)
8. [Known bugs & config smells](#8-known-bugs--config-smells)
9. [Is the compute laptop a bottleneck?](#9-is-the-compute-laptop-a-bottleneck)
10. [Diagnostic & tuning runbook](#10-diagnostic--tuning-runbook)
11. [Appendix: run-log evidence](#11-appendix-run-log-evidence)

---

## 1. System overview & host split

Autonomous navigation is one capability of a larger feeding executive. You bring
the robot up by running (per `scripts/feeding-compute.sh` on the compute box and
`scripts/feeding-nuc.sh` on the NUC):

- `roscore`
- `roslaunch feeding_deployment sensors.launch`
- `roslaunch feeding_deployment cartographer_localization.launch load_state_filename:=<map>.pbstream`
- `roslaunch feeding_deployment shared_autonomy.launch` (this **includes** `navigation.launch`)
- `python run.py … ` (the executive)

### Two hosts + a cluster

| Host | Address | Runs |
|---|---|---|
| **Compute** (Lenovo laptop) | `192.168.1.2` | `roscore`, **all sensors** (2× RPLIDAR, ZED Mini VIO, nav+arm RealSense, FT), URDF/TF, **Cartographer localization**, **move_base + TEB + cmd_vel bridge + zed_health_monitor**, shared_autonomy manager/teleop, webapp (`npm`), wrist driver, watchdog stack, TTS, health/sensor/nav loggers, `run.py`, rosbridge (TLS 9090). Drives the base over RPC. |
| **NUC** | `192.168.1.3` (user `emprise`) | `arm_server.py`, **`base_server.py`** (owns the base Arduino), bulldog + e-stop. RPC targets for the compute-side cmd_vel bridge and arm client. |
| **E-stop Mac** | `192.168.1.13` | UDP e-stop heartbeat → NUC → bulldog. |
| **Cluster** | — | molmo VLM server (`launch_molmo`), not in the nav loop. |

The base Arduino is **physically plugged into the NUC**, so the compute box can
only command the base through the NUC's RPC server (and the Bulldog e-stop that
guards it). Verify the compute↔NUC hop is **wired Ethernet** — it sits inside the
10 Hz control loop.

### Control flow (goal → wheels)

```
run.py (executive, actionlib client)
  └─ NavigateHLA  ──send_goal──▶  action "navigate"          (FEEDING_NAV_ACTION, default "navigate"; navigate.py:239)
        └─ shared_autonomy_manager.py   (SimpleActionServer, a MoveBaseAction passthrough)
              └──forward──▶  action "move_base"   (move_base + global planner + TEB)
                    └─ /cmd_vel ──▶ cmd_vel_bridge_basicmicro.py  (compute)
                          └──RPC set_speeds──▶ base_server.py / BaseInterface  (NUC)
                                └─ VentionBase ──serial "A=.. B=.."──▶ Arduino
                                      └──packet serial──▶ 2× RoboClaw ──▶ 4 motors
```

Two side-channels wrap this:
- **Teleop / shared autonomy:** Xbox / webapp publish `/shared_autonomy/{takeover,done,resume,cancel}`; the manager cancels move_base and can itself report `SUCCEEDED` for a human-completed goal (an actionlib client only trusts its own server — that is *why* the manager exists, `shared_autonomy_manager.py:40-43`). Human teleop drives `/cmd_vel_teleop` with priority (bypasses the ZED hold).
- **Safety interlock:** `zed_health_monitor.py` asserts `/nav_safety_hold` (+ `_reason`) on VIO divergence or a map→odom "yank"; the cmd_vel bridge zeros the motors while held, and the manager pauses & re-sends the goal on recovery.

### Localization TF ownership
Cartographer owns `map→odom`; the ZED VIO owns `odom→zed_mini_base_link`; the URDF
chains that to `vention_base_link`. Nobody else publishes those edges (see §3).

---

## 2. Hardware wiring: sensors → motors

Grounded in `hardware.txt` (BOM), the Arduino firmware (`PacketSerialSetSpeed.ino`),
`vention_arduino_control.py`, and `docs/wheel_odom_bringup.md` /
`docs/networking_diagnostics.md`.

### Drivetrain (bottom of the stack)
- **4× goBILDA 5303 Saturn planetary gearmotors** — 71.2:1, 260 RPM, 8 mm REX shaft,
  3.3–5 V quadrature encoder. Nominal **28 counts/motor-rev × 71.2 = 1993.6
  counts/wheel-rev**; over a 96 mm wheel (0.30159 m/rev) with 1:1 miter gears ⇒
  ~6610 counts/m nominal. **Measured `counts_per_meter = 4874.0`**
  (`wheel_odom_publisher.py:88`, tape-measured 2026-07-08).
- **4× 96 mm goBILDA Gecko wheels**, **1:1 clamping miter gears** (8 mm REX, 24T).
  4-wheel **skid steer**: geometric wheelbase ≈ 0.41 m, but the effective track is
  **`track_width_m = 0.85`** (~2× geometric, from scrub) and **yaw is advisory**
  (`wheel_odom_publisher.py:95-104`).
- **2× RoboClaw ST 2x45A** motor controllers, one per L/R wheel pair. Arduino↔RoboClaw
  is **packet serial @9600 baud** (`PacketSerialSetSpeed.ino` `CONTROLLER_BAUD=9600`),
  both controllers at **address 128 (0x80)**, distinguished by separate SoftwareSerial
  buses: **Driver A = RIGHT** (RX10/TX11), **Driver B = LEFT** (RX8/TX9). Closed-loop
  velocity via `SpeedM1/M2` in **encoder counts/s**.

### Microcontroller & serial to the NUC
- **Arduino Uno R3 (ATMEGA328P)** running `PacketSerialSetSpeed.ino`.
  - USB↔NUC at **115200 baud** (`vention_arduino_control.py:45`), stable by-id path
    `…usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00`
    (`vention_arduino_control.py:44`, `base_server.py:19`).
  - Streams encoder telemetry as `E <millis> <a1> <a2> <b1> <b2> <okA> <okB>` at
    poll `ENC_POLL_MS=100` (~10 Hz; measured powered E-rate ≈ 8.8 Hz,
    `wheel_odom_bringup.md:60`). Firmware fresh-command watchdog **`CMD_STALE_MS=5000`**.
  - Per-command echo `Parsed A=.. B=..` — used by the host echo-confirm re-send
    (a SoftwareSerial encoder read can mangle an inbound command line).
- **NUC** runs `base_server.py`, which owns the single `VentionBase` serial
  connection and serves `BaseInterface` over `multiprocess.managers` RPC on
  **`192.168.1.3:5001`** (`base_interface.py:21-23`). It also hosts the
  **authoritative lost-command watchdog** (`BASE_CMD_TIMEOUT=0.3 s`,
  `base_interface.py:25`) and the Bulldog heartbeat gate.

### Command path from the compute box
`cmd_vel_bridge_basicmicro.py` (compute) subscribes `/cmd_vel` (autonomous) and
`/cmd_vel_teleop` (human), converts a Twist to left/right counts/s, and calls
`BaseInterfaceClient.set_speeds()` over RPC. See §6 for the exact mixing math and
the latency budget.

### Power & safety
- **3× Power-Sonic PS-1290 12 V 9 Ah SLA batteries.**
- **2× 22 mm latching E-stops** (IP65, 1NO1NC).
- **E-stop chain:** the Mac (`192.168.1.13`) streams a ~82–100 Hz UDP heartbeat to
  the NUC → `/experimentor_estop`; `bulldog.py` debounces (threshold **30 Hz**,
  `ESTOP_FREQ_GRACE_S=1.0`, `networking_diagnostics.md:34-40`) and calls
  `emergency_stop()` on the arm and base. Because the base Arduino is on the NUC,
  the same Bulldog stop reaches the wheels.

### Sensors
- **2× RPLIDAR A1** (`rplidar_ros`/`rplidarNode`), USB @115200, `angle_compensate=true`,
  driver defaults for scan rate (~5.5 Hz) and angle/range (SLAM range is clamped
  downstream by Cartographer, §7). Namespaces `lidar_r` / `lidar_l`; frames
  `lidar_r` / `lidar_l`. **⚠ the USB by-path ports differ between launch files**
  (`rplidar_a1.launch` uses `…13.4.x`, `sensors.launch`/`base_sensors.launch` use
  `…4.4.x`) — verify which hub the lidars are on for a given rig.
- **ZED Mini** (`zed_wrapper/zedm.launch`, node `/zed_mini/zed_node`): USB3, VIO
  (camera + IMU fused internally, `imu_fusion=true`). Grab 30 Hz / **publish 15 Hz**
  (`pub_frame_rate`). Publishes `nav_msgs/Odometry` on `/zed_mini/zed_node/odom`
  and owns TF `odom→zed_mini_base_link`.
- **Nav RealSense D435** (`vention_base_cam_1_link`, front), **arm RealSense**
  (`camera_link`), **FT sensor** (`192.168.1.7` in `sensors.launch:137`, `192.168.1.4`
  in `arm_sensors.launch` — **flag: the two launch families disagree**).

### Network
Netgear Nighthawk RAX43v2, SSID `FeedingDeployment-5G`, **5 GHz channel 44, 80 MHz,
Smart Connect OFF** (DFS-avoidance, `networking_diagnostics.md:16-30`). rosbridge on
TLS **9090**.

---

## 3. TF tree & frames

```
map                          ← Cartographer (localization mode) publishes map→odom
 └─ odom                     ← ZED VIO publishes odom→zed_mini_base_link
     └─ zed_mini_base_link
         └─ vention_base_link   ← URDF INVERTED fixed joint (robot_state_publisher)
             ├─ lidar_r          ← static_transform_publisher
             ├─ lidar_l          ← static_transform_publisher
             ├─ vention_base_cam_1_link   ← URDF joint (nav D435)
             └─ world → arm_base_link → …  ← Kinova arm chain (combined.urdf only)
```

Cartographer config: `tracking_frame="vention_base_link"`, `published_frame="odom"`,
`provide_odom_frame=false`, `publish_frame_projected_to_2d=true`
(`vention_2lidar_localization.lua:9-14`).

**The inverted ZED joint (important quirk).** In the URDF, `vention_base_link` is a
*child* of `zed_mini_base_link`, so `robot_state_publisher` provides
`zed_mini_base_link → vention_base_link` while the ZED provides
`odom → zed_mini_base_link`, chaining to a full `odom → vention_base_link`
(`zed.launch:11-22`, `urdf/combined.urdf.xacro:40-47`). ZED `publish_map_tf=false`
so Cartographer owns `map→odom` uncontested.

### Static transforms (translation m; rpy rad; parent `vention_base_link`)
| Child | xyz | rpy | Publisher |
|---|---|---|---|
| `lidar_r` | (0.2575, −0.135, 0.415) | (π, 0, 0) | `static_transform_publisher` (`sensors.launch:47-49`) |
| `lidar_l` | (0.2575, +0.135, 0.415) | (π, 0, 0) | `static_transform_publisher` (`sensors.launch:50-51`) |
| `zed_mini_base_link`→`vention_base_link` | (−0.3176, 0, −0.386) | (0,0,0) | URDF (`combined.urdf.xacro:43-47`) |
| `vention_base_cam_1_link` | (0.3176, 0, 0.386) | (0,0,0) | URDF ("TODO measured") |
| `world` (arm root) | (0.212, 0, 0.39) | (0,0,−π/2) | URDF (combined only) |

Both lidars are ~0.2575 m forward, 0.415 m up, ±0.135 m lateral, **rolled 180°**
(mounted upside-down). Coverage at that height matters for tight-space collision
checking (§10, Phase 5).

---

## 4. Perception → localization pipeline

### Lidar → Cartographer
Cartographer consumes the **two raw scans directly**:
`scan_1→/lidar_r/scan`, `scan_2→/lidar_l/scan` (`cartographer_localization.launch:14-15`),
with `num_laser_scans=2`. **`merge_laserscans.py` / `/scan_merged` is dead code** —
launched by nothing (grep-verified). move_base costmaps also read the two raw scans
(`costmap_common.yaml:26-42`).

### ZED odom → sanitizer/differentiator (`zed_pose_to_odom_feedback.py`)
The raw ZED odom is *never* consumed directly by the planner or SLAM. This node
produces two derived streams:
- **`/zed_mini/zed_node/odom_sanitized`** — full pose with single-frame VIO teleports
  gated; **this is what Cartographer consumes** (`cartographer_localization.launch:19`).
  A physically-impossible frame-to-frame jump is held at the last good pose; a
  *sustained* shift is **adopted after `jump_accept_frames=5` consistent raw frames**
  as a relocalization (`zed_pose_to_odom_feedback.py:147-154`) — see bug #2.
- **`/move_base/odom_feedback`** — a twist obtained by differencing the sanitized pose
  over `vel_diff_window=0.08 s`; **this is what TEB uses as its velocity feedback**
  (`teb_local_planner.yaml:3`). Gates: `max_lin_vel=0.5`, `max_ang_vel=1.5`
  (`odom_pipeline.yaml`).

Ablation switches live in `config/nav/odom_pipeline.yaml` (`enable_sanitizer`,
`enable_windowed_diff`) — false = original pass-through behavior; they change only
the math, not the wiring.

### Cartographer (localization mode)
Runs against a **frozen `.pbstream`** (`load_frozen_state=true`), publishing `map→odom`
**only at pose-graph optimizations**, which are counted in **nodes**, and node
insertion is motion-filter-gated. So corrections arrive as **discrete jumps**, not a
smooth stream — the robot's map-frame pose = `map→odom` (jumpy) ∘ `odom→base` (smooth
VIO). This is the structural reason heading corrections can look like overshoot (§8).

### ZED health interlock (`zed_health_monitor.py`)
Five detector channels — **status, silence, gap, jump, yank** — assert
`/nav_safety_hold`. The **yank** channel watches `map→odom` for a Cartographer
relocalization/alias jump (e.g. the recurring 180° table alias) and, above the gates,
holds the base *and clears move_base costmaps*. Disable the whole chain with
`zed_interlock:=false` or `NO_ZED_INTERLOCK=1`; per-channel with
`enable_yank_channel:=false` (`navigation.launch:62-89`).

---

## 5. Planning: move_base + TEB

`navigation.launch` starts **move_base** with:
- `base_local_planner=teb_local_planner/TebLocalPlannerROS` (`navigation.launch:12`)
- costmaps: `costmap_common.yaml` into both namespaces, then `global_costmap.yaml`
  (static map, `map` frame) and `local_costmap.yaml` (8×8 m rolling, `odom` frame).
- **`planner_frequency=1`** (global replan Hz), **`controller_frequency=10`** (TEB Hz),
  `planner_patience=controller_patience=15` (`navigation.launch:21-28`).

**Pose vs velocity into TEB.** TEB gets the robot's *pose* from TF
(`map→odom ∘ odom→base`, via the global costmap `map` frame) and its *velocity seed*
from `odom_feedback`. So Cartographer's discrete `map→odom` snaps enter TEB through
the pose and — because global replans re-express the plan's orientation from the
current estimate (`global_plan_overwrite_orientation: true`) — through the replanned
plan once per `planner_frequency` tick (bug #3).

**Global planner.** Not explicitly set → move_base default `NavfnROS`
(`nav_diag_logger` records `/move_base/NavfnROS/plan`).

**shared_autonomy_manager** is a transparent `navigate`→`move_base` passthrough that
adds the takeover/hold/resume state machine and can report a human-completed
`SUCCEEDED` (`shared_autonomy_manager.py`).

**`navigate.py` extras** (executive-side, not move_base): a localization-stall
watchdog (30 s per-incident), a post-arrival confirm-replan (`_GOAL_CONFIRM_SETTLE_S`),
a **heading-only refinement window** (`_refine_cmd`), and a learned per-location SE(2)
`PositionOffset`. The refinement window rotates in place toward the goal heading and
**never translates**, on the (now partly stale) assumption that rotation has no
actuator floor — see bug #12.

---

## 6. Execution: cmd_vel → wheels, and the latency budget

### The mixing + stiction floor (`cmd_vel_bridge_basicmicro.py:216-265`)
```
w = 0 if |w| < w_deadband (0.03 rad/s)          # angular deadband
lin_units = v * linear_scale   (800)
rot_units = w * angular_scale  (600)
right = lin_units + rot_units ; left = lin_units - rot_units   # diff-drive mix
peak = max(|right|, |left|)
if 0 < peak < min_move_units (250):                            # ratio-preserving stiction floor
    scale = 250 / peak ; right *= scale ; left *= scale        # BOTH wheels up, ratio preserved
right, left = clamp(±max_speed_units = 2500)
base.set_speeds(right, left)     # RPC to NUC
```
The floor is **ratio-preserving** (curvature kept), which is correct for arcs, but for
**near-pure rotation** the peak wheel = `|ω|·600`, so any `ω` below `250/600 =
0.417 rad/s` is snapped up to 0.417 rad/s. Combined with the deadband this makes the
angular channel a **deadband + relay** (bug #4): dead zone up to 0.03, a step to
0.417, then linear. Minimum executable in-place turn ≈ 0.417 rad/s = 83% of
`max_vel_theta`. At 10 Hz one tick = 0.417×0.1 = **2.4°**, larger than the ~1.43°
refinement success band — a guaranteed heading limit cycle. **Confirmed in logs**:
82.8% of in-band in-place rotations applied at exactly 0.4167 rad/s (§11).

### The command path & its latencies (no acceleration ramp — every delay integrates into heading)
| Hop | Mechanism | Approx. latency | Notes |
|---|---|---|---|
| TEB → `/cmd_vel` | ROS topic | 100 ms period | `controller_frequency=10` |
| `/cmd_vel` → bridge cb | ROS transport | ~ms; **no `tcp_nodelay`** | `queue_size=10` (bug #8, latent) |
| bridge → NUC | **blocking** `multiprocess` RPC | link-dependent | verify wired |
| NUC → serial | `VentionBase._send_setpoints` | write+flush ~1 ms | **20 Hz rate-limit / 5 Hz same-setpoint / echo-confirm** (`vention_arduino_control.py:369-419`) |
| Arduino → RoboClaw | packet serial @9600 | ~11 ms (11-byte cmd) | closed-loop velocity on RoboClaw |
| RoboClaw → wheels | closed-loop | motor time constant | **no host-side accel ramp** |
| encoders → `/wheel_odom` | RoboClaw→Arduino ~10 Hz → NUC cache → RPC poll 20 Hz | — | **observational only, not in the control loop** |

**Two safety timeouts:** the NUC's `BASE_CMD_TIMEOUT=0.3 s` (no `set_speeds` → stop)
and the Arduino's `CMD_STALE_MS=5000`. The bridge itself owns no watchdog.

**Stale-command lag was measured, not assumed:** cmd→applied sign-flip median lag is
**0 ms** (p90 1 ms) — so `queue_size=10` is a *latent* defensive fix, **not** the
observed overshoot cause (§8 bug #8, §11).

---

## 7. Parameter catalog

Every exposed knob with its current value and location. **★ = tune this for the
current symptoms.** "inherited" = not set in-repo, takes the upstream default (verify
against the installed package version).

### 7.1 Cartographer — `config/vention_2lidar_localization.lua`
| Param | Value | Line | Note |
|---|---|---|---|
| ★ `POSE_GRAPH.optimize_every_n_nodes` | 25 | :62 | correction cadence (nodes, not seconds); 90 in mapping lua |
| ★ `motion_filter.max_distance_meters` | 0.05 | :50 | node emitted every 5 cm driving |
| ★ `motion_filter.max_time_seconds` | 0.5 | :49 | node every 0.5 s parked |
| ★ `motion_filter.max_angle_radians` | rad(0.5) | :51 | |
| ★ `use_online_correlative_scan_matching` | **false** (inherited) | — | no CSM to seed Ceres at speed |
| ★ `constraint_builder.fast_correlative_scan_matcher.linear_search_window` | **7.0 m** (inherited) | — | the "7 m search radius" (`navigation_debug.md:7`) |
| `max_range` / `missing_data_ray_length` | 12.0 / 12.0 | :41-42 | trimmed from 30 |
| `min_range` | 0.15 | :40 | |
| `ceres_scan_matcher.translation_weight` / `rotation_weight` | 10 / 40 | :53-54 | |
| `pure_localization_trimmer.max_submaps_to_keep` | 3 | :45-47 | localization-only |
| `use_imu_data` | false | :39 | |
| `pose_publish_period_sec` | 0.01 (100 Hz) | :27 | |
| `submap_publish_period_sec` | 0.3 | :26 | |
| `lookup_transform_timeout_sec` | 0.2 | :25 | |
| `global_constraint_search_after_n_seconds` | 10 (inherited) | — | delays first global relocalization |
| `constraint_builder.min_score` / `global_localization_min_score` | 0.55 / 0.6 (inherited) | — | |
| `MAP_BUILDER.num_background_threads` | 4 (inherited) | — | |

### 7.2 ZED wrapper — `zed_wrapper` defaults (overridden bits in `sensors.launch:84-88`)
| Param | Value | Note |
|---|---|---|
| ★ `pos_tracking/area_memory` | **true** (default) | loop closure = relocalization jump source (bug #1) |
| ★ `general/pub_frame_rate` | 15.0 | published odom/image rate; USB-bandwidth lever |
| ★ `pos_tracking/two_d_mode` | false | commented-out attempt in `base_sensors.launch:101-103` |
| `general/grab_resolution` / `grab_frame_rate` | HD720 / 30 | |
| `depth/depth_mode` | ULTRA | heavy; consider PERFORMANCE for USB headroom |
| `pos_tracking/imu_fusion` | true | VIO |
| `pos_tracking/publish_tf` / `publish_map_tf` | true / **false** | ZED owns odom→base only (`sensors.launch:84-85`) |
| `pos_tracking/base_frame` | `zed_mini_base_link` | ⚠ wrapper's own `general/base_frame` stays `base_link` (latent) |

### 7.3 odom sanitizer — `config/nav/odom_pipeline.yaml`
| Param | Value | Note |
|---|---|---|
| ★ `jump_accept_frames` | 5 | adopt-after-N; the leak lever (bug #2) |
| `enable_sanitizer` / `enable_windowed_diff` | true / true | ablation off = original behavior |
| `max_lin_vel` / `max_ang_vel` | 0.5 / 1.5 | teleport gate |
| `vel_diff_window` | 0.08 s | TEB velocity noise floor |

### 7.4 TEB — `config/nav/teb_local_planner.yaml`
| Param | Value | Line | Note |
|---|---|---|---|
| ★ `min_obstacle_dist` | 0.01 | :54 | **below one 0.05 costmap cell** → grazing/infeasible (bug #5) |
| ★ `weight_viapoint` | 1.0 | :93 | pull to (jittery) plan; was 5.0 |
| ★ `weight_obstacle` | 100.0 | :89 | |
| ★ `max_global_plan_lookahead_dist` | 1.0 | :15 | longer band averages over yaw jitter |
| `max_vel_x` / `_backwards` | 0.4 / 0.4 | :25-26 | (was 0.2 in the mined runs) |
| `max_vel_theta` | 0.5 | :28 | |
| `acc_lim_x` / `acc_lim_theta` | 0.7 / 1.5 | :29-30 | **planner-model only; actuator has no ramp** |
| `dt_ref` / `dt_hysteresis` | 0.2 / 0.1 | :10-11 | |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.05 / 0.05 | :42-43 | |
| `enable_homotopy_class_planning` | true | :101 | multi-route; `max_number_classes=2` |
| `oscillation_recovery` (+ v/omega eps, durations) | true / 0.1 / 0.1 / 10 s | :126-130 | |
| `feasibility_check_no_poses` | 1 | :19 | (2/4 in mined runs) |

### 7.5 Costmaps — `config/nav/{costmap_common,global_costmap,local_costmap}.yaml`
| Param | Value | Note |
|---|---|---|
| `footprint` | [[0.37,0.295],[0.37,−0.295],[−0.37,−0.295],[−0.37,0.295]] | 0.74×0.59 m |
| ★ `inflation_layer.inflation_radius` | **0.05 in common, overridden to 0.5** by local/global | the 0.05 is dead config (bug #13) |
| `inflation_layer.cost_scaling_factor` | 5.0 | local/global |
| `obstacle_range` / `raytrace_range` | 8.0 / 10.0 | common |
| `local_costmap` size / res / update / publish | 8×8 m / 0.05 / 10 Hz / 5 Hz | rolling, `odom` frame |
| `global_costmap` update / publish | 2 Hz / 1 Hz | static, `map` frame |
| `transform_tolerance` | 0.5 | both |

### 7.6 cmd_vel bridge — `launch/navigation.launch:46-51`
| Param | Value | Note |
|---|---|---|
| ★ `min_move_units` | 250 | stiction floor; = 0.417 rad/s / 0.3125 m/s min (bug #4) |
| `linear_scale` / `angular_scale` | 800 / 600 | m/s, rad/s → counts/s |
| `w_deadband` | 0.03 (default) | angular deadband |
| `max_speed_units` | 2500 | wheel clamp |

### 7.7 move_base — `launch/navigation.launch:21-28`
| Param | Value | Note |
|---|---|---|
| ★ `planner_frequency` | 1 | global replan Hz; comment intends 0.2 (bug #3) |
| `controller_frequency` | 10 | TEB Hz |
| `planner_patience` / `controller_patience` | 15 / 15 | |

### 7.8 zed_health_monitor — `launch/navigation.launch:62-89`
`max_lin_vel=0.5`, `max_ang_vel=1.5`, `recovery_stable_s=2.0`, `enable_yank_channel=true`,
`yank_lin_m=0.5`, `yank_ang_rad=0.4`, `yank_settle_s=5.0`, `release_mo_rate_rad_s=0.03`,
`max_hold_s=45.0`.

---

## 8. Known bugs & config smells

Ordered by *evidenced* impact (see §11 for the numbers). Nothing here is auto-fixed —
each is a runbook item.

| # | Issue | Location | Evidence | Effect | Fix |
|---|---|---|---|---|---|
| 1 | **ZED VIO teleports** (frame loss → velocity blow-up) | ZED wrapper / USB | 7292 frames >1 m/s, max **1116 m / 2984 m/s**; 46 CORRUPTED, 2 REBOOTING (run dir1) | Corrupts localization; cascades everywhere | USB3 bandwidth/cable audit; `area_memory:=false`; lower `pub_frame_rate`/`depth_mode`; ZED on its own controller |
| 2 | **Sanitizer "adopt sustained shift" leaks big jumps** | `zed_pose_to_odom_feedback.py:147-154` | 537 adopt events; leaked steps up to **1089 m** into `odom_sanitized` | Multi-hundred-m teleports reach Cartographer | Cap adopted-jump magnitude; require lidar agreement before adopting |
| 3 | **`planner_frequency=1` re-injects Cartographer yaw jitter** | `navigation.launch:24-27` | ~1.2 Hz vx+ω hunting both runs; 31–35 oscillation recoveries | Jittery path following | Set 0.2 Hz; decouple plan orientation from Cartographer yaw |
| 4 | **Stiction floor = deadband + relay on ω** (min 0.417 rad/s; 2.4°/tick > 1.43° band) | `cmd_vel_bridge:240`, `navigation.launch:51` | 82.8% of in-place rotations applied at exactly 0.4167 rad/s | Heading bang-bang on in-place turns | Lower/remove angular floor; add host-side accel ramp |
| 5 | **`min_obstacle_dist=0.01` (< 1 cell) + tight inflation** | `teb_local_planner.yaml:54`, costmaps | footprint held a lethal cell 78.8%/47.8% of the time; 1889/1286 "not feasible" resets | Collisions, chronic infeasibility | Raise `min_obstacle_dist` ≥ 0.05; audit inflation/footprint |
| 6 | **180° yaw alias (table)** in map→odom | Cartographer + symmetric env | 70 ~180° yaw yanks (dir2) | Wrong heading lock → mis-parks | Yank channel (exists) + break view symmetry |
| 7 | `use_online_correlative_scan_matching=false` + tight motion_filter + 7 m search | `vention_2lidar_localization.lua` | 8117/19604 pose-lookup extrapolation errors (pipeline-driven) | Can't-keep-up-at-speed | Enable CSM; relax motion_filter |
| 8 | `/cmd_vel` `queue_size=10` + blocking RPC | `cmd_vel_bridge:139,263` | **REFUTED as observed cause** (cmd→applied lag median 0 ms) | Latent risk only | Defensive: `queue_size=1`, `tcp_nodelay=True` |
| 9 | Rate-limit drops newest *changed* setpoint (<50 ms) | `vention_arduino_control.py:408-414` | not isolated | Latent delayed stop/reversal | Never gate a *changed* setpoint |
| 10 | Serial reconnect blocks command path secs (holds `_send_lock`) | `vention_arduino_control.py:107,143` | not isolated | Runaway during a serial hiccup | Non-blocking reconnect; release lock |
| 11 | Slow first fix | lua + launch stagger + governor | node-cadence math (~12 s parked); governor `powersave` | Slow startup | performance governor; front-load a global constraint |
| 12 | Stale docstrings ("1 Hz re-send"; "rotation has no floor / `min_rot_units=0`") | `base_interface.py:12`, `navigate.py:722`, `custom_param.yaml:9` | code review | Misleads tuning (refinement assumes smooth rotation) | Correct the docs |
| 13 | Dead code / config drift | various | grep | Confusion during tuning | `merge_laserscans.py` unused; `costmap_common` inflation 0.05 overridden; FT ip .7 vs .4; lidar by-path ports differ |

---

## 9. Is the compute laptop a bottleneck?

**No — capacity is not the limit.** The compute box is an **Intel i9-14900HX (24C/32T),
RTX 4090 Laptop (16 GB), 31 GB RAM**. During the mined runs, `sys_1hz.csv` shows mean
1-min load **3.5–5.4 on 32 cores (~11–17%)**, with `load>8` for <3% of the run. The
8117 tf-extrapolation errors are explained by the **broken ZED odom pipeline, not CPU
starvation**.

Cheap, secondary wins worth doing anyway:
- **CPU governor is `powersave`** — pinned at 2.4 GHz even idle. Set `performance`
  (helps bursty Cartographer optimization latency). Helper: `scripts/perf_env.sh`.
- **CPU temperature briefly touches 100 °C** — transient thermal throttling is
  possible; check airflow/cooling.
- **RViz RobotModel 5M-tri mesh** at ~20–40% GPU can starve the ZED VIO USB path
  (documented) — decimate or disable RobotModel during runs.
- **Everything runs on one box** — pin Cartographer to P-cores if you see contention,
  and confirm the compute↔NUC link (and the ZED's USB3 controller) are not shared.
- Memory pressure (swap in use, ~1.6 GB free) — watch via `compute_health_monitor`,
  but it was not implicated in the logs.

---

## 10. Diagnostic & tuning runbook

Bottom-up: trust each layer before the one above depends on it. Every step is
**isolate → measure with a named tool → change (you apply) → re-measure**. New
harnesses (§ scripts) complement the existing tools (`nav_diag_logger.py`,
`drift_traces.launch`, `measure_localization.py`, `nav_residual.py`,
`calibrate_wheel_odom.py`).

**Priority:** ZED integrity (Phase 2) is the dominant, everything-cascades-from-here
problem — you cannot tune any higher layer over a teleporting ZED. Phase 1 (actuation)
is independent and fast; do it in parallel. Because the mined logs predate the current
config, Phase 0 ends with a **fresh baseline**.

### Phase 0 — Environment + fresh baseline (no tuning yet)
- `scripts/perf_env.sh` → CPU governor `performance`; check temp headroom; verify
  compute↔NUC is wired and the ZED owns its own USB3 controller; decimate/disable the
  RViz RobotModel mesh; `rosparam get` the odom_pipeline flags to confirm they are
  live; `git status` clean.
- **Capture a fresh `nav_diag_logger` baseline on the CURRENT config**
  (`rosrun feeding_deployment nav_diag_logger.py`), a short fridge→microwave→table
  loop. This is the before/after reference for every later phase.

### Phase 1 — Actuation & kinematics (independent, parallel)
Tools: `scripts/base_cmd_path.py` (new), `calibrate_wheel_odom.py`, `bench_test_encoders.py`.
- Calibrate `counts_per_meter` / `track_width`; verify encoder signs (`--creep`).
- Characterize the **stiction relay**: `base_cmd_path.py --mode rotate` sweeping ω;
  confirm min executable ω ≈ 0.417 rad/s and the 2.4°/tick vs 1.43° band.
- Apply (runbook): lower/remove the angular floor or add a host-side accel ramp
  (**the confirmed rotation-bang-bang fix**). The `queue_size=1` + `tcp_nodelay` and
  "never drop a changed setpoint" changes are **defensive only** (lag ≈ 0 ms in logs).
- **Metric:** commanded-vs-applied overshoot on a fixed rotate ≤ target; smooth arcs.

### Phase 2 — Odometry / ZED (HIGHEST SEVERITY — gates 3–5)
Tools: `drift_traces.launch record:=true` + `drift_trace_compare.py` + `drift_lock.py`.
- Drive a fixed loop; watch raw(red)/sanitized(orange) vs carto(green) vs wheel(blue).
  Correlate any teleport with `rosout` `CORRUPTED`/`REBOOTING` and USB load.
- **Fix the input first:** USB3 bandwidth (dedicated controller, good cable, not
  shared with the RealSenses); `area_memory:=false`; consider `two_d_mode`, lower
  `pub_frame_rate`/`depth_mode`.
- **Then harden the sanitizer** (bug #2): cap the magnitude a single "adopt sustained
  shift" can accept and/or require lidar agreement, so a persistent 1 km teleport can
  never be re-accepted as a relocalization.
- **Metric:** zero leaked sanitized steps > (say) 0.3 m over a loop; raw jump rate down.

### Phase 3 — Localization / Cartographer
Tool: `measure_localization.py` (static + moving).
- Measure map→odom jitter, first-fix time, and **max speed before divergence**.
- *Slow startup:* keep `optimize_every_n_nodes` low but front-load a global constraint;
  verify pbstream size; performance governor (Phase 0).
- *Can't-keep-up-at-speed:* enable `use_online_correlative_scan_matching=true`, relax
  `motion_filter`, revisit the 7 m search window.
- **Metric:** first fix < N s; map→base jitter within goal tolerance at target speed.

### Phase 4 — Local planning / TEB
Tool: `scripts/teb_shape_test.py` (new).
- Tune on a fixed line / arc / in-place rotation: `planner_frequency` (→ 0.2),
  `weight_viapoint`, `weight_obstacle`, `max_global_plan_lookahead_dist`, oscillation
  params.
- **Metric:** small plan-tracking error; no ω sign-flip hunting in `cmd_vel.csv`.

### Phase 5 — Collision / costmap
Tool: `scripts/costmap_collision_probe.py` (new).
- Raise `min_obstacle_dist` (0.01 → ≥ 0.05); reconcile the inflation override; check
  footprint vs the real robot; verify 2-lidar coverage at 0.415 m in tight spaces.
- **Metric:** footprint never holds a lethal cell on a nominal pass; no grazing.

### Phase 6 — Integration
Tool: `nav_diag_logger.py`.
- Full fridge↔microwave↔table legs; compare `events.csv`/`goals.csv` against the
  Phase 0 baseline.

---

## 11. Appendix: run-log evidence

Two `nav_diag_logger` runs were mined (both **predate the current config** —
snapshots show `max_vel_x=0.2`, `dt_ref=0.3`; current disk is 0.4 / 0.2):
- **dir1** = `navlog_20260707_123511` (git `e582a9b`) — a badly-degraded ZED run.
- **dir2** = `navlog_20260706_131121` (git `02e2261`) — the milder "1 Hz hunting" run.

| Finding | dir1 | dir2 | Verdict |
|---|---|---|---|
| ZED frames >1 m/s | 7292 (0.94%); max 1116 m / 2984 m/s | 121 (0.03%); max 10.7 m | teleports **confirmed** |
| Sanitizer adopt/leak | 537 adopts; sanitized steps up to 1089 m | 18 adopts; max 12.5 m | leak path **confirmed** (bug #2) |
| map→odom | std 9.7 m; 3288 yanks; yaw max 179° | transl bounded; **70 ~180° yaw yanks** | alias **confirmed** |
| move_base "Extrapolation … →map" | 8117 | 19604 | pose-lookup starvation **confirmed** |
| In-place rotation floored to 0.4167 rad/s | 82.8% of in-band | — | relay **confirmed** (bug #4) |
| cmd→applied sign-flip lag | median 0 ms, p90 1 ms | median 0 ms | stale-command **refuted** (bug #8) |
| ω/vx hunting (worst 60 s) | 2.38 flips/s (~1.19 Hz) | 2.50 flips/s (~1.25 Hz) | hunting **confirmed** (bug #3) |
| TEB infeasible resets / costmap dumps | 1889 / 65 | 1286 / 144 | infeasibility **confirmed** (bug #5) |
| Footprint held a lethal cell | 78.8% of active time | 47.8% | grazing **confirmed** |
| CPU load (32 cores) | mean 3.5 (~11%), peak 32.3 | mean 5.4 (~17%) | compute **not** the bottleneck |
| CPU temp | mean 76 °C, peak 100 °C | mean 84 °C, peak 100 °C | transient thermal risk |
| Goals succeeded / started; park p90 | 25 / 87; p90 1.4 m | 73 / 147; p90 0.10 m | dir1 heavy-tailed from teleports |

> Reproduce any row with the CSVs under
> `src/feeding_deployment/integration/log/system_logs/<navlog_dir>/` — see
> `scripts/nav_diag_logger.py` for the column schemas.
