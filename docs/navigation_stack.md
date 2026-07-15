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
12. [Base command calibration & measured kinematics](#12-base-command-calibration--measured-kinematics)

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
| **Compute** (Lenovo laptop) | `192.168.1.2` | `roscore`, **all sensors** (2× RPLIDAR, ZED Mini [IMU-only + SVO recording since 2026-07-15], nav+arm RealSense, FT), URDF/TF, **Cartographer localization**, **move_base + TEB + cmd_vel bridge**, shared_autonomy manager/teleop, webapp (`npm`), wrist driver, watchdog stack, TTS, health/sensor/nav loggers, `run.py`, rosbridge (TLS 9090). Drives the base over RPC. |
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
- **Safety interlock:** DORMANT — `/nav_safety_hold` has no publisher since `zed_health_monitor` was deleted (2026-07-15, IMU-only ZED). The subscriber hooks remain (bridge zeros motors while held; manager pauses & re-sends on release) and re-engage if a Phase-3 interlock republishes the topic.

### Localization TF ownership (rewired 2026-07-13, Phase-2 cutover)
Cartographer owns `map→odom`; the **fused wheel+IMU EKF** (`ekf_fused_imu_wheel` in
`sensors.launch`) owns `odom→vention_base_link` and publishes
`/odometry/fused_imu_wheel` (also Cartographer's + TEB's odometry topic). The ZED
publishes **no TF at all** (`pos_tracking/publish_tf=false`) — its VIO odom topic
lives on purely as a monitored witness. Nobody else publishes those edges (see §3).

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
 └─ odom                     ← ekf_fused_imu_wheel publishes odom→vention_base_link (20 Hz)
     └─ vention_base_link       (URDF root since 2026-07-13)
         ├─ lidar_r           ← static_transform_publisher
         ├─ lidar_l           ← static_transform_publisher
         ├─ vention_base_cam_1_link      ← URDF joint (nav D435)
         ├─ zed_mini_base_link → camera/imu frames  ← ZED's own description publisher
         └─ world → arm_base_link → …    ← Kinova arm chain (combined.urdf only)
```

Cartographer config: `tracking_frame="vention_base_link"`, `published_frame="odom"`,
`provide_odom_frame=false`, `publish_frame_projected_to_2d=true`
(`vention_2lidar_localization.lua:9-14`).

**Frame-ownership rules (hard-won 2026-07-13; violate = "two unconnected trees"):**
- The ZED **mount edge** (`vention_base_link → zed_mini_base_link`, +0.3176/0/+0.386)
  is owned by the ZED wrapper's own description publisher via the `base_frame` +
  `cam_pos_*` args on the `zedm.launch` include — deliberately NOT in
  `combined.urdf.xacro`. Two static publishers of the same child frame fight
  (per-listener last-writer-wins). The wrapper's default `base_frame:=base_link`
  roots the camera under an orphan frame; historically that orphan was masked by
  the ZED's dynamic `odom→base_link` TF re-asserting the parent 15×/s.
- The nodelet reads **`general/base_frame`** (a `pos_tracking/base_frame` param is
  silently ignored — a no-op override shipped for weeks). It stays
  `zed_mini_base_link` so the ZED odom topic's `child_frame_id` semantics are
  unchanged.
- Pre-cutover, the URDF held an INVERTED joint (`zed_mini_base_link` parent of
  `vention_base_link`) because the ZED owned odom. `vention.urdf` (legacy standalone
  launches only) still has that direction — never run those alongside
  `sensors.launch`.

### Static transforms (translation m; rpy rad; parent `vention_base_link`)
| Child | xyz | rpy | Publisher |
|---|---|---|---|
| `lidar_r` | (0.2575, −0.135, 0.415) | (π, 0, 0) | `static_transform_publisher` (`sensors.launch:47-49`) |
| `lidar_l` | (0.2575, +0.135, 0.415) | (π, 0, 0) | `static_transform_publisher` (`sensors.launch:50-51`) |
| `zed_mini_base_link` | (0.3176, 0, 0.386) | (0,0,0) | ZED description publisher (`sensors.launch` zedm include, `base_frame`+`cam_pos_*`) |
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

### Fused odometry — the authoritative source (`sensors.launch`, since 2026-07-13)
Three nodes, all in `sensors.launch` so they outlive `prefix+r` restarts:
- **`wheel_odom_publisher`** — encoder odometry over the NUC base RPC
  (`counts_per_meter=4874`, `track_width=0.85`); trustworthy `vx`, advisory yaw.
- **`gyro_bias_estimator`** — republishes the ZED IMU with the gyro bias subtracted
  (`/zed_mini/zed_node/imu/data_debiased` + diag `/gyro_bias_estimate` in deg/s).
  Bias is estimated only during PROVEN stillness (wheel vx ~0 AND gyro ~0 — the dual
  gate rejects in-place spins that fool a wheels-only check) with a running-mean →
  EMA. The raw bias is a **per-boot lottery** (−131 deg/h Jul 10; ~0 wandering to
  −25 deg/h within a session) — never hard-code it. Wait for the
  "first calibration complete" log (≥15 s stillness) before driving.
- **`ekf_fused_imu_wheel`** (robot_localization) — fuses wheel `vx`(+advisory `vyaw`)
  + debiased gyro `vyaw`; ZED VIO **excluded**. `publish_tf=true` → owns
  `odom→vention_base_link` at 20 Hz and publishes **`/odometry/fused_imu_wheel`**,
  consumed by Cartographer (`cartographer_localization.launch` `odom` remap) and TEB
  (`odom_topic`). Deliberately **no respawn**: an auto-restarted EKF resumes at the
  odom origin — a silent teleport into Cartographer; a dead EKF instead stalls the
  stack loudly (restart sensors, then `prefix+r`).

### ZED odom → sanitizer/differentiator — RETIRED [2026-07-15]
**The ZED runs IMU-only since 2026-07-15**: `depth/depth_mode=NONE` +
`pos_tracking_enabled=false` in `sensors.launch`, after 4 SDK-internal
segfaults since Jun 21 (tracking/depth internals, SDK 5.0.7; the Jul 15 one
killed a feeding run mid-drive). VIO odom, `odom_sanitized`, and
`/move_base/odom_feedback` no longer exist; `zed_pose_to_odom_feedback` was
removed from `sensors.launch` (script kept in git). The only live ZED product
is the IMU (~200 Hz) feeding `gyro_bias_estimator` → the EKF's `vyaw`.
The navigation dataset (color/depth for offline method comparison) is now a
per-bringup **SVO recording** (`zed_svo_recorder.py` →
`integration/log/svo/`), replayable through the SDK offline. Drift traces are
now carto / wheel / fused_imu_wheel only (see `drift_trace_compare.py`).

### Cartographer (localization mode)
Runs against a **frozen `.pbstream`** (`load_frozen_state=true`), publishing `map→odom`
**only at pose-graph optimizations**, which are counted in **nodes**, and node
insertion is motion-filter-gated. So corrections arrive as **discrete jumps**, not a
smooth stream — the robot's map-frame pose = `map→odom` (jumpy) ∘ `odom→base` (smooth
EKF, 20 Hz). With the fused odom the observed correction stream is ~5–8 cm / ~1.3°
during driving (session_20260713_152341). This is the structural reason heading
corrections can look like overshoot (§8).

### ZED health interlock — DELETED [2026-07-15]
`zed_health_monitor.py` (five detector channels asserting `/nav_safety_hold`)
was unplumbed from `navigation.launch` on 2026-07-13 and deleted from the tree
on 2026-07-15 with the IMU-only ZED sweep (it watched VIO odom/status, which no
longer exist). The `/nav_safety_hold` SUBSCRIBER hooks (cmd_vel bridge,
shared_autonomy_manager, navigate.py) remain as no-ops — a Phase-3 interlock
(e.g. the map→odom yank channel) can republish the topic and they re-engage.

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

Every exposed knob with its current value, location, and **what it does**. **★ = tune this for the
current symptoms.** "inherited" = not set in-repo, takes the upstream default (verify
against the installed package version).

### 7.1 Cartographer — `config/vention_2lidar_localization.lua`
| Param | Value | Line | What it does |
|---|---|---|---|
| ★ `POSE_GRAPH.optimize_every_n_nodes` | 25 | :62 | Runs the pose-graph optimization that recomputes `map→odom` every N inserted scan nodes — the *only* moment localization corrections are applied. Lower = more frequent corrections (counts **nodes**, not seconds; 90 in the mapping lua). |
| ★ `motion_filter.max_distance_meters` | 0.05 | :50 | A new scan node is inserted once the robot has moved this far — throttles node rate by distance (a node every 5 cm driving). |
| ★ `motion_filter.max_time_seconds` | 0.5 | :49 | …or once this much time passes, so nodes keep forming while parked (every 0.5 s stationary). |
| ★ `motion_filter.max_angle_radians` | rad(0.5) | :51 | …or once the robot has rotated this much; whichever of distance/time/angle trips first makes a node. |
| ★ `use_online_correlative_scan_matching` | **false** (inherited) | — | Coarse brute-force scan matcher that seeds Ceres so matching survives a poor odom prior. Off ⇒ Ceres leans on the odom prior alone — fragile at speed. |
| ★ `constraint_builder.fast_correlative_scan_matcher.linear_search_window` | **7.0 m** (inherited) | — | Half-width of the box the loop-closure/relocalization matcher searches around the prior. Larger tolerates bigger error but is slower and more alias-prone (the "7 m search radius"). |
| `max_range` / `missing_data_ray_length` | 12.0 / 12.0 | :41-42 | Farthest lidar return used / range at which a no-return ray is painted as free space (trimmed from 30). |
| `min_range` | 0.15 | :40 | Returns closer than this are discarded (self-hits / too near). |
| `ceres_scan_matcher.translation_weight` / `rotation_weight` | 10 / 40 | :53-54 | How hard the local Ceres match pulls toward the scan vs the odom prior, in translation / rotation (higher = trust the scan more). |
| `pure_localization_trimmer.max_submaps_to_keep` | 3 | :45-47 | In localization mode, keep only the N most recent submaps — bounds memory/compute (localization-only). |
| `use_imu_data` | false | :39 | Whether the 2D trajectory builder fuses IMU for orientation/gravity; off ⇒ lidar + odom only (Cartographer then has no gravity reference and cannot see pitch/roll). |
| `pose_publish_period_sec` | 0.01 (100 Hz) | :27 | How often the estimated pose / `map→odom` TF is published. |
| `submap_publish_period_sec` | 0.3 | :26 | How often submaps are published (visualization / constraint building). |
| `lookup_transform_timeout_sec` | 0.2 | :25 | How long Cartographer blocks waiting for a required TF before abandoning that lookup. |
| `global_constraint_search_after_n_seconds` | 10 (inherited) | — | Delay after startup before the first *global* (full-map) relocalization search runs. |
| `constraint_builder.min_score` / `global_localization_min_score` | 0.55 / 0.6 (inherited) | — | Match-score thresholds to accept a local / global loop-closure constraint; higher = stricter, fewer false matches. |
| `MAP_BUILDER.num_background_threads` | 4 (inherited) | — | Worker threads for pose-graph optimization and constraint building. |

### 7.2 ZED wrapper — `zed_wrapper` defaults (overridden bits in `sensors.launch`)
**[2026-07-15] IMU-only ZED**: `sensors.launch` now also overrides
`depth/depth_mode=NONE` (hard-disables depth AND blocks tracking from ever
starting) and `pos_tracking/pos_tracking_enabled=false`. Every `pos_tracking/*`
row below is therefore inert — kept as the safe config should VIO ever be
re-enabled. Raw stereo+IMU is recorded to SVO per bringup (`zed_svo_recorder.py`).

| Param | Value | What it does |
|---|---|---|
| ★ `pos_tracking/area_memory` | **true** (default) | Enables the ZED's spatial memory (loop closure / relocalization) — lets VIO snap back to remembered places, but each snap is a discrete pose jump (bug #1). |
| ★ `general/pub_frame_rate` | 15.0 | Rate the wrapper publishes images/odom at (may be below the grab rate); lowering it cuts USB/CPU load. |
| ★ `pos_tracking/two_d_mode` | false | Constrains VIO to SE(2), pinning z / pitch / roll to fixed values. On ⇒ no vertical or tilt drift — the right choice for a ground robot (commented-out in `base_sensors.launch:101-103`). |
| `general/grab_resolution` / `grab_frame_rate` | HD720 / 30 | Resolution / FPS the SDK captures from the camera sensor. |
| `depth/depth_mode` | ULTRA | Depth-computation quality preset; higher = more GPU/USB. Not needed for VIO/nav (consider PERFORMANCE for USB headroom). |
| `pos_tracking/imu_fusion` | true | Fuse the built-in IMU with visual tracking (true VIO) and derive the gravity/vertical reference from it. |
| `pos_tracking/publish_tf` / `publish_map_tf` | **false** / **false** | The ZED broadcasts NO TF since the 2026-07-13 cutover (`ekf_fused_imu_wheel` owns `odom→base`; Cartographer owns `map→odom`). Verified in wrapper source: one gated call site, no force-publish path; `sensors/publish_imu_tf` is independent and stays on. |
| `general/base_frame` | `zed_mini_base_link` | The frame the ZED tracks/publishes odom for. ⚠ This is the REAL param — `pos_tracking/base_frame` is silently ignored (was a no-op override for weeks). Must stay `zed_mini_base_link` (odom-topic child semantics + the wrapper's camera→base startup lookup). |

### 7.3 odom sanitizer — `config/nav/odom_pipeline.yaml` — RETIRED [2026-07-15]
The sanitizer node is out of `sensors.launch` (IMU-only ZED: no VIO odom to
sanitize); this catalog stays for reading pre-Jul-15 logs/bags.

| Param | Value | What it does |
|---|---|---|
| ★ `jump_accept_frames` | 5 | Consecutive self-consistent raw frames required before a *sustained* pose shift is accepted as a real relocalization instead of rejected as a teleport — the leak lever behind bug #2. |
| `enable_sanitizer` / `enable_windowed_diff` | true / true | Master switches: gate single-frame teleports before republishing / compute the TEB twist by windowed differencing. Off = original pass-through. |
| `max_lin_vel` / `max_ang_vel` | 0.5 / 1.5 | Teleport gate: a frame-to-frame pose jump implying speed above this (m/s / rad/s) is dropped and the last good pose is held. |
| `vel_diff_window` | 0.08 s | Time span over which the sanitized pose is differenced to make the velocity feedback; larger = smoother but laggier twist. |

### 7.4 TEB — `config/nav/teb_local_planner.yaml`
| Param | Value | Line | What it does |
|---|---|---|---|
| `odom_topic` | `/odometry/fused_imu_wheel` | :5 | Velocity feedback source — the fused wheel+IMU EKF since 2026-07-13 (was the ZED-derived `/move_base/odom_feedback`). |
| ★ `min_obstacle_dist` | 0.01 | :54 | Minimum distance the optimizer keeps the footprint from any obstacle. At 0.01 m (below one 0.05 m costmap cell) trajectories graze cells the feasibility check calls collision ⇒ grazing/infeasible (bug #5). |
| ★ `weight_viapoint` | 1.0 | :93 | How hard the trajectory is pulled onto the global plan's via-points; higher = hug the plan (and its jitter) more tightly (was 5.0). |
| ★ `weight_obstacle` | 100.0 | :89 | Penalty weight for approaching obstacles; higher = stronger standoff/avoidance. |
| ★ `max_global_plan_lookahead_dist` | 1.0 | :15 | How far along the global plan TEB optimizes at once; longer averages over plan/yaw jitter but costs more compute. |
| `max_vel_x` / `_backwards` | 0.075 / 0.075 | :25-26 | Max forward / reverse linear speed the optimizer commands. **Autonomy tier, tuned 2026-07-10 — see §12** (was 0.4). |
| `max_vel_theta` | 0.125 | :28 | Max angular (yaw) rate. Set to 5/3 × `max_vel_x` (matches the teleop ratio, §12); was 0.5. |
| `acc_lim_x` / `acc_lim_theta` | 0.7 / 1.5 | :29-30 | Linear / angular acceleration limits used to shape the trajectory (planner-model only — the actuator has no ramp). |
| `dt_ref` / `dt_hysteresis` | 0.2 / 0.1 | :10-11 | Target time spacing between trajectory poses / hysteresis before the elastic band is resized. |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.05 / 0.05 | :42-43 | Position / heading error at which the goal is declared reached. |
| `enable_homotopy_class_planning` | true | :101 | Explore multiple distinct routes (e.g. left vs right around an obstacle) in parallel and pick the best (`max_number_classes=2`). |
| `oscillation_recovery` (+ v/omega eps, durations) | true / 0.1 / 0.1 / 10 s | :126-130 | Detect back-and-forth non-progress (speed below the eps for the duration) and trigger a recovery. |
| `feasibility_check_no_poses` | 1 | :19 | How many poses along the candidate trajectory are collision-checked before it is accepted (2/4 in the mined runs). |

### 7.5 Costmaps — `config/nav/{costmap_common,global_costmap,local_costmap}.yaml`
| Param | Value | What it does |
|---|---|---|
| `footprint` | [[0.37,0.295],[0.37,−0.295],[−0.37,−0.295],[−0.37,0.295]] | Robot outline (polygon in the base frame) used for collision checking and inflation (0.74×0.59 m). |
| ★ `inflation_layer.inflation_radius` | **0.05 in common, overridden to 0.5** by local/global | Distance obstacles are grown with a decaying cost cushion so the planner keeps clear of them (the 0.05 in common is dead config — bug #13). |
| `inflation_layer.cost_scaling_factor` | 5.0 | Exponential rate at which the inflated cost decays with distance; higher = cost falls off faster (thinner effective cushion). |
| `obstacle_range` / `raytrace_range` | 8.0 / 10.0 | Max sensor range at which to *mark* obstacles / to *clear* free space along a ray. |
| `local_costmap` size / res / update / publish | 8×8 m / 0.05 / 10 Hz / 5 Hz | Rolling-window costmap extent, cell size, and refresh / publish rates (`odom` frame). |
| `global_costmap` update / publish | 2 Hz / 1 Hz | Refresh / publish rates of the static map-frame costmap. |
| `transform_tolerance` | 0.5 | How stale a TF may be before the costmap treats it as invalid and stops updating. |

### 7.6 cmd_vel bridge — `launch/navigation.launch:46-51`
| Param | Value | What it does |
|---|---|---|
| ★ `min_move_units` | 100 | Ratio-preserving **per-wheel** stiction floor: commands below it are scaled up so slow commands still move. At the calibrated scales this floors motion to **0.0205 m/s / 0.0385 rad/s** (§12). Breakaway sweep shows reliable motion to ~40 counts, so 100 keeps margin + fine control (was 250 ⇒ the old bang-bang, bug #4, resolved). |
| `linear_scale` / `angular_scale` | 4874 / 2600 | m/s / rad/s → motor counts. **Physically calibrated (§12):** `linear_scale` = `counts_per_meter` so commanded m/s == actual; `angular_scale` from measured rotation (was 800 / 600 — ~6× / ~4× low). |
| `w_deadband` | 0.03 (default) | Angular commands whose magnitude is below this are zeroed, to avoid sign-flip jitter near zero. |
| `max_speed_units` | 2500 | Per-wheel command clamp (hardware/safety ceiling). |

### 7.7 move_base — `launch/navigation.launch:21-28`
| Param | Value | What it does |
|---|---|---|
| ★ `planner_frequency` | 1 | How often the global planner replans, in Hz (0 = only on a new goal). Each replan re-expresses the plan's orientation from the current estimate, re-injecting Cartographer yaw jitter (bug #3; comment intends 0.2). |
| `controller_frequency` | 10 | How often the local planner (TEB) runs and publishes `/cmd_vel`, in Hz. |
| `planner_patience` / `controller_patience` | 15 / 15 | How long move_base waits for a valid global plan / valid control before invoking recovery behaviors (s). |

### 7.8 zed_health_monitor — DELETED [2026-07-15]
Script + launch params removed with the IMU-only ZED sweep (unplumbed from
`navigation.launch` 2026-07-13). Catalog kept for reading pre-Jul-15 logs.

| Param | Value | What it does |
|---|---|---|
| `max_lin_vel` / `max_ang_vel` | 0.5 / 1.5 | Jump channel: raw ZED odom implying a speed above this (m/s / rad/s) is flagged as VIO divergence and holds the base. |
| `recovery_stable_s` | 2.0 | How long tracking must be continuously healthy again before a hold is released. |
| `enable_yank_channel` | true | Enable the `map→odom` "yank" detector for Cartographer relocalization/alias jumps. |
| `yank_lin_m` / `yank_ang_rad` | 0.5 / 0.4 | A `map→odom` step larger than this (m / rad) counts as a yank ⇒ hold the base + clear move_base costmaps. |
| `yank_settle_s` | 5.0 | Settle time after a yank before release is considered (pose-graph snaps ping-pong). |
| `release_mo_rate_rad_s` | 0.03 | Don't release while `map→odom` yaw is still moving faster than this (rad/s). |
| `max_hold_s` | 45.0 | Escape hatch: maximum hold duration before release is forced. |

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

**Priority (updated 2026-07-13):** Phase 2 (odometry) is **DONE** — resolved by
replacing ZED VIO with the fused wheel+IMU EKF (see Phase 2 RESULTS). The dominant
open work moves up the stack: Phase 3 (cartographer over the new trustworthy prior,
speed-ceiling re-test) then 4–5. Because the mined logs predate the current config,
any fresh comparison still starts from a **fresh baseline**.

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

### Phase 1 — RESULTS (2026-07-09)
*Direct-serial characterization on the NUC (base_server stopped, exclusive port), compass as ground truth. Tools: `base_cmd_path.py` (rotate_left/right/straight/reverse/arc); NUC-side `/tmp/{breakaway_step,straight_step,ab_step}.py`.*

**Root cause of the intermittent 20–30° rotation overshoot: v7 firmware command-mangling.** The Arduino v7 firmware reads encoders over SoftwareSerial with interrupts masked, dropping bytes of inbound `A= B=` commands — **~5/s under active commanding** (the counter reached 121 in a ~2.5 min run; the base pane throttles WARN logs to 1 per 2 s, so counting log *lines* undercounts — read `unparseable total=`). A mangled **STOP** let the base coast to the firmware `CMD_STALE_MS=5000` watchdog → up to ~5 s × ~5.5°/s ≈ **27° overshoot** (the "20–30°"); a mangled **START** = commanded but no motion.
- **Fix applied (host-side, no reflash):** persistent echo-confirm retry in `vention_arduino_control.py` — a ~50 Hz background keepalive re-sends the current setpoint until the firmware echoes `Parsed A=…`. Deployed to the NUC and **validated under load** (`session_202009_phase_1_scond`): the retry fired 4× on real mangled starts/stops, each resolved in <~1 s, and the mangle rate stayed ~5/s (no vicious-cycle hammer). Missed-stop coast is now ~0.1 s ≈ 0.5° instead of 27°. **This was the primary overshoot fix**, not the floor/scale tuning below.
- **Band-aid, not a cure.** The ~5/s mangling is firmware-level and persists. Durable fix deferred (not needed yet): raise `ENC_POLL_MAX_MS` (300 → ≤500 ms; keep it **<1 s** or it defeats the retry's `encoders_fresh(1.0)` gate), or move to hardware UART / a checksum+ACK protocol. (memory: `v7-encoder-streaming-mangles-commands`)

**Kinematics — revises the plan's assumptions:**
- **No stiction floor exists down to 40 counts/s.** The RoboClaw velocity loop turns the wheels at every commanded speed 250→40; the base rotated reliably throughout. The plan's "min executable ω ≈ 0.417 rad/s" was a **config artifact** (`min_move_units=250 / angular_scale=600`), not a physical limit.
- **Slip is motion-dependent:** straight ≈ 0%, gentle arc ≈ 0%, in-place rotation ≈ **20%**. **In-place rotation is the worst case; arcs slip *less*** (little lateral scrub) — the drivetrain traces blended arcs faithfully and smoothly. (Corrects the earlier worry that blends slip more.)
- **Scale calibration (ground-truth measured):**
  - `counts_per_meter ≈ 4874` **confirmed** (17 cm actual vs 16.3 cm encoder, straight).
  - `linear_scale` should be **≈ 4874** (= counts_per_meter) but is **800 → ~6× low** (base moves ~16% of commanded m/s; deliberate "slow for localization").
  - `angular_scale` should be **≈ 2600** (250 counts/s → 0.096 rad/s actual) but is **600 → ~4× low**.
  - **Arc over-turn:** physical `linear:angular` ≈ 4874:2600 ≈ **1.87** vs configured 800:600 = **1.33** → blended arcs **over-turn ~40%** (tighter than TEB plans → wiggle on curves). A config-ratio bug, independent of slip.

**Recommended `navigation.launch` changes — NOT yet applied:**
1. `min_move_units`: 250 → **100** (finer near-goal; secondary now the retry killed the big overshoot).
2. `linear_scale`: 800 → **~4874**.
3. `angular_scale`: 600 → **~2600**.
- **Mandatory coupling:** these set the gain from TEB command → actual motion; raising them 4–6× means re-setting TEB `max_vel_x` / `max_vel_theta` / `acc_lim_*` to the actual slow speeds wanted (~0.15 m/s, ~0.3 rad/s) **in the same edit**, or rotations/arcs turn aggressive and overshoot returns. Speed policy belongs in TEB limits, not in detuned scales. (memory: `base-command-calibration-jul9`)

**Operational note:** "base won't move + `/cmd_vel_teleop` has traffic" in a manual bringup = the `cmd_vel_bridge` node isn't running (it lives in `navigation.launch`; a manual bringup that starts move_base alone omits it). Signature: base_server sends only `0,0`, `cmd_latency.csv` empty, no bridge in the rosmaster `+SUB` list — NOT the mangling/retry. (memory: `cmd-vel-bridge-missing-in-manual-bringup`)

**Status:** overshoot root-caused and mitigated (retry deployed + validated under load); kinematics characterized; three param changes + a TEB-limit re-check queued but not applied.

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

### Phase 2 — RESULTS (2026-07-10 → 2026-07-13) — **DONE, resolved by replacement**
*The plan above assumed "fix the ZED". The measured answer was stronger: replace ZED
VIO as the odometry source entirely. Tools built: `fused_odom_observer.launch
ablate:=true` (parallel observe-only EKF ablation variants), `zupt_publisher.py`,
`gyro_bias_estimator.py`, `plot_drift_bag.py`; five→eleven map-anchored traces.*

**Failure characterization (why VIO lost):**
- ZED VIO's dominant real-world failure is **slow confident creep** (<0.5 m/s), not
  just teleports: 17 m (jul10-6), 5.9 m (jul13 runs) divergences with the sanitizer
  passing them **byte-identical** — the 0.5 m/s gate never sees them (bug #2 is
  unfixable at that layer for this mode). Sunlight/exposure is the usual trigger.
- Ablation verdict (jul10-6 run, carto reference invalid — both it and ZED blew up):
  the **Mahalanobis reject-gate is the anti-jump hero** (survivors: gate/improved/
  no-VIO at ~1 m vs dragged variants at 17 m) but the gate does NOT catch creep
  (jul13: gate dragged 2.1 m by a creep while no-VIO variants sat at 0.26 m). ZUPT
  and gyro-yaw alone don't protect against translational error at all.
- Loop-closure benchmark (jul10-6, return-to-start): open-loop wheel+IMU closed
  16.3 m / 14 min / 1100° of turning to **0.57 m / −34°**, and counterfactual replay
  showed −31° of that yaw was ONE number: the **uncorrected ZED gyro z bias**
  (measured −0.036°/s ≈ −131°/h, stable within a run, wandering across it, different
  every boot). Wheel-only yaw scrubbed −49°; ZED VIO 17 m.

**The fix (two stages):**
1. **`gyro_bias_estimator.py`** — stillness-calibrated debiasing (dual wheel+gyro
   still gate; running-mean→EMA, τ=30 s; calibrates in ~11 s of stillness; tracks
   thermal wander). Replay-validated on recorded runs: −35° → −3.6° yaw. Field
   A/B (jul13 pre-cutover): gold (debiased) 0.64 m / −5.5° vs teal (raw gyro)
   0.75 m / −10.1° over 28 m — on a lucky low-bias boot; the fix makes boot luck
   irrelevant.
2. **Stage-2 cutover (2026-07-13):** `ekf_fused_imu_wheel` (wheel vx + debiased gyro
   vyaw, no VIO) owns `odom→vention_base_link` + feeds Cartographer and TEB
   (`/odometry/fused_imu_wheel`). ZED TF fully off; URDF un-inverted; ZED mount edge
   owned by the wrapper's description publisher (see §3 frame-ownership rules — the
   orphan-`base_link` two-trees trap and the `general/base_frame` no-op are documented
   there). Rollback = git revert.

**Cutover validation (session_20260713_152341, 168 s / 11.6 m / 768° drive):**
- **ZED VIO diverged 5.9 m (creep) during the run → localization did not care.**
  Carto's corrections stayed at the 5–8 cm / ~1.3° routine level while driving; zero
  discontinuities; the run before the cutover with the same class of ZED failure
  mislocalized carto by 4 m / 78°.
- All open-loop traces scored against carto (position error at drive end / final yaw
  error / RMS / max over the drive):

| Trace | End err | End yaw err | RMS | Max |
|---|---|---|---|---|
| **fused_imu_wheel (live odom, gold)** | **0.26 m** | **−3.1°** | **0.17 m** | 0.28 m |
| no-VIO raw gyro (teal) | 0.26 m | −3.7° | 0.17 m | 0.29 m |
| wheel only (blue) | 0.30 m | +2.3° | 0.16 m | 0.31 m |
| +gyro-yaw | 0.32 m | −3.6° | 0.39 m | 0.75 m |
| improved (gate+ZUPT+gyro) | 0.47 m | −3.8° | 0.32 m | 0.49 m |
| stock fused (sanitized ZED + wheel) | 1.56 m | +22.1° | 0.69 m | 1.62 m |
| +ZUPT | 1.84 m | +28.6° | 0.79 m | 1.91 m |
| +reject-gate | 2.06 m | +27.1° | 1.10 m | 2.20 m |
| ZED raw (red) | 5.87 m | +16.5° | 4.23 m | 6.42 m |
| ZED sanitized (orange) | 5.87 m | +16.5° | 4.32 m | 6.42 m |

  Readings: every variant still fusing ZED velocity (stock/ZUPT/gate) was dragged
  1.5–2.2 m by the creep — the Mahalanobis gate is jump-armor, not creep-armor;
  `improved` survived only because its gyro+ZUPT sides dominated. The three
  ZED-velocity-free rows cluster at ~0.3 m. Raw gyro ≈ debiased this boot (low-bias
  lottery draw); the estimator exists for the −131 deg/h days. Wheel-only yaw did
  well here (gentle arcs, ~770° cumulative) — do not extrapolate; it scrubbed −49°
  on the jul10 spin-heavy loop.
- Authoritative odom vs carto: **RMS 0.17 m, end 0.26 m / −3.1°** (that difference
  IS the total map→odom correction — i.e., odometry good enough that Cartographer
  barely works).
- Trace-reading note: gold (fused_imu_wheel) is now carto's own odom component —
  gold-vs-green shows pure map→odom corrections; red/orange remain the independent
  ZED witnesses.

**Metric verdict:** the Phase-2 metric ("zero leaked sanitized steps > 0.3 m") was
made moot — leaked ZED steps no longer reach anything. Effective new metric, met:
localization unaffected by a multi-meter VIO divergence.

**Residuals / deferred:** per-run gyro-bias wander (~±0.01°/s) leaves a ~2–5° open-
loop yaw floor (fine under carto correction); wheel translation floor ~0.3–0.4 m per
15 min from spin scrub (unobservable to wheels+gyro; a healthy gated VIO could see
it — not currently worth the complexity); nothing yet monitors the NEW chain
(wheel-odom silence / EKF death) — Phase 3 item. ZED hardware stays for its IMU
(the only gyro on the robot) + traces; its depth is idle (future obstacle layer?).
(memories: `phase2-odom-fusion-ablation`)

### Phase 3 — Localization / Cartographer
Tool: `measure_localization.py` (static + moving).
- Measure map→odom jitter, first-fix time, and **max speed before divergence**.
- *Slow startup:* keep `optimize_every_n_nodes` low but front-load a global constraint;
  verify pbstream size; performance governor (Phase 0).
- *Can't-keep-up-at-speed:* enable `use_online_correlative_scan_matching=true`, relax
  `motion_filter`, revisit the 7 m search window.
- **Metric:** first fix < N s; map→base jitter within goal tolerance at target speed.

**Updated starting points post-Phase-2 (2026-07-13):** the odom prior is now smooth,
teleport-free, and ~0.2 m RMS over a loop — every historical carto symptom should be
re-measured before tuning, since most were ZED-caused:
1. **Speed ceiling re-test first.** "Carto can't keep up at speed" was measured over
   a teleporting ZED; the cautious tiers (TEB 0.075 / teleop 0.10 m/s) may be far
   below the real ceiling now. Walk speeds up on a fixed loop and find the true
   divergence point before touching lua params.
2. **Raise carto's odometry trust.** The lua odometry weights were implicitly tuned
   against ZED behavior; with a trustworthy prior, higher odometry weight directly
   attacks the **180° table alias** (the alias must now fight a confident prior)
   and featureless-corridor drift.
3. **Guard the new chain** (the one unmonitored failure mode): a wheel-odom-silence
   / EKF-death channel in the health monitor (mid-run RPC loss = EKF holds last vx
   and glides smoothly; only the yank channel would notice, late). Also re-scope the
   ZED channels: VIO faults no longer corrupt localization, so status/jump/silence
   holds are now advisory-grade — consider demoting them from base-stopping to
   logging, keeping yank (carto self-consistency) as the hard channel.
4. **Tool label debt:** `measure_localization.py` labels `odom→base` "ZED VIO
   odometry"; `nav_diag_logger` tf.csv `ob_age_s` now measures the EKF edge (~20 Hz,
   not ZED ~15 Hz); `navigate.py:187` comment says "odom→base from VIO". Update
   before trusting their printed interpretations.
5. **Alias tripwire (cheap, optional):** the once-rejected magnetometer idea has one
   honest job — a coarse heading consistency check to break the 180° alias
   (±40° accuracy suffices); survey field sanity near the table first.

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

---

## 12. Base command calibration & measured kinematics

*Direct-serial characterization on the NUC (base_server stopped, exclusive port), compass as ground truth, 2026-07-09/10. Authoritative record of the tuned numbers so they can be revisited/reverted. Tools: `scripts/base_cmd_path.py` + NUC-side `/tmp/{breakaway_step,straight_step,ab_step}.py`.*

### 12.1 Measured constants
| Const | Value | Meaning | How derived |
|---|---|---|---|
| `linear_scale` | 4874 | v [m/s] → wheel counts | set = `counts_per_meter` so commanded m/s == actual |
| `angular_scale` | 2600 | w [rad/s] → wheel counts | measured rotation: 250 counts/s → 0.096 rad/s |
| `counts_per_meter` | 4874 | odom counts → m | **confirmed**: 300 counts/s ×3 s = 16.3 cm enc vs 17 cm measured |
| `track_width_m` | 0.85 | odom counts → yaw | nominal; effective skid track is wider (rotation slips ~20%) |
| `min_move_units` | 100 | per-wheel stiction floor (counts) | breakaway sweep: reliable to ~40 counts ⇒ 100 = margin + fine control |
| `max_speed_units` | 2500 | per-wheel output clamp | safety ceiling (~0.51 m/s/wheel) |
| `w_deadband` | 0.03 rad/s | **angular anti-sign-flip ("non-jerk") threshold** | \|w\|<0.03 ⇒ go straight, so near-zero noise can't wiggle the heading. **No linear deadband** — slow forward creep is wanted for docking. |

### 12.2 Minimum executable speeds (the floor)
From `min_move_units=100` and the scales:
- **Translational: 0.0205 m/s** (100/4874, ~2 cm/s). Smaller commands snap *up* to this; never zeroed.
- **Rotational: 0.0385 rad/s ≈ 2.2°/s** (100/2600). Plus the 0.03 rad/s deadband: \|w\|<0.03 → 0; 0.03–0.0385 → snapped to 0.0385; above → as commanded.

### 12.3 Slip by motion type
Straight ≈ 0%, gentle arc ≈ 0%, **in-place rotation ≈ 20%**. In-place rotation is worst-case (max lateral scrub); arcs slip less. The drivetrain traces arcs faithfully, so arc mis-tracking is a config-ratio issue (scale ratio), not slip.

### 12.4 Tuned speed set (cautious tier, 2026-07-10)
Both slow; autonomy < teleop; 5/3 angular:linear ratio; floor kept at 100 (20–31% of these maxes ⇒ smooth control without touching the floor).

| | linear | angular | where |
|---|---|---|---|
| Teleop (Xbox / webapp / drift) | 0.10 m/s | 0.1667 rad/s | `shared_autonomy.launch`, `webapp/.../navigation_teleop.vue`, `zed_drift_test.launch` |
| Teleop (direct-serial, counts) | 487 | 433 | `teleop_keyboard_direct.py`, `vention_teleop_{keyboard,controller}.py` |
| Autonomy (TEB) | 0.075 m/s | 0.125 rad/s | `config/nav/teb_local_planner.yaml` |

Autonomy is conservative: only ~0.066 m/s is proven reliable for localization; faster is untested until Phase 2/3.

### 12.5 Raw tuning-run data
**Breakaway sweep (rotate-left, compass truth, slip-prone patch) → `min_move_units`:**
| cmd (counts/s) | dur (s) | actual° | actual °/s |
|---|---|---|---|
| 250 | 2 | 11 | 5.5 |
| 200 | 2 | 7 | 3.5 |
| 150 | 2 | 6 | 3.0 |
| 120 | 2 | 6 | 3.0 |
| 100 | 3 | 7 | 2.3 |
| 80 | 3 | 5 | 1.7 |
| 60 | 4 | 5 | 1.25 |
| 40 | 5 | 4 | 0.8 |

⇒ no stiction stall down to 40 counts (the RoboClaw velocity loop keeps the wheels turning); ~20% slip vs encoders. So 250 was ~3–6× higher than needed.

**Straight-line → `counts_per_meter` / `linear_scale`:** 300 counts/s × 3 s → encoder 16.3 cm, measured **17 cm** (≈0% slip) ⇒ `counts_per_meter` ≈ 4874 confirmed; 300 counts/s = 0.057 m/s actual ⇒ `linear_scale` ≈ 4874.

**Arc (right 300 / left 150 × 5 s) → arc fidelity:** encoder 19.5 cm / 5.4°; measured **18 cm / 6°** (compass 114→108). Arc slip ≈ 0%, smooth even with the slow wheel at 150 counts.

**Rotation → `angular_scale`:** 250 counts/s → 5.5°/s = 0.096 rad/s ⇒ `angular_scale` ≈ 250 / 0.096 ≈ 2600.

### 12.6 Reverting / re-tuning
- **Faster:** raise the teleop / TEB `max_vel_*` (and the direct-serial count defaults). The scales are physical, so `max_vel_x` values are true m/s — pick the actual speed you want.
- **Min speed / granularity:** adjust the single `min_move_units` — do **not** split it per-axis (it's one per-wheel stiction limit). Don't go below ~40–60 counts (breakaway / jerk).
- `angular_scale=2600` was measured on a slip-prone patch; re-verify on the deployment floor (less slip ⇒ the effective value is lower).
