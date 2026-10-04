# Autonomous microwave button press — runbook (ROS 2)

Detect the panel, go to a stored spot in front of the button, press along the panel normal,
come back. Runs on rchi-cpu-5 (Comfee microwave, D435i wrist camera).
What the code does: `src/feeding_deployment/button_press/README.md`.

## Where the code lives

| What | Where |
|---|---|
| **Bring-up (everything below the driver, no motion)** | `scripts/button_press/bringup.sh` |
| **Driver** | `src/feeding_deployment/button_press/press_button.py` |
| Button finder (5-dome layout) | `src/feeding_deployment/button_press/dome_pattern.py` |
| Panel frame / plane fit + rays (pure, tested) | `button_press/{panel_frame,geometry}.py` |
| ROS 2 inputs (camera, tf) / arm IK + motion gates | `button_press/perception.py` / `button_press/arm.py` |
| Measure the pre-press spot / live detection viewer | `scripts/button_press/{measure_prepress_offset,view_detection}.py` |
| Tests | `tests/test_button_press_{dome_pattern,geometry,panel_frame}.py` |

**Not yet integrated:** `PressMicrowaveButtonHLA` (`actions/press_microwave_button.py`) still
runs the older open-loop pre-press/press pose sequence.

**Someone stands at the physical e-stop for every `--execute`.** Killing `bulldog_bypass.py`
e-stops the arm within ~1 s; Ctrl-C in the driver stops the *next* move. `arm_halt.py` is
unverified on hardware.

---

## 1. Bring-up (no motion)

```bash
./scripts/button_press/bringup.sh            # start (skips anything already running), then health check
./scripts/button_press/bringup.sh status     # health check only
source ~/press_logs/bringup/env.sh           # same env in your own terminal
```

It starts, in order: `arm_server` → `joint_state_bridge` (within 10 s — it is the Kortex
keepalive) → stub base → `bulldog_bypass` (**motion unlocked from here**) → speed `low` →
`robot_state_publisher` → hand-eye calibration tf → RealSense (aligned depth, IMU off).
PIDs and logs: `~/press_logs/bringup/<name>.{pid,log}`. Override the calibration file with `CALIB`.

The health check wants `ARMSTATE_SERVOING_READY`, speed `low`, gripper **closed (> 0.7)** —
the press uses the closed fingertips — `/joint_states` ~50 Hz, camera ~15 Hz, and the tf
`arm_base_link → camera_color_optical_frame`. Watch detection with
`python3 -u scripts/button_press/view_detection.py --target timer_clock`.

`INVALID_USER_SESSION_ACCESS` / `ARMSTATE_IN_FAULT`: teleop, an e-stop, or a new Kortex
session took the arm. `bringup.sh stop`, then `bringup.sh` again.

## 2. The press

Start with the panel in view, roughly facing it, **the camera at least 22 cm away** (the dome
detector refuses anything closer) — ~25–40 cm is the tested range.

```bash
python3 -u -m feeding_deployment.button_press.press_button --target timer_clock            # dry run: plans + gates every leg
python3 -u -m feeding_deployment.button_press.press_button --target timer_clock --execute  # does it
```

- `--target` is any dome name: `start_30s` (default — actually runs the microwave),
  `timer_clock`, `power_level`, `wgt_time_defrost`, `stop_eco`.
- `--presses N` (0 = go and come back), `--press-in` (default 0.010, max 0.03), `--no-return`.
  Every leg is one smooth Cartesian trajectory. Speed preset `low` or `medium`.
- The spot is the constant `PREPRESS_EE_OFFSET_M` / `PREPRESS_EE_QUAT_PANEL` in
  `press_button.py`, relative to the detected button. Re-measure it when the fingers or the
  camera mount change: `python3 -u scripts/button_press/measure_prepress_offset.py --target timer_clock`
  with the camera ~25 cm from the panel. That script makes no motion: you hand-guide the arm,
  then paste the printed values.
- From further than 25 cm the arm first stages 8 cm out from the pre-press spot (camera ~25 cm)
  and re-measures there. Far 3D estimates read ~1.5 cm low (depth/hand-eye bias, seen
  2026-10-03), which is why the final measurement, and the offset measurement, happen at
  ~25 cm. A dry run from far away plans only up to the staging pose.
- The whole sequence is pre-checked in the sim before the first move. On an abort
  mid-approach it goes straight back to the start; an aborted press backs itself out and
  holds. Per-run JSONL lands in `~/press_logs/` (`--log-dir` to change), the start joints in
  `~/press_logs/start_joints.json`. To get back after a hold: `scripts/session/goto_preset.py`.

## 3. Shutdown

Re-check `gripper_pos` and `get_arm_state()` (idle, `SERVOING_READY`), then
`./scripts/button_press/bringup.sh stop` (stops everything it started, in reverse; the arm
e-stops ~1 s after the bypass goes, which is expected).

## Gotchas

- `pkill -f <name>` from a wrapper shell can kill the shell itself; use `bringup.sh stop` or
  the pidfiles in `~/press_logs/bringup/`.
- `PYTHONPATH` must be *prepended* (`$PWD/src:$PYTHONPATH`): `PYTHONPATH=src` alone drops
  rclpy, and without `src` first the editable install (no `button_press`) wins.
- Glare can hide a dome; the detector then aborts rather than guess. Check `view_detection.py`.
