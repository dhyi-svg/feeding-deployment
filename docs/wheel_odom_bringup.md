# Wheel Odometry Bring-Up (Vention base)

The base's goBILDA 5303 motors have integrated encoders that the RoboClaws
already use for closed-loop `SpeedM1/M2` velocity control ("speed units" ==
encoder counts/sec). Firmware **v7** adds the read-back path: the Arduino polls
both RoboClaws (`ReadEncoders`) at ~10 Hz and streams
`E <millis> <a1> <a2> <b1> <b2> <okA> <okB>` lines over USB. The NUC host
caches the latest snapshot (reader thread in `vention_arduino_control.py`),
serves it over RPC (`BaseInterface.get_encoders()`), and
`wheel_odom_publisher.py` on the compute box publishes `nav_msgs/Odometry` on
`/wheel_odom` (topic only — **no TF**; the ZED owns `odom→…`, Cartographer owns
`map→odom`).

Expected numbers: 28 counts/motor-rev x 71.2:1 = **1993.6 counts/wheel-rev**;
96 mm wheel (0.30159 m/rev), miter gears 1:1 → **~6610 counts/m**.

## Status (2026-07-07, Claude)

Already done — off-robot + serial-side (motor power was OFF throughout):

- v7 flashed and byte-verified on the base Arduino (`flash_optiboot.py`, 70 pages),
  **re-flashed after a 7-finding adversarial review** (all fixes below folded in).
- Banner `Ready v7 enc` confirmed; E-lines streaming (~8 Hz motors-off rate —
  will be ~10 Hz powered); dead-controller 1 Hz backoff confirmed working.
- Final-stop delivery via echo-confirm PASS (the safety-relevant gate).
- Rollback: pristine v6 hex at NUC `~/wheel_odom_fw/v6/`, v7 at `.../v7/`.
- Host/RPC/node code written; committed + **NUC clone synced** — but base_server
  was left STOPPED; the new host code loads on your next `launch_base`.

Review fixes applied (all 7 confirmed defects):

1. **Poll starvation (blocker)**: a continuous >~15 Hz changing-command stream
   kept the firmware loop permanently in the send branch, so encoder polling
   (and, downstream, host echo-confirm) would go silent during motion. Added a
   `ENC_POLL_MAX_MS=300` hard ceiling that forces a poll regardless of send/RX
   activity → ≥3 Hz E-lines guaranteed even under full command load.
2. **Backoff deferred a new stop (safety)**: a controller that executes but
   can't ACK got backed off, delaying a *new* stop up to 1 s. A changed
   setpoint now clears the send backoff for its first attempt.
3. **RoboClaw power-cycle count reset (correctness)**: motor-power loss zeros
   the RoboClaws' volatile counts without rebooting the Arduino → phantom
   multi-meter jump on recovery. Publisher now re-baselines after any ok=0 gap
   and rejects implausible per-tick speeds (`max_plausible_speed_mps=2.0`).
4. **Torn setpoint pair**: `(a,b)` mutation moved inside `_send_lock` so a
   racing stop can't produce a half-and-half pivot.
5. E-line stamp taken *after* the reads (not before) so retry latency doesn't
   corrupt the twist.
6. Bench creep/teardown stop now uses the bounded echo-confirm re-send.
7. `/wheel_odom` header stamped at measurement time (`now - age_s`), not receipt.

**Note on the motors-off stream-test number**: with motor power OFF the
command-mangling gate reads high (~68%), because fix 2's changed-setpoint
bypass re-enables dead-controller retry storms every line. This is a bench
artifact — powered controllers ack immediately and don't storm. Rerun the gate
with motor power ON for the real (low) figure; the final-delivery PASS is valid
either way.

Powered validation + calibration DONE (2026-07-08, driven over ssh):

- Powered bench: E-rate 8.8 Hz, `ok`=1.00 both sides, **0% command mangling**
  (the motors-off 68% was purely the dead-controller retry artifact), final
  stop delivery PASS.
- Creep sign test: all four motors count positive → `side_a_sign=side_b_sign=+1`
  (node defaults), rate ≈ commanded → units are counts/sec.
- **`counts_per_meter` = 4874** — from an 8271-count drive tape-measured at
  1.697 m (26% below the 6610 parts-list estimate; the BOM over-estimated
  encoder/gear resolution or under-estimated effective rolling diameter).
- **`track_width_m` = 0.85** — refined from three in-place spins. A 192° spin
  implied 0.766, but two clean near-full ~325° spins both implied ~0.85 (wheels
  repeatable to 0.05%, physical angle to 3°). Effective track grows with turn
  size (scrub), so no single value is exact; 0.85 fits substantial turns. The
  ~35° full-turn shortfall is real under-rotation, not compass error (a fixed
  bias cancels over a full turn since start==end heading). Baked as node default.
- Spin direction (CCW → +yaw) confirms the A=right/B=left mapping and that
  wheel-odom yaw matches the ZED/Cartographer (REP-103) convention.

Remaining — needs motors on (marked ⚑): only the live `/wheel_odom` node check
after a stack restart. x-forward and yaw signs are set to REP-103 from the
compass test but worth a glance against ZED on the first live run (a
world-frame cross-check the standalone calibration couldn't do).

## Key design facts (why things look the way they do)

- **v6's silent pathology, now visible + fixed**: with unpowered RoboClaws the
  old loop retried failed sends every iteration (~400 ms of SoftwareSerial
  storms), which would have starved encoder polls and eaten inbound commands.
  v7 backs off send retries (1 Hz after 3 consecutive failures) and encoder
  reads (1 Hz after 5) per side; healthy controllers see zero change.
- **Some command mangling during encoder polls is by design**: SoftwareSerial
  RX masks interrupts ~1 ms/byte, and the Uno's USART FIFO is ~3 bytes. The
  poll only starts when the host line is quiet (`Serial.available()==0`), and
  the NUC host re-sends any command whose `Parsed A=.. B=..` echo doesn't
  appear within 0.1 s (echo-confirm), with the same-value refresh period
  tightened 1.0 s → 0.2 s as backstop. A future clean fix is raising the
  RoboClaw packet-serial baud 9600 → 38400 in Motion Studio (masking per byte
  then fits inside the USART FIFO), but that touches a working controller
  config — only bother if echo-confirm warnings turn out noisy in practice.
- **Arduino reset safety**: DTR resets (any serial reopen, incl. base_server
  restart) zero counts+millis. The bridge counts resets; the publisher
  re-baselines and never integrates across one — `/wheel_odom` must NOT
  teleport when you bounce base_server.

## ⚑ 1. Powered bench check (motor power ON, robot idle)

base_server must be STOPPED (it holds the port). Robot on the floor with
~10 cm clearance, or wheels lifted.

```bash
# on the NUC
~/miniconda3/envs/feeding_deployment/bin/python ~/wheel_odom_fw/bench_test_encoders.py \
    --watch 10 --stream-test 30
```

PASS looks like: E-rate ~10 Hz, `ok_a`/`ok_b` ratios 1.00, watch-window count
deltas ~0 (PID holds zero), final-delivery PASS. Nudge a wheel gently during
the watch to see counts move (don't fight the PID hard — it actively holds
zero through a 71.2:1 gearbox).

## ⚑ 2. Creep sign test (robot MOVES ~6 cm forward)

```bash
~/miniconda3/envs/feeding_deployment/bin/python ~/wheel_odom_fw/bench_test_encoders.py \
    --watch 0 --creep            # prompts for YES; A=200 B=200 for 2 s (~3 cm/s)
```

PASS: all four count deltas **positive** (confirms `side_a_sign=+1`,
`side_b_sign=+1` — expected, since opposed encoder polarity would make the
velocity PID run away) and rates ≈ 200/s (confirms units are counts/sec).
If any motor reads negative, set the matching `~side_a_sign`/`~side_b_sign`
param on `wheel_odom_publisher` instead of touching firmware.

## ⚑ 3. Restart the stack with the new host code

Normal `feeding-nuc.sh` order (arm, base, then bulldog). The NUC clone is
already synced; `launch_base` picks up the reader thread + echo-confirm
automatically. Then drive a little with teleop: responsiveness must be
unchanged, and any `echo missing ... re-sending` warnings in the base pane
should be rare (they are the repair loop working, not an error — frequent ones
mean the mangling budget is worse than expected: report it).

Rollback (if anything is off): stop base pane, then

```bash
~/miniconda3/envs/feeding_deployment/bin/python ~/wheel_odom_fw/flash_optiboot.py \
    ~/wheel_odom_fw/v6/PacketSerialSetSpeed.ino.hex
git -C ~/feeding-deployment revert <wheel-odom commit>   # or checkout the old base_controller/
```

(New host code + v6 firmware is also safe — `/wheel_odom` just stays silent.)

## ⚑ 4. Wheel odom node (compute box)

```bash
rosrun feeding_deployment wheel_odom_publisher.py
# checks:
rostopic hz /wheel_odom            # ~10 Hz (dedup'd to the firmware poll rate)
rostopic echo /wheel_odom/counts   # [millis, a1, a2, b1, b2, ok_a, ok_b, resets]
```

Push the robot gently by hand: pose x should grow; front/rear disagreement
warnings should stay quiet. Bounce base_server mid-stream: `/wheel_odom` pose
must hold (re-baseline), not teleport.

## ⚑ 5. Calibration (on the home's main floor surface)

1. **counts_per_meter**: tape-measure a straight 2.0 m teleop drive; scale the
   default (6610) by measured/reported distance. 1–2% agreement expected.
2. **track_width_m**: rotate 2x360° in place (teleop), read total yaw from
   `/wheel_odom`; `track_width_m *= reported_yaw / (4*pi)`. Grippy wheels scrub:
   the effective value is commonly 1.3–2x the geometric spacing and differs
   between hardwood and rug — calibrate where the robot actually drives.
3. Sanity: short drive, compare `/wheel_odom` linear distance against ZED odom.

Set the calibrated values as node params (or bake new defaults into
`wheel_odom_publisher.py`).

## Drift test (wheel vs ZED vs Cartographer, live on the map)

The diagnostic the wheel odometry was built for. Four traces in RViz on the
known map: **green** = live Cartographer (lidar reference), **red** = raw ZED
odom (open loop), **orange** = sanitized ZED odom (open loop), **blue** = wheel
odom (open loop). Reading: red/orange peeling from green = VIO drift, located
on the map (this is the instrument for the Jul 8 slow-creep failure); blue
peeling from green = wheel slip; red-vs-blue disagreements are arbitrated by
green.

Bring-up:

```bash
# 1. NUC: base_server up (tmux 'robot').  2. compute: sensors.launch.
# 3. compute:
roslaunch feeding_deployment zed_drift_test.launch          # carto:=false if
                                                            # localization pane
                                                            # already runs
# 4. your terminal:
rosrun feeding_deployment drift_lock.py
#    -> live map pose prints; let localization settle (scan hugs the walls in
#       RViz), press ENTER to lock the anchor. Traces start from that point.
#       Press ENTER again anytime to RE-lock (clears traces). Ctrl-C to exit.
# 5. drive with the Xbox X-deadman; watch the traces.
```

Watching the health monitor (included by default, `monitor:=false` to omit):

```bash
rostopic echo /nav_safety_hold_reason   # every currently-firing channel,
                                        # " + "-joined; empty string = clear
```

The monitor's own log in the launch terminal narrates with timestamps:
`HOLD asserted [...]`, `HOLD now [...]` (mid-hold escalations), and
`HOLD released after Ns -- recovered (fired: ...)`. It is purely
observational here — holds never stop your driving (teleop is hold-exempt,
and the keyboard tool bypasses the bridge entirely). Two expected quirks:
`clear_costmaps failed` warnings when `yank` fires (no move_base running),
and the `jump` channel only auto-pauses for **Xbox** teleop — keyboard
driving above ~0.5 m/s (`--max_translation` ≳ 2400) will false-trigger it
(at the suggested 1500 ≈ 0.31 m/s all gates clear).

### Traces during REAL runs (full nav stack)

The observers are split into `launch/drift_traces.launch`, which is safe to
run ALONGSIDE the full stack (no node/topic overlap):

```bash
# full stack up as usual (sensors, cartographer pane, navigation.launch,
# shared_autonomy.launch), then:
roslaunch feeding_deployment drift_traces.launch          # record:=true for a bag
rosrun feeding_deployment drift_lock.py                   # ENTER to lock/re-lock
```

Traces render in the normal `vention.rviz` view under the **"Drift traces"**
group (empty until you lock; harmless when the tracer isn't running). Unlike
the teleop-only test, holds from the health monitor DO gate autonomy here —
that's the real navigation behavior, and the traces let you see what the pose
sources were doing when a hold fires.

Caveats:
- **Do NOT run alongside navigation.launch / shared_autonomy.launch** — same
  node names, ROS silently kills the older instances. (This applies to
  `zed_drift_test.launch`; `drift_traces.launch` is the alongside-safe one.)
- Open-loop traces are baked with the anchor-time `map→odom`; post-lock
  Cartographer yanks move only the green trace (by design).
- A full ZED restart re-zeroes its odom origin → red/orange teleport; re-lock.
  A wheel_odom_publisher restart freezes the blue trace (loud log); re-lock.
- Run RViz on the operator laptop if possible (heavy rendering on the compute
  box starves ZED VIO); `rviz:=false` disables the local one.
- `record:=true` bags the odoms + tf + scans (no images/Paths) into
  `system_logs/`.

## Files

| What | Where |
|---|---|
| Firmware v7 | `src/feeding_deployment/control/base_controller/PacketSerialSetSpeed/PacketSerialSetSpeed.ino` |
| Flasher (no avrdude needed — NUC can't run the 32-bit one) | `scripts/flash_optiboot.py` |
| Bench tool | `scripts/bench_test_encoders.py` |
| Calibration tool (straight/rotate drive) | `scripts/calibrate_wheel_odom.py` |
| Host bridge (reader thread, echo-confirm) | `.../base_controller/vention_arduino_control.py` |
| RPC | `.../base_controller/base_interface.py`, `base_client.py` |
| ROS node | `.../base_controller/wheel_odom_publisher.py` |
| Staged artifacts on NUC | `~/wheel_odom_fw/{v6,v7}/*.hex`, bench + flasher copies |
