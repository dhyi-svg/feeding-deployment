# Mac → NUC WiFi / UDP Link Diagnostics

The experimentor e-stop button is on a Mac that streams ~82–100 Hz UDP packets over
WiFi (`FeedingDeployment-5G`) to the NUC (wired to the router), which republishes them
onto `/experimentor_estop` for [`bulldog.py`](../src/feeding_deployment/safety/bulldog.py)
to monitor. Intermittent ~1 s drops on that link were tripping false e-stops.

This doc covers (1) the WiFi config rules that prevent the drops, (2) the `bulldog.py`
debounce that tolerates any that slip through, and (3) a 1-hour diagnostic to find the
*cause* of remaining drops.

---

## 1. WiFi configuration rules (do not silently revert)

Router: **Netgear Nighthawk RAX43v2**, SSID `FeedingDeployment-5G`.

- **5 GHz channel: a non-DFS channel (36/40/44/48 or 149–165).** DFS channels (52–144)
  let the AP vacate the channel for 1+ second on a (often false) radar detection — a
  clean ~1 s blackout for every client. Currently **channel 44**. ✅
- **5 GHz width: 80 MHz** (Mode = "1800 Mbps", NOT "3600 Mbps"/160 MHz). At 160 MHz the
  bonded channel around 44 spans 36–64, which **includes DFS sub-channels 52–64** — so
  160 MHz reintroduces DFS blackouts even though the control channel (44) looks safe.
  80 MHz at channel 44 spans only 36–48, all non-DFS. ✅
- **Smart Connect: OFF.** Band steering would move the Mac between 2.4/5 GHz mid-session
  (a blackout). Keep the dedicated `-5G` SSID. ✅
- Mac: run the sender under `caffeinate -dimsu`, lid open, plugged in (avoids App Nap /
  WiFi power-save freezing the send loop). Keep RSSI better than ~−65 dBm and the Mac
  clear of the moving arm (metal fades 5 GHz).
- Best long-term fix: wire the Mac to the router via USB-C Ethernet — removes WiFi from
  the safety heartbeat entirely.

## 2. bulldog.py debounce (why false stops stopped)

The frequency check is the link-liveness heartbeat, not the button press (presses are
latched on their own immediate path). It now requires the rate to stay below threshold
**continuously for `ESTOP_FREQ_GRACE_S` (1.0 s)** before tripping, so a transient ~1 s
WiFi blackout is ridden out. Threshold is **30 Hz** (nominal is ~82). Swallowed dips are
logged as `[near-miss] ...` so a degrading link is visible *before* it ever trips. If
near-misses creep toward 1.0 s, raise `ESTOP_FREQ_GRACE_S` (1.2–1.5 s).

---

## 3. The 1-hour link diagnostic

Measures the raw UDP link directly to find *why* it drops. Runs on a **separate UDP port
(5006), with no ROS**, so it cannot touch `/experimentor_estop` or interfere with the
robot. Run it **while the robot operates normally** — during the test the **physical
e-stop wired to the NUC** ([`run_bulldog.sh`](../src/feeding_deployment/integration/run_bulldog.sh))
is the real safety device; the Mac's normal `estop_sender.py` is NOT running.

Because the NUC is wired to the router, any loss seen here is over-the-air loss on the
Mac→router hop — the clean measurement point.

Static IPs: **NUC = 192.168.1.3**, **Mac = 192.168.1.8** (compute box = 192.168.1.2).

### Before you start
- Confirm both machines' clocks are roughly synced (`chronyc tracking` / `sntp`).
  Loss detection uses sequence numbers (clock-free) and RTT is measured on the Mac's one
  clock; only the coarse correlation of WiFi events vs. loss needs ~50 ms agreement.

### Run it

**On the NUC** (robot stack already up; any terminal):
```bash
conda activate controller
cd ~/feeding-deployment/src/feeding_deployment/safety
python net_diag_receiver.py
# binds udp/5006; writes recv.csv, gaps.csv, sys.csv under
# integration/log/net_diag/<timestamp>/  -- note the printed path
```

**On the Mac** (root so `wdutil` returns full WiFi stats without a mid-run prompt):
```bash
caffeinate -dimsu sudo python net_diag_sender.py --host 192.168.1.3 --duration 3600
# streams ~100 Hz for 1 h; writes sent.csv, acks.csv, wifi_stats.csv,
# wifi_events.log, run_meta.json under integration/log/net_diag/<timestamp>/
```

Let it run the full hour of normal feeding. Both sides print a live heartbeat; the NUC
prints `[gap]` lines as drops happen. Stop the receiver with Ctrl+C when the sender
finishes (it prints a summary).

### Analyze (offline, on the compute box)

Collect everything into one directory, then run the analyzer:
```bash
mkdir -p <run-dir>
# Mac (192.168.1.8): sent/acks/wifi_stats/wifi_events/meta
scp '<mac-user>@192.168.1.8:.../net_diag/<mac-run>/*' <run-dir>/
# NUC (192.168.1.3): recv.csv, gaps.csv, sys.csv
scp 'isacc@192.168.1.3:.../net_diag/<nuc-run>/*' <run-dir>/

conda activate feed
python scripts/analyze_net_diag.py <run-dir>
```

It prints and writes `<run-dir>/report.txt` (and `timeline.png`): forward/reverse loss,
inter-arrival gap and RTT stats, and a **per-loss-event table** correlating each drop
with the nearest macOS WiFi event (roam/scan/channel-switch) and the RSSI just before,
plus the concurrent NUC load. The report ends with a verdict.

### Interpreting the verdict
- **Forward loss ≈ 0, no loss events** → the air link is clean. The live-system drops are
  NOT this WiFi hop; investigate the ROS hop (bridge → bulldog) or whether the updated
  `bulldog.py` is actually deployed on the NUC.
- **Loss correlated with `wifi-event`** → roam/scan/channel-switch; reduce scans (strong
  RSSI, fixed channel) or wire the Mac.
- **Loss correlated with `RF-fade`** → weak/obstructed signal; move the Mac closer / clear
  of the arm, or wire it.
- **Loss correlated with `NUC-stall`** → not WiFi; the NUC was saturated (check `sys.csv`).
- **`unexplained`** → widen capture (`--wifi-interval 0.5`, broaden the `log show`
  predicate) and re-run.

### Manual WiFi event capture (fallback)
If the sender's auto-capture fails, on the Mac after the run:
```bash
log show --start '<YYYY-MM-DD HH:MM:SS>' \
  --predicate 'subsystem == "com.apple.wifi" OR process == "airportd" OR process == "wifid"' \
  --info --style syslog > wifi_events.log
```
(Use the `t0_wall` from `run_meta.json`, converted to local time, as the start.)
