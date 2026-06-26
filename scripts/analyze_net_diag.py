#!/usr/bin/env python3
"""analyze_net_diag.py -- offline analysis for the Mac->NUC link diagnostic.

Run AFTER a `net_diag_sender.py` (Mac) + `net_diag_receiver.py` (NUC) session has
finished and you have collected all CSVs into one directory:

    <run-dir>/
        recv.csv          (from the NUC receiver)
        gaps.csv          (NUC, optional)
        sys.csv           (NUC, optional)
        sent.csv          (from the Mac sender)
        acks.csv          (Mac)
        wifi_stats.csv    (Mac)
        wifi_events.log   (Mac, optional)
        run_meta.json     (Mac)

It computes forward loss, loss-event list, inter-arrival gaps, RTT stats, and --
the payoff -- correlates each loss event with the nearest macOS WiFi event and the
RSSI just before it, so each drop gets a likely cause. Writes report.txt (and a
timeline PNG if matplotlib is available) into <run-dir>.

    python scripts/analyze_net_diag.py <run-dir>
"""

import argparse
import csv
import datetime
import json
import os
import re
import sys

import numpy as np

CORRELATE_WINDOW_S = 2.0   # match WiFi events within +/- this of a loss event
RF_FADE_DBM = -70.0        # RSSI at/below this near a drop -> suspect RF fade
# Match only MEANINGFUL state-change events, not the constant GET-BSSID/NOISE/
# CHANNEL polling noise (which would swamp the nearest-event match). These are the
# things that actually take the radio off-channel: scans, AWDL, roams, channel
# switches, (de)auth/disassoc, link up/down.
WIFI_EVENT_KEYWORDS = re.compile(
    r"\[SCAN\]|\bAWDL\b|\broam|\bCSA\b|disassoc|deauth|channel switch|"
    r"link (?:up|down)|beacon loss|disconnect",
    re.IGNORECASE)


def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def _floats(rows, key):
    out = []
    for r in rows:
        v = r.get(key, "")
        if v != "":
            try:
                out.append(float(v))
            except ValueError:
                pass
    return np.array(out)


def load_run(run_dir):
    recv = _read_csv(os.path.join(run_dir, "recv.csv"))
    sent = _read_csv(os.path.join(run_dir, "sent.csv"))
    acks = _read_csv(os.path.join(run_dir, "acks.csv"))
    wifi = _read_csv(os.path.join(run_dir, "wifi_stats.csv"))
    sysrows = _read_csv(os.path.join(run_dir, "sys.csv"))
    meta = {}
    meta_path = os.path.join(run_dir, "run_meta.json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    events = parse_wifi_events(os.path.join(run_dir, "wifi_events.log"))
    return recv, sent, acks, wifi, sysrows, meta, events


def parse_wifi_events(path):
    """Return [(epoch, text), ...] from a `log show --style syslog` dump."""
    if not os.path.exists(path):
        return []
    events = []
    # syslog style: "2026-06-26 14:23:01.123456-0400  host process[pid] <Level>: msg"
    ts_re = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+[+-]\d{4})\s+(.*)$")
    with open(path, errors="replace") as f:
        for line in f:
            m = ts_re.match(line)
            if not m or not WIFI_EVENT_KEYWORDS.search(m.group(2)):
                continue
            try:
                dt = datetime.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f%z")
            except ValueError:
                continue
            # Drop the constant boilerplate prefix (...(CoreWiFi) [...] [corewifi])
            # so the meaningful tail (e.g. "BEGIN REQ [SCAN] ...") survives truncation.
            text = re.sub(r"^.*\[corewifi\]\s*", "", m.group(2).strip())
            events.append((dt.timestamp(), text))
    return events


def find_loss_events(recv):
    """Contiguous missing-seq runs -> list of dicts. Duration is the directly
    measured wall-clock gap between the packets bracketing the loss (no rate
    assumption), and we also derive the effective send rate from the data."""
    seq_to_wall = {}
    for r in recv:
        try:
            seq_to_wall[int(r["seq"])] = float(r["t_recv_wall"])
        except (ValueError, KeyError):
            continue
    seqs = sorted(seq_to_wall)
    events = []
    for a, b in zip(seqs, seqs[1:]):
        if b > a + 1:
            events.append({
                "start_wall": seq_to_wall[a],
                "after_seq": a, "before_seq": b, "lost": b - a - 1,
                "duration_s": seq_to_wall[b] - seq_to_wall[a],
            })
    eff_rate = float("nan")
    if len(seqs) > 1:
        span_s = seq_to_wall[seqs[-1]] - seq_to_wall[seqs[0]]
        if span_s > 0:
            eff_rate = (seqs[-1] - seqs[0]) / span_s
    return events, seqs, eff_rate


def rssi_before(wifi_t, wifi_rssi, t):
    if wifi_t.size == 0:
        return None
    idx = np.searchsorted(wifi_t, t) - 1
    if idx < 0:
        return None
    v = wifi_rssi[idx]
    return None if np.isnan(v) else float(v)


def nearest_event(events, t, window=CORRELATE_WINDOW_S):
    best = None
    for et, text in events:
        dt = abs(et - t)
        if dt <= window and (best is None or dt < best[0]):
            best = (dt, et, text)
    return best  # (dt, epoch, text) or None


def sys_at(sys_t, sys_load, sys_psi, t):
    if sys_t.size == 0:
        return None, None
    idx = np.searchsorted(sys_t, t) - 1
    if idx < 0:
        idx = 0
    return float(sys_load[idx]), float(sys_psi[idx])


def classify(ev, rssi_b, near_evt, nuc_load, nuc_psi):
    if near_evt is not None:
        return f"wifi-event ({near_evt[2][:80]})"
    if rssi_b is not None and rssi_b <= RF_FADE_DBM:
        return f"RF-fade (RSSI {rssi_b:.0f} dBm before)"
    if (nuc_psi is not None and nuc_psi > 25.0) or (nuc_load is not None and nuc_load > 3.0):
        return f"NUC-stall (load {nuc_load:.1f}, psiMem {nuc_psi:.0f})"
    return "unexplained"


def pct(arr, q):
    return float(np.percentile(arr, q)) if arr.size else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", help="directory holding the collected CSVs")
    ap.add_argument("--top", type=int, default=30, help="max loss events to list")
    args = ap.parse_args()

    recv, sent, acks, wifi, sysrows, meta, events = load_run(args.run_dir)
    if not recv:
        sys.exit(f"no recv.csv in {args.run_dir} (need the NUC receiver output)")

    nominal_rate = float(meta.get("rate", 100.0))
    n_sent = int(meta.get("sent", len(sent)))
    n_recv = len(recv)
    n_acked = int(meta.get("acked", len(acks)))

    loss_events, seqs, eff_rate = find_loss_events(recv)
    seq_span = (seqs[-1] - seqs[0] + 1) if seqs else 0
    forward_lost = seq_span - n_recv
    forward_loss_pct = 100.0 * forward_lost / seq_span if seq_span else 0.0

    # Inter-arrival gaps (recompute from recv monotonic times, in arrival order).
    recv_mono = _floats(recv, "t_recv_mono")
    gaps_ms = np.diff(recv_mono) * 1000.0 if recv_mono.size > 1 else np.array([])

    rtt = _floats(acks, "rtt_ms")
    wifi_t = _floats(wifi, "t_wall")
    wifi_rssi = _floats(wifi, "rssi") if wifi else np.array([])
    # Keep wifi arrays aligned/sorted by time.
    if wifi_t.size and wifi_t.size == wifi_rssi.size:
        order = np.argsort(wifi_t)
        wifi_t, wifi_rssi = wifi_t[order], wifi_rssi[order]
    else:
        wifi_rssi = np.full(wifi_t.shape, np.nan)

    sys_t = _floats(sysrows, "t_wall")
    sys_load = _floats(sysrows, "load_norm")
    sys_psi = _floats(sysrows, "psi_mem_avg10")

    lines = []
    lines.append("=== Mac->NUC link diagnostic report ===")
    if meta:
        dur = meta.get("t1_wall", 0) - meta.get("t0_wall", 0)
        lines.append(f"host={meta.get('host')} nominal_rate={nominal_rate:.0f}Hz "
                     f"effective_rate={eff_rate:.1f}Hz duration={dur:.0f}s")
    lines.append(f"sent={n_sent}  received={n_recv}  acked={n_acked}")
    lines.append(f"forward loss: {forward_lost} / {seq_span} seqs ({forward_loss_pct:.3f}%)")
    rev_lost = max(0, n_sent - n_acked)
    lines.append(f"reverse loss (sent-acked): {rev_lost} "
                 f"({100.0 * rev_lost / max(1, n_sent):.3f}%)")
    if gaps_ms.size:
        lines.append(f"inter-arrival gap: median={np.median(gaps_ms):.1f}ms "
                     f"p99={pct(gaps_ms, 99):.1f}ms max={gaps_ms.max():.0f}ms")
    if rtt.size:
        lines.append(f"RTT ms: median={pct(rtt, 50):.1f} p99={pct(rtt, 99):.1f} "
                     f"max={rtt.max():.1f}")
    n_send_err = int(meta.get("send_errors", 0))
    if n_send_err:
        lines.append(f"sender interface errors: {n_send_err} (WiFi dropped the Mac's "
                     f"IP mid-send; see send_errors.csv) -- these are full L2/L3 drops, "
                     f"worse than a scan blackout")
    lines.append(f"WiFi samples: {wifi_t.size}  parsed WiFi events: {len(events)}")
    lines.append(f"loss events (missing-seq runs): {len(loss_events)}")
    lines.append("")

    # Correlate each loss event, biggest first.
    loss_events.sort(key=lambda e: e["lost"], reverse=True)
    lines.append(f"--- top {min(args.top, len(loss_events))} loss events ---")
    lines.append(f"{'wall_time':19} {'lost':>5} {'dur_s':>6} {'rssi_b':>6} "
                 f"{'nuc_ld':>6} cause")
    cause_tally = {}
    for ev in loss_events[:args.top]:
        t = ev["start_wall"]
        rb = rssi_before(wifi_t, wifi_rssi, t)
        ne = nearest_event(events, t)
        nl, npsi = sys_at(sys_t, sys_load, sys_psi, t)
        cause = classify(ev, rb, ne, nl, npsi)
        cause_tally[cause.split(" (")[0]] = cause_tally.get(cause.split(" (")[0], 0) + 1
        tstr = datetime.datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S")
        lines.append(f"{tstr:19} {ev['lost']:>5} {ev['duration_s']:>6.2f} "
                     f"{('%.0f' % rb) if rb is not None else '?':>6} "
                     f"{('%.1f' % nl) if nl is not None else '?':>6} {cause}")

    lines.append("")
    lines.append("--- cause tally (all listed events) ---")
    for c, n in sorted(cause_tally.items(), key=lambda x: -x[1]):
        lines.append(f"  {n:>4}  {c}")
    lines.append("")
    lines.append(verdict(forward_loss_pct, loss_events, cause_tally))

    report = "\n".join(lines)
    print(report)
    out_path = os.path.join(args.run_dir, "report.txt")
    with open(out_path, "w") as f:
        f.write(report + "\n")
    print(f"\nwrote {out_path}")

    make_timeline(args.run_dir, wifi_t, wifi_rssi, loss_events)


def verdict(forward_loss_pct, loss_events, cause_tally):
    if forward_loss_pct < 0.01 and not loss_events:
        return ("VERDICT: air link is CLEAN over the run. The drops seen in the live "
                "system are NOT this WiFi hop -- look at the ROS hop (bridge->bulldog) "
                "or whether the updated bulldog is actually deployed on the NUC.")
    if not cause_tally:
        return "VERDICT: loss present but no events listed."
    top = max(cause_tally.items(), key=lambda x: x[1])
    return (f"VERDICT: dominant loss cause = '{top[0]}' ({top[1]} events). "
            f"Act on that class (see runbook for fixes).")


def make_timeline(run_dir, wifi_t, wifi_rssi, loss_events):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    if wifi_t.size == 0:
        return
    fig, ax = plt.subplots(figsize=(14, 4))
    t0 = wifi_t[0]
    ax.plot(wifi_t - t0, wifi_rssi, lw=0.8, label="RSSI (dBm)")
    for ev in loss_events:
        x = ev["start_wall"] - t0
        ax.axvspan(x, x + max(ev["duration_s"], 0.2), color="red", alpha=0.25)
    ax.set_xlabel("seconds since start")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title("RSSI over time; red = loss events")
    ax.legend(loc="lower left")
    out = os.path.join(run_dir, "timeline.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
