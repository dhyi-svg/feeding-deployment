#!/usr/bin/env bash
# perf_env.sh — put the compute box in a real-time-friendly power state.
#
# The nav stack (Cartographer scan-matching + move_base) is bursty and latency
# sensitive. This box ships with the CPU governor on "powersave" (pinned ~2.4 GHz
# even idle), which needlessly slows the bursts. This flips all cores to
# "performance" and prints thermal + governor status so you can confirm.
#
# It does NOT touch ROS or any config. Run it once per boot, before a tuning
# session. Needs sudo for the governor write (that is the only privileged action).
#
#   scripts/perf_env.sh              # set performance, show status
#   scripts/perf_env.sh --status     # show status only, no changes
#   scripts/perf_env.sh --restore    # set governor back to powersave
#
# Optional Cartographer P-core pinning is printed as a suggestion, not applied,
# because the node's PID isn't known until it's launched:
#   taskset -cp 0-15 $(pgrep -f cartographer_node)   # P-cores on i9-14900HX

set -euo pipefail

GOV="performance"
ACTION="set"
case "${1:-}" in
  --status)  ACTION="status" ;;
  --restore) GOV="powersave" ;;
  "" )       : ;;
  * ) echo "usage: perf_env.sh [--status|--restore]" >&2; exit 2 ;;
esac

show_status() {
  echo "== CPU governor (per policy) =="
  for f in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
    [ -r "$f" ] && echo "  $f -> $(cat "$f")"
  done | sort -u | head -8
  echo "== current freq (cpu0) =="
  cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null | awk '{printf "  %.0f MHz\n", $1/1000}'
  echo "== thermal =="
  if command -v sensors >/dev/null 2>&1; then
    sensors 2>/dev/null | grep -iE "package id|tctl|cpu" | head -4 | sed 's/^/  /'
  else
    for z in /sys/class/thermal/thermal_zone*/temp; do
      [ -r "$z" ] && awk '{printf "  %s: %.1f C\n", FILENAME, $1/1000}' "$z"
    done | head -4
  fi
  echo "== load =="
  echo "  $(uptime)"
}

if [ "$ACTION" = "status" ]; then
  show_status
  exit 0
fi

echo "Setting CPU governor -> $GOV (sudo required)..."
if command -v cpupower >/dev/null 2>&1; then
  sudo cpupower frequency-set -g "$GOV" >/dev/null
else
  for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo "$GOV" | sudo tee "$f" >/dev/null
  done
fi
echo "Done."
echo
show_status
echo
echo "Suggestion (run after Cartographer is up, not applied here):"
echo "  taskset -cp 0-15 \$(pgrep -f cartographer_node)   # pin to P-cores"
