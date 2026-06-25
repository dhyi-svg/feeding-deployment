#!/usr/bin/env bash
# freeze_watch.sh - Resource/thermal logger to diagnose the laptop freeze that
# happens after running cartographer_localization + shared_autonomy for a while.
#
# Runs INDEPENDENTLY of ROS (plain bash) so it keeps logging even if roscore
# dies, and it fsyncs each sample to disk so the last lines survive a hard
# freeze + manual reboot.
#
# Usage:
#   bash freeze_watch.sh            # logs to ~/freeze_watch.log every 2s
#   INTERVAL=1 LOG=~/fw.log bash freeze_watch.sh
#
# After a freeze + reboot, read the tail:
#   tail -n 40 ~/freeze_watch.log
# The last timestamp = moment before the freeze. Look for a temp near 100C
# (thermal), available-MEM near 0 (OOM), or one process pinning CPU/GPU.

set -u
LOG="${LOG:-$HOME/freeze_watch.log}"
INTERVAL="${INTERVAL:-2}"
SYNC_EVERY="${SYNC_EVERY:-3}"   # fsync every N samples (keeps disk cost low)

# Thresholds for the WARN flag (Celsius / MiB).
CPU_WARN="${CPU_WARN:-90}"
GPU_WARN="${GPU_WARN:-85}"
MEM_WARN_MB="${MEM_WARN_MB:-800}"

have_nvidia=0; command -v nvidia-smi >/dev/null 2>&1 && have_nvidia=1
have_sensors=0; command -v sensors  >/dev/null 2>&1 && have_sensors=1

echo "=== freeze_watch started $(date '+%F %T')  pid=$$  interval=${INTERVAL}s -> $LOG ===" >> "$LOG"
sync

cpu_temp() {
  if [ "$have_sensors" = 1 ]; then
    sensors 2>/dev/null | awk -F'[+.]' '/Package id 0/{print $2; exit}'
  else
    awk '{printf "%d", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null
  fi
}

i=0
while true; do
  ts=$(date '+%F %T')
  ct=$(cpu_temp); ct=${ct:-0}

  # memory (MiB) + swap used
  read -r mem_used mem_avail swap_used < <(free -m | awk '
    /^Mem:/{u=$3; a=$7} /^Swap:/{s=$3} END{print u, a, s}')

  load=$(awk '{print $1" "$2" "$3}' /proc/loadavg)

  gpu_line="gpu=n/a"
  if [ "$have_nvidia" = 1 ]; then
    gpu=$(nvidia-smi --query-gpu=temperature.gpu,utilization.gpu,memory.used \
          --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
    gt=${gpu%%,*}; rest=${gpu#*,}; gu=${rest%%,*}; gm=${rest#*,}
    gpu_line="gpuT=${gt}C gpuUtil=${gu}% gpuMem=${gm}MiB"
  else
    gt=0
  fi

  # top 3 CPU consumers
  top3=$(ps -eo pcpu,comm --sort=-pcpu --no-headers 2>/dev/null | head -3 \
         | awk '{printf "%s(%s%%) ", $2, $1}')

  flag=""
  [ "${ct%.*}" -ge "$CPU_WARN" ] 2>/dev/null && flag="$flag CPU_HOT"
  [ "${gt%.*}" -ge "$GPU_WARN" ] 2>/dev/null && flag="$flag GPU_HOT"
  [ "${mem_avail:-9999}" -le "$MEM_WARN_MB" ] 2>/dev/null && flag="$flag LOW_MEM"
  [ -n "$flag" ] && flag=" <<<WARN:$flag>>>"

  printf '%s cpuT=%sC %s mem_used=%sMiB mem_avail=%sMiB swap=%sMiB load=[%s] top:%s%s\n' \
    "$ts" "$ct" "$gpu_line" "$mem_used" "$mem_avail" "$swap_used" "$load" "$top3" "$flag" >> "$LOG"

  i=$((i+1))
  if [ $((i % SYNC_EVERY)) -eq 0 ]; then sync; fi
  sleep "$INTERVAL"
done
