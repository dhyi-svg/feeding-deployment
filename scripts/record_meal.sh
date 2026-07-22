#!/usr/bin/env bash
# record_meal.sh -- per-meal dataset recorders (rosbags) for the deployment.
#
#   ./record_meal.sh [start] [label]   start recorders + live status loop
#   ./record_meal.sh stop              gracefully stop the active recording
#   ./record_meal.sh status            one-shot liveness/size report
#
# Two always-on recorders per meal (base vision is covered by the per-bringup
# ZED SVO2 recording, scripts/zed_svo_recorder.py -> log/svo/):
#   core.bag        low-rate everything: F/T, arm+wrist state, odometry, IMU,
#                   lidar, safety (e-stop heartbeat), shared-autonomy events,
#                   move_base/cartographer, webapp channels, tf. --lz4.
#   arm_vision.bag  RealSense compressed RGB + compressedDepth + calib/meta.
#                   NO extra compression (payload is already JPEG/PNG).
#
# Output: $BAG_ROOT/meal_<stamp>[_label]/ -- OUTSIDE system_logs/ on purpose so
# SESSION_KEEP pruning can never touch dataset files (same reasoning as log/svo).
#
# Ctrl-C in the pane == graceful stop. `record_meal.sh stop` (wired into the
# feeding_stop alias) does the same from outside, and MUST run before tmux
# sessions are killed: a SIGHUP'd rosbag leaves an unindexed .bag.active.

set -uo pipefail

INTEGRATION_DIR="$HOME/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration"
# Dataset root lives on the dedicated ext4 NVMe (label "robot-data", fstab
# nofail) -- physically separate from the (near-full) OS disk.
DATASET_ROOT="${DATASET_ROOT:-/data/feeding_dataset}"
BAG_ROOT="${BAG_ROOT:-$DATASET_ROOT/bags}"
SVO_DIR="${SVO_DIR:-$DATASET_ROOT/svo}"
MIN_FREE_GB="${MIN_FREE_GB:-150}"
STATUS_PERIOD="${STATUS_PERIOD:-10}"     # seconds between status lines
ACTIVE_WAIT_S="${ACTIVE_WAIT_S:-30}"     # max wait for .active finalization

# ---------------------------------------------------------------- topic lists
CORE_TOPICS=(
  # force/torque (bias events re-zero the sensor -- required to interpret it)
  /forque/forqueSensor /forque/bias /forque/bias_status
  # arm + wrist + gripper-side state
  /joint_states /robot_joint_states /wrist_joint_states
  /cmd_wrist_joint_angles /wrist_cartesian_states /robot_cartesian_state
  /collision_force /collision_free /disable_collision_sensor
  # perception event outputs (tiny; the pkl/day-bundle is the full record)
  /aruco_pose /aruco_pose_0 /aruco_pose_1
  /attachment_center /attachment_points /handle_center /handle_points
  /head_perception/unexpected /head_perception/set_filter_noisy_readings
  # base + odometry
  /zed_mini/zed_node/imu/data /zed_mini/zed_node/imu/data_raw
  /zed_mini/zed_node/imu/data_debiased /gyro_bias_estimate
  /wheel_odom /wheel_odom/counts /wheel_odom/side_disagreement
  /odometry/fused_imu_wheel
  /cmd_vel /cmd_vel_teleop
  /cmd_vel_bridge_basicmicro/applied /cmd_vel_bridge_basicmicro/rpc_latency_s
  /lidar_l/scan /lidar_r/scan /lidar_l/scan_gated /lidar_r/scan_gated
  /scan_gate/open
  # navigation stack
  /move_base/current_goal /move_base/result /move_base/status
  /move_base/recovery_status /move_base/GlobalPlanner/plan
  /move_base/TebLocalPlannerROS/local_plan /move_base/odom_feedback
  /navigate/goal /navigate/result /navigate/status
  /map /constraint_list /submap_list /scan_matched_points2
  # safety -- compute-master view only. The e-stop chain (/experimentor_estop,
  # /bulldog_status) lives on the NUC's OWN roscore and is invisible here; its
  # record is the NUC-side logs (nuc_execution_log.txt, teardown collection).
  # /watchdog_status is a ~kHz Bool heartbeat; /watchdog_anomaly carries the
  # anomaly reason (published by safety/watchdog.py at trip time).
  /watchdog_status /watchdog_anomaly /diagnostics
  # shared-autonomy state machine (proactive-vs-reactive intervention markers)
  /shared_autonomy/takeover /shared_autonomy/resume
  /shared_autonomy/done /shared_autonomy/cancel
  # interaction / webapp channels
  /webapp_to_robot /robot_to_webapp
  /robot_settings_to_webapp /webapp_settings_to_robot
  /transfer_button /speak /skill_plan /named_locations
  /client_count /connected_clients
  /deployment/annotations
  # transforms
  /tf /tf_static
)

ARM_VISION_TOPICS=(
  /camera/color/image_raw/compressed
  /camera/aligned_depth_to_color/image_raw/compressedDepth
  /camera/color/camera_info /camera/aligned_depth_to_color/camera_info
  /camera/color/metadata /camera/depth/metadata
  /camera/extrinsics/depth_to_color
)

# ------------------------------------------------------------------- helpers
newest_meal_dir() { ls -dt "$BAG_ROOT"/meal_* 2>/dev/null | head -1; }
newest_svo()      { ls -t "$SVO_DIR"/*.svo2 2>/dev/null | head -1; }

live_pids() {   # <meal_dir> -> prints pids from .pids that are still alive
  local d="$1" pid
  [[ -f "$d/.pids" ]] || return 0
  while read -r pid; do
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && echo "$pid"
  done < "$d/.pids"
}

dir_size() { du -sh "$1" 2>/dev/null | cut -f1; }

free_gb() { df -BG --output=avail "$1" 2>/dev/null | tail -1 | tr -dc '0-9'; }

stop_meal() {   # <meal_dir>  -- SIGINT recorders, wait for finalize, summarize
  local d="$1" pid pids
  pids="$(live_pids "$d")"
  if [[ -z "$pids" ]]; then
    echo "[record_meal] no live recorders for $(basename "$d")."
  else
    echo "[record_meal] stopping recorders for $(basename "$d") ..."
    for pid in $pids; do kill -INT "$pid" 2>/dev/null || true; done
    local deadline=$(( $(date +%s) + ACTIVE_WAIT_S ))
    while compgen -G "$d/*.bag.active" >/dev/null; do
      (( $(date +%s) >= deadline )) && break
      sleep 1
    done
  fi
  # end-of-meal summary (best-effort; rosbag info needs the workspace env)
  {
    echo "stopped_iso: $(date -Is)"
    if compgen -G "$d/*.bag.active" >/dev/null; then
      echo "WARNING: unfinalized .active bags left behind:"
      ls -l "$d"/*.bag.active
      echo "  (recover with: rosbag reindex <file>)"
    fi
    local b
    for b in "$d"/*.bag; do
      [[ -e "$b" ]] || continue
      echo "---- $(basename "$b")"
      rosbag info "$b" 2>/dev/null || echo "  (rosbag info unavailable)"
    done
    echo "---- svo files during window"
    ls -l "$SVO_DIR"/*.svo2 2>/dev/null || echo "  (none)"
  } >> "$d/stop_summary.txt"
  rm -f "$d/.pids"
  if compgen -G "$d/*.bag.active" >/dev/null; then
    echo "[record_meal] *** WARNING: .active bags remain in $d -- see stop_summary.txt ***"
  else
    echo "[record_meal] stopped cleanly -> $d (summary: stop_summary.txt)"
  fi
}

# --------------------------------------------------------------------- start
do_start() {
  local label
  label="$(printf '%s' "${1:-${SESSION_LABEL:-}}" | tr ' /' '__' | tr -cd 'A-Za-z0-9_-')"

  # refuse double-start
  local prev; prev="$(newest_meal_dir)"
  if [[ -n "$prev" && -n "$(live_pids "$prev")" ]]; then
    echo "[record_meal] recorders already running for $(basename "$prev") -- 'stop' first." >&2
    exit 1
  fi

  # fstab uses nofail: if the data disk ever fails to mount, /data is an empty
  # dir on the near-full root filesystem -- refuse rather than fill the OS disk.
  if [[ "$BAG_ROOT" == /data/* ]] && ! mountpoint -q /data; then
    echo "[record_meal] REFUSING to start: /data is NOT mounted (robot-data disk missing?)" >&2
    exit 1
  fi

  echo "[record_meal] waiting for ROS master ..."
  until rostopic list >/dev/null 2>&1; do sleep 3; done

  mkdir -p "$BAG_ROOT"
  local avail; avail="$(free_gb "$BAG_ROOT")"
  if [[ -n "$avail" && "$avail" -lt "$MIN_FREE_GB" ]]; then
    echo "[record_meal] REFUSING to start: only ${avail} GB free (< ${MIN_FREE_GB} GB) on $BAG_ROOT" >&2
    exit 1
  fi

  local stamp meal_dir
  stamp="$(date +%Y%m%d_%H%M%S)"
  meal_dir="$BAG_ROOT/meal_${stamp}${label:+_$label}"
  mkdir -p "$meal_dir"

  # depth PNG effort: level 9 (default) saturates a core at 30 Hz; 3 is fast.
  # Best-effort with retries -- the camera may still be opening.
  (
    for _ in 1 2 3 4 5; do
      if rosrun dynamic_reconfigure dynparam set \
           /camera/aligned_depth_to_color/image_raw/compressedDepth png_level 3 \
           >/dev/null 2>&1; then
        echo "[record_meal] compressedDepth png_level=3 set"; exit 0
      fi
      sleep 5
    done
    echo "[record_meal] WARN: could not set png_level (camera down?) -- default 9 in effect"
  ) &

  local svo0; svo0="$(newest_svo)"
  local git_sha; git_sha="$(git -C "$INTEGRATION_DIR" rev-parse HEAD 2>/dev/null || echo unknown)"
  cat > "$meal_dir/meta.json" <<EOF
{
  "meal_dir": "$(basename "$meal_dir")",
  "label": "${label}",
  "started_iso": "$(date -Is)",
  "started_epoch": $(date +%s.%N),
  "host": "$(hostname)",
  "git": "$git_sha",
  "svo_at_start": "${svo0:-none}",
  "free_gb_at_start": ${avail:-0},
  "bags": {
    "core":       { "lz4": true,  "split_mb": 2048, "topics": $(printf '%s\n' "${CORE_TOPICS[@]}" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().split()))') },
    "arm_vision": { "lz4": false, "split_mb": 4096, "topics": $(printf '%s\n' "${ARM_VISION_TOPICS[@]}" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().split()))') }
  }
}
EOF

  echo "[record_meal] recording -> $meal_dir"
  nice -n 10 rosbag record -O "$meal_dir/core" --split --size=2048 -b 512 --lz4 \
    "${CORE_TOPICS[@]}" &
  local core_pid=$!
  nice -n 10 rosbag record -O "$meal_dir/arm_vision" --split --size=4096 -b 1024 \
    "${ARM_VISION_TOPICS[@]}" &
  local vision_pid=$!
  printf '%s\n%s\n' "$core_pid" "$vision_pid" > "$meal_dir/.pids"

  trap 'echo; stop_meal "$meal_dir"; exit 0' INT TERM

  # live status loop -- proves recording is alive at a glance
  local svo svo_sz core_n vis_n
  while :; do
    sleep "$STATUS_PERIOD"
    if [[ -z "$(live_pids "$meal_dir")" ]]; then
      echo "[record_meal] *** recorders died -- see pane output above ***"
      stop_meal "$meal_dir"
      exit 1
    fi
    core_n="$(ls "$meal_dir"/core*.bag "$meal_dir"/core*.bag.active 2>/dev/null | wc -l)"
    vis_n="$(ls "$meal_dir"/arm_vision*.bag "$meal_dir"/arm_vision*.bag.active 2>/dev/null | wc -l)"
    svo="$(newest_svo)"; svo_sz="$([[ -n "$svo" ]] && du -sh "$svo" | cut -f1 || echo n/a)"
    echo "[record_meal $(date +%H:%M:%S)] total=$(dir_size "$meal_dir")  core:${core_n} chunk(s)  vision:${vis_n} chunk(s)  svo:${svo_sz}  free:$(free_gb "$BAG_ROOT")G"
  done
}

# ---------------------------------------------------------------------- stop
do_stop() {
  local d; d="$(newest_meal_dir)"
  [[ -n "$d" ]] || { echo "[record_meal] nothing under $BAG_ROOT -- never started."; exit 0; }
  if [[ ! -f "$d/.pids" && -z "$(compgen -G "$d/*.bag.active")" ]]; then
    echo "[record_meal] $(basename "$d") already stopped."
    exit 0
  fi
  stop_meal "$d"
}

# -------------------------------------------------------------------- status
do_status() {
  local d; d="$(newest_meal_dir)"
  [[ -n "$d" ]] || { echo "[record_meal] no meal dirs under $BAG_ROOT."; exit 0; }
  echo "meal dir : $d ($(dir_size "$d"))"
  local pids; pids="$(live_pids "$d")"
  if [[ -n "$pids" ]]; then echo "recorders: RUNNING (pids: $(echo $pids | tr '\n' ' '))"
  else echo "recorders: not running"; fi
  ls -lh "$d"/*.bag* 2>/dev/null || true
  local svo; svo="$(newest_svo)"
  [[ -n "$svo" ]] && echo "svo      : $svo ($(du -sh "$svo" | cut -f1))" || echo "svo      : none"
}

case "${1:-start}" in
  start)  shift || true; do_start "${1:-}" ;;
  stop)   do_stop ;;
  status) do_status ;;
  *) echo "usage: $0 [start [label]|stop|status]" >&2; exit 2 ;;
esac
