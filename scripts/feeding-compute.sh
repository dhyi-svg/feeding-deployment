#!/usr/bin/env bash
# feeding-compute.sh -- compute 'feeding' tmux session: 8-pane 2x4 grid, plus a
#                       one-key (prefix + r) restart of the BOTTOM ROW only.
#
# Run this ON COMPUTE.
#
#   ./feeding-compute.sh           build the session (or attach if it exists)
#   ./feeding-compute.sh restart   Ctrl+C panes 5-8, relaunch 5 ->(10s)-> 6 ->(5s)->
#                                  7, then pre-type 8 (no Enter). Panes 1-4 are
#                                  never touched. (This is what 'prefix + r' runs.)
#
# Grid (numbered left->right, top->bottom):
#   1 roscore          2 launch_sensors   3 launch_app       4 launch_utensil
#   5 launch_watchdog  6 carto_localize   7 shared_autonomy  8 run.py
#
# Compute ~/.bashrc auto-activates ROS + conda feed + sources the workspace, so
# raw roslaunch/python work in every pane with no env prefix. Only pane 8 needs
# its cwd set to the integration dir (handled below).
#
# Permanence: 'prefix + r' is installed at build time (lasts for the tmux server's
# life). To survive a full server restart, also add to compute ~/.tmux.conf:
#   bind r run-shell -b "~/deployment_ws/src/feeding-deployment/scripts/feeding-compute.sh restart"

set -euo pipefail

SESSION=feeding
WIN=feeding
TARGET="$SESSION:$WIN"

# Tunables (env-overridable).
RESTART_GRACE="${RESTART_GRACE:-2}"               # seconds after C-c before relaunching pane 5
POST_WATCHDOG_DELAY="${POST_WATCHDOG_DELAY:-10}"  # after watchdog (5), before cartographer (6)
INTER_DELAY="${INTER_DELAY:-5}"                   # after cartographer (6), before shared_autonomy (7)

# Absolute path to this script, so the tmux binding can call it back.
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

INTEGRATION_DIR="$HOME/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration"
MAP_FILE="$HOME/deployment_ws/src/feeding-deployment/maps/emprise_572_6-8.pbstream"

# Commands per pane (1..8). Defined once; shared by build and restart so they
# can't drift. Panes 6/7/8 are the ones the restart relaunches besides 5.
CMD1='roscore'
CMD2='launch_sensors'
CMD3='launch_app'
CMD4='launch_utensil'
CMD5='launch_watchdog'
CMD6="roslaunch feeding_deployment cartographer_localization.launch load_state_filename:=$MAP_FILE"
CMD7='roslaunch feeding_deployment shared_autonomy.launch'
CMD8='python run.py --user bohan_jun27 --run_on_robot --use_interface --resume_from_state 21_stow_utensil --no_waits --day 1'

# ----- 'logger' tmux session (separate from 'feeding') ---------------------- #
# Two stacked panes: top = system near-hang watchdog, bottom = ROS sensor logger.
# Both keep a 3 h ROLLING window and stream/flush to disk (negligible RAM).
#   - health monitor rolls natively via --window-seconds.
#   - sensor logger accumulates per run, so we re-run it in 3 h chunks and prune
#     to the newest $LOGGER_KEEP (the rolling-window equivalent for a CSV).
# Skip entirely with NO_LOGGER=1. Tunables: LOGGER_CYCLE (s), LOGGER_KEEP.
LOGGER_SESSION=logger
SAFETY_DIR="$HOME/deployment_ws/src/feeding-deployment/src/feeding_deployment/safety"
LOGGER_LOG_DIR="$INTEGRATION_DIR/log/system_logs"   # fixed (tmux session isn't tied to a run user)
LOGGER_CYCLE="${LOGGER_CYCLE:-10800}"     # 3 h per chunk / rolling window
LOGGER_KEEP="${LOGGER_KEEP:-2}"           # sensor chunks to retain (~6 h on disk)
# Use `python` for both (the workspace/conda interpreter active in every pane).
# Both scripts mkdir their output dir, so log/system_logs/ is created as needed.
CMD_HEALTH="python $INTEGRATION_DIR/compute_health_monitor.py --no-kill --window-seconds $LOGGER_CYCLE --logfile $LOGGER_LOG_DIR/health_monitor.log"
# Distinct 'sensorlog_' prefix so the prune can NEVER match the existing
# 'sensor_diag_*' analysis runs (or anything else) in the log dir.
CMD_SENSORLOG="until rostopic list >/dev/null 2>&1; do sleep 3; done; while true; do python $SAFETY_DIR/sensor_diag_logger.py --duration $LOGGER_CYCLE --outdir $LOGGER_LOG_DIR/sensorlog_\$(date +%Y%m%d_%H%M%S); ls -dt $LOGGER_LOG_DIR/sensorlog_* 2>/dev/null | tail -n +$((LOGGER_KEEP+1)) | xargs -r rm -rf; done"

# Attach to $SESSION, or switch to it if we're already inside another tmux client
# (plain 'attach' refuses to nest).
attach_or_switch() {
  if [[ -n "${TMUX:-}" ]]; then
    exec tmux switch-client -t "$SESSION"
  else
    exec tmux attach -t "$SESSION"
  fi
}

# Build the detached 'logger' session: top pane health monitor, bottom pane
# sensor logger. No-op if it already exists, so re-running build won't duplicate.
build_logger_session() {
  if tmux has-session -t "$LOGGER_SESSION" 2>/dev/null; then
    echo "logger session already running -- leaving it."
    return 0
  fi
  tmux new-session -d -s "$LOGGER_SESSION" -n "$LOGGER_SESSION"
  tmux split-window -v -t "$LOGGER_SESSION"          # top + bottom
  tmux set-option -w -t "$LOGGER_SESSION" pane-border-status top
  tmux set-option -w -t "$LOGGER_SESSION" pane-border-format ' #{pane_title} '
  local lpanes
  mapfile -t lpanes < <(tmux list-panes -t "$LOGGER_SESSION" \
      -F '#{pane_top} #{pane_id}' | sort -n -k1,1 | awk '{ print $2 }')
  tmux select-pane -t "${lpanes[0]}" -T 'health_monitor (3h rolling)'
  tmux select-pane -t "${lpanes[1]}" -T 'sensor_diag (3h rolling)'
  tmux send-keys -t "${lpanes[0]}" "$CMD_HEALTH" Enter
  tmux send-keys -t "${lpanes[1]}" "$CMD_SENSORLOG" Enter
  echo "Built tmux session '$LOGGER_SESSION' (health top, sensors bottom; ${LOGGER_CYCLE}s rolling)."
}

# Print the 8 pane ids in VISUAL order (top->bottom, then left->right). Title-free
# on purpose: htop/roslaunch overwrite pane titles, so geometry is the only stable
# key. Each line of output is one pane id; caller reads into an array.
ordered_panes() {
  tmux list-panes -t "$TARGET" -F '#{pane_top} #{pane_left} #{pane_id}' \
    | sort -n -k1,1 -k2,2 \
    | awk '{ print $3 }'
}

# ------------------------------------------------------------------ restart ---
do_restart() {
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "restart: session '$SESSION' not found -- run '$SELF' first." >&2
    exit 1
  fi

  local panes
  mapfile -t panes < <(ordered_panes)
  if (( ${#panes[@]} < 8 )); then
    echo "restart: expected >=8 panes in $TARGET, found ${#panes[@]}." >&2
    echo "  run '$SELF' to (re)build the session." >&2
    exit 1
  fi
  local p5="${panes[4]}" p6="${panes[5]}" p7="${panes[6]}" p8="${panes[7]}"

  # 1. Stop ONLY the bottom row (5-8). Top row (1-4) is left untouched.
  tmux send-keys -t "$p5" C-c
  tmux send-keys -t "$p6" C-c
  tmux send-keys -t "$p7" C-c
  tmux send-keys -t "$p8" C-c

  # 2. Let them exit / free ports.
  sleep "$RESTART_GRACE"

  # 3. Relaunch staggered: watchdog, wait 10s, cartographer, wait 5s, shared_autonomy.
  #    (C-u clears any partial input.)
  tmux send-keys -t "$p5" C-u "$CMD5" Enter
  sleep "$POST_WATCHDOG_DELAY"
  tmux send-keys -t "$p6" C-u "$CMD6" Enter
  sleep "$INTER_DELAY"
  tmux send-keys -t "$p7" C-u "$CMD7" Enter

  # 4. Pane 8: set cwd, then PRE-TYPE the run.py command WITHOUT Enter so the user
  #    can edit it before running.
  tmux send-keys -t "$p8" C-u "cd $INTEGRATION_DIR" Enter
  tmux send-keys -t "$p8" C-u "$CMD8"

  tmux display-message "feeding-compute: restarted 5-7; pane 8 pre-typed (edit + Enter to run)"
}

# -------------------------------------------------------------------- build ---
do_build() {
  # Ensure the independent 'logger' session exists (built before we possibly
  # exec into an attach below). Set NO_LOGGER=1 to skip.
  if [[ -z "${NO_LOGGER:-}" ]]; then
    build_logger_session
  fi

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "session '$SESSION' already exists -- attaching."
    attach_or_switch
  fi

  # Build an even 2x4 grid. NOT 'tiled' (that makes 3 columns for 8 panes):
  #   - make 4 even columns, then split each column into top/bottom.
  tmux new-session -d -s "$SESSION" -n "$WIN"
  tmux split-window -h -t "$TARGET"      # 2 cols
  tmux split-window -h -t "$TARGET"      # 3 cols
  tmux split-window -h -t "$TARGET"      # 4 cols
  tmux select-layout -t "$TARGET" even-horizontal
  # Split each of the 4 columns into two rows.
  local col
  for col in $(tmux list-panes -t "$TARGET" -F '#{pane_id}'); do
    tmux split-window -v -t "$col"
  done

  # Pane title bar (display only; restart does NOT depend on titles).
  tmux set-option -w -t "$TARGET" pane-border-status top
  tmux set-option -w -t "$TARGET" pane-border-format ' #{pane_index} #{pane_title} '

  # Map panes in VISUAL order and pre-type each command (no Enter -- you fire the
  # bringup yourself).
  local panes
  mapfile -t panes < <(ordered_panes)
  local titles=(roscore sensors app utensil watchdog carto shared_auto run)
  local cmds=("$CMD1" "$CMD2" "$CMD3" "$CMD4" "$CMD5" "$CMD6" "$CMD7" "$CMD8")
  local i
  for i in "${!cmds[@]}"; do
    tmux select-pane -t "${panes[$i]}" -T "$((i+1)) ${titles[$i]}"
    if (( i == 7 )); then
      # Pane 8: set cwd first (Enter), then pre-type run.py WITHOUT Enter.
      tmux send-keys -t "${panes[$i]}" C-u "cd $INTEGRATION_DIR" Enter
      tmux send-keys -t "${panes[$i]}" C-u "${cmds[$i]}"
    else
      tmux send-keys -t "${panes[$i]}" "${cmds[$i]}"
    fi
  done

  tmux select-pane -t "${panes[0]}"

  # Install the restart hotkey: prefix + r.
  tmux bind-key r run-shell -b "$SELF restart"

  echo "Built tmux session '$SESSION' (2x4 grid; commands pre-typed, no Enter)."
  echo "Restart bottom row (5-8) anytime with 'prefix + r'."
  attach_or_switch
}

case "${1:-build}" in
  build)   do_build ;;
  restart) do_restart ;;
  *) echo "usage: $0 [build|restart]" >&2; exit 2 ;;
esac
