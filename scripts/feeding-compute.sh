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
#   ./feeding-compute.sh collect   Assemble the per-run log bundle (compute + NUC
#                                  tmux/ROS/system logs) under system_logs/
#                                  session_<stamp>/ for post-hoc analysis. Single
#                                  rsync pull from the NUC; does NOT kill sessions.
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
MAP_FILE="$HOME/deployment_ws/src/feeding-deployment/maps/aimee-7-1.pbstream"

# Shared pane-logging helper (deployed on both machines via the repo).
SCRIPT_DIR="$(dirname "$SELF")"
HELPER="$SCRIPT_DIR/tmux-pane-log.sh"

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
NAVLOG_KEEP="${NAVLOG_KEEP:-10}"          # nav diag run dirs to retain

# ----- session logging: per-run bundle + tmux pane capture ------------------ #
# Each build stamps a bundle dir under system_logs and points 'current_session'
# at it; global tmux hooks pipe every pane (cleaned + ISO-timestamped) into it.
# At teardown, `collect` pulls the NUC's logs in ONE rsync and finalizes the
# bundle for post-hoc analysis. Disable pane capture with NO_PANELOG=1.
SESS_ROOT="$LOGGER_LOG_DIR"                        # .../integration/log/system_logs
SESSION_KEEP="${SESSION_KEEP:-10}"                 # per-run bundles to retain
NUC_HOST="${NUC_HOST:-emprise@192.168.1.3}"        # compute->NUC, key auth
NUC_REPO="${NUC_REPO:-/home/emprise/feeding-deployment}"
NUC_SESS_ROOT="$NUC_REPO/src/feeding_deployment/integration/log/system_logs"
# Use `python` for both (the workspace/conda interpreter active in every pane).
# Both scripts mkdir their output dir, so log/system_logs/ is created as needed.
CMD_HEALTH="python $INTEGRATION_DIR/compute_health_monitor.py --no-kill --window-seconds $LOGGER_CYCLE --logfile $LOGGER_LOG_DIR/health_monitor.log"
# Distinct 'sensorlog_' prefix so the prune can NEVER match the existing
# 'sensor_diag_*' analysis runs (or anything else) in the log dir.
CMD_SENSORLOG="until rostopic list >/dev/null 2>&1; do sleep 3; done; while true; do python $SAFETY_DIR/sensor_diag_logger.py --duration $LOGGER_CYCLE --outdir $LOGGER_LOG_DIR/sensorlog_\$(date +%Y%m%d_%H%M%S); ls -dt $LOGGER_LOG_DIR/sensorlog_* 2>/dev/null | tail -n +$((LOGGER_KEEP+1)) | xargs -r rm -rf; done"
# Nav diagnostics (scripts/nav_diag_logger.py): one navlog_<stamp> dir per
# (re)start. The script itself waits for roscore and retries its params
# snapshot until move_base/ZED are up, so pre-bringup start order is safe;
# the rostopic gate just keeps the pane quiet before roscore exists.
CMD_NAVLOG="while true; do until rostopic list >/dev/null 2>&1; do sleep 3; done; python $HOME/deployment_ws/src/feeding-deployment/scripts/nav_diag_logger.py; echo 'nav_diag_logger exited; restarting in 5s'; ls -dt $LOGGER_LOG_DIR/navlog_* 2>/dev/null | tail -n +$((NAVLOG_KEEP+1)) | xargs -r rm -rf; sleep 5; done"

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
  tmux split-window -v -t "$LOGGER_SESSION"          # 2 stacked
  tmux split-window -v -t "$LOGGER_SESSION"          # 3 stacked
  tmux select-layout -t "$LOGGER_SESSION" even-vertical
  tmux set-option -w -t "$LOGGER_SESSION" pane-border-status top
  tmux set-option -w -t "$LOGGER_SESSION" pane-border-format ' #{pane_title} '
  local lpanes
  mapfile -t lpanes < <(tmux list-panes -t "$LOGGER_SESSION" \
      -F '#{pane_top} #{pane_id}' | sort -n -k1,1 | awk '{ print $2 }')
  tmux select-pane -t "${lpanes[0]}" -T 'health_monitor (3h rolling)'
  tmux select-pane -t "${lpanes[1]}" -T 'sensor_diag (3h rolling)'
  tmux select-pane -t "${lpanes[2]}" -T 'nav_diag (per-run navlog_*)'
  tmux send-keys -t "${lpanes[0]}" "$CMD_HEALTH" Enter
  tmux send-keys -t "${lpanes[1]}" "$CMD_SENSORLOG" Enter
  tmux send-keys -t "${lpanes[2]}" "$CMD_NAVLOG" Enter
  echo "Built tmux session '$LOGGER_SESSION' (health / sensors / nav_diag; ${LOGGER_CYCLE}s rolling)."
}

# ----- session-logging helpers ---------------------------------------------- #
# Fresh per-run bundle; point current_session at it; prune old bundles.
new_bundle() {
  local stamp bundle
  stamp="$(printf '%(%Y%m%d_%H%M%S)T' -1)"
  # Optional run label (set via SESSION_LABEL env, e.g. from the feeding_start
  # alias) appended to the bundle name for easy identification later.
  bundle="$SESS_ROOT/session_${stamp}${SESSION_LABEL:+_$SESSION_LABEL}"
  mkdir -p "$bundle/compute/tmux" "$bundle/compute/ros" "$bundle/nuc"
  printf '%(%Y-%m-%dT%H:%M:%S)T' -1 > "$bundle/.started_iso"
  # Point current_session at the new bundle. If a stray REAL dir sits there,
  # remove it first -- 'ln -sfn' onto a directory links INSIDE it, not over it.
  [[ -L "$SESS_ROOT/current_session" || ! -e "$SESS_ROOT/current_session" ]] || rm -rf "$SESS_ROOT/current_session"
  ln -sfn "$bundle" "$SESS_ROOT/current_session"
  # keep newest $SESSION_KEEP, but NEVER the current one (guards against an mtime
  # tie deleting the live bundle). Glob never matches sensorlog_/sensor_diag_.
  local keep_target d
  keep_target="$(readlink -f "$SESS_ROOT/current_session" 2>/dev/null)"
  ls -dt "$SESS_ROOT"/session_* 2>/dev/null | tail -n +$((SESSION_KEEP+1)) | while read -r d; do
    [[ "$(readlink -f "$d")" == "$keep_target" ]] && continue
    rm -rf "$d"
  done
  echo "session bundle: $bundle"
}

# Install global tmux hooks so every pane (existing, script-built, or split off
# by hand later) is piped to the current bundle. Server-lifetime, like `bind r`.
# pipe-pane rides the pane, so panes keep logging straight through 'prefix + r'.
install_pane_logging() {
  [[ -n "${NO_PANELOG:-}" ]] && { echo "pane-logging: disabled (NO_PANELOG)"; return 0; }
  [[ -x "$HELPER" ]] || { echo "pane-logging: $HELPER not executable; skipping" >&2; return 0; }
  # set-hook/pipe-pane need a live session (an empty server exits immediately);
  # return 1 so the caller knows to retry once its own session is up.
  tmux list-sessions >/dev/null 2>&1 || { echo "pane-logging: no live tmux session yet; deferring"; return 1; }
  local root="$SESS_ROOT/current_session/compute"
  # Plain pipe-pane (no -o): -o TOGGLES, so a pane hit by two hooks (a new
  # session fires BOTH window-linked and session-created) would open then close.
  # window-linked + after-split-window cover new sessions/windows + splits with
  # no overlap; plain pipe-pane always ends with the pipe OPEN (idempotent).
  local pipe="pipe-pane \"TMUXLOG_ROOT='$root' exec '$HELPER' '#{session_name}' '#{window_index}' '#{pane_index}' '#{pane_id}'\""
  local mark="run-shell -b \"TMUXLOG_ROOT='$root' '$HELPER' --event '#{hook}' '#{session_name}' '#{window_index}' '#{pane_index}'\""
  local ev
  for ev in window-linked after-split-window; do
    tmux set-hook -g  "$ev" "$pipe"    # pipe the new pane
    tmux set-hook -ag "$ev" "$mark"    # + record it on the timeline
  done
  tmux set-hook -g pane-exited "run-shell -b \"TMUXLOG_ROOT='$root' '$HELPER' --event pane-exited '#{session_name}' '#{window_index}' '#{pane_index}'\""
  # backstop: pipe any panes that already exist (those created before the hooks)
  local p
  for p in $(tmux list-panes -a -F '#{pane_id}' 2>/dev/null); do
    tmux pipe-pane -t "$p" "TMUXLOG_ROOT='$root' exec '$HELPER' '#{session_name}' '#{window_index}' '#{pane_index}' '#{pane_id}'"
  done
  echo "pane-logging: hooks installed (-> $root/tmux)"
  return 0
}

# gzip a snapshot of a ROS log dir (follows the 'latest' symlink) into <dst>.
snapshot_ros_logs() {   # <src_latest_dir> <dst_ros_dir>
  local src="$1" dst="$2" f
  mkdir -p "$dst"
  [[ -d "$src" ]] || { echo "  (no ROS logs at $src)"; return 0; }
  for f in "$src"/*.log; do
    [[ -e "$f" ]] || continue
    gzip -c "$f" > "$dst/$(basename "$f").gz" 2>/dev/null || true
  done
  echo "  ROS logs -> $dst ($(ls "$dst" 2>/dev/null | wc -l) files)"
}

write_run_meta() {   # <bundle>
  local b="$1" started stopped csha size
  started="$(cat "$b/.started_iso" 2>/dev/null || echo unknown)"
  stopped="$(printf '%(%Y-%m-%dT%H:%M:%S)T' -1)"
  csha="$(git -C "$INTEGRATION_DIR" rev-parse HEAD 2>/dev/null || echo unknown)"
  size="$(du -sh "$b" 2>/dev/null | cut -f1)"
  cat > "$b/run_meta.json" <<EOF
{
  "bundle": "$(basename "$b")",
  "started": "$started",
  "collected": "$stopped",
  "compute": { "host": "$(hostname)", "user": "${USER:-}", "git": "$csha" },
  "nuc": { "host": "$NUC_HOST", "meta": "nuc/nuc_meta.json" },
  "bundle_size": "$size"
}
EOF
}

write_manifest() {   # <bundle>
  local b="$1"
  cat > "$b/MANIFEST.md" <<EOF
# Mealtime session bundle -- $(basename "$b")

Self-contained logs for one feeding run (compute + NUC), for post-hoc analysis.

## Reading order
1. compute/tmux/events.log + nuc/tmux/events.log -- tmux timeline (pane
   create/exit, prefix+r restarts). Anchor for everything else.
2. nuc_execution_log.txt -- e-stop activations + heartbeat-loss anomalies
   (pushed from the NUC's bulldog over SFTP).
3. compute/sensorlog_*/ -- per-stream Hz, dropouts, USB events (events.log,
   samples.csv), and compute/health_monitor.log (CPU/GPU/mem/temp).
4. compute/tmux/*.log + nuc/tmux/*.log -- every pane's commands + output,
   ANSI-stripped and ISO-timestamped (grep by wall-clock time).
5. compute/ros/*.log.gz + nuc/ros/*.log.gz -- ROS rosout/master/node logs.

## Layout
    run_meta.json          run window, hosts, git SHAs, size
    compute/  tmux/ ros/ health_monitor.log execution_log.txt sensorlog_*/
    nuc/      tmux/ ros/ nuc_meta.json
    nuc_execution_log.txt

All timestamps are wall-clock (machine local time) so streams line up across files.
EOF
}

# Teardown: assemble the bundle. Single rsync pull from the NUC (no per-run
# network traffic). Does NOT kill the tmux sessions -- do that yourself after.
do_collect() {
  local link="$SESS_ROOT/current_session" bundle
  [[ -e "$link" ]] || { echo "collect: no current_session under $SESS_ROOT -- run build first." >&2; exit 1; }
  bundle="$(readlink -f "$link")"
  local cdir="$bundle/compute" ndir="$bundle/nuc"
  mkdir -p "$cdir/ros" "$ndir"
  echo "collect: bundle = $bundle"

  # --- compute-side snapshots ---
  echo "collect: snapshotting compute logs ..."
  cp -f "$INTEGRATION_DIR/log/execution_log.txt"     "$cdir/"   2>/dev/null || true
  cp -f "$INTEGRATION_DIR/log/nuc_execution_log.txt" "$bundle/" 2>/dev/null || true
  cp -f "$SESS_ROOT/health_monitor.log"              "$cdir/"   2>/dev/null || true
  local slog; slog="$(ls -dt "$SESS_ROOT"/sensorlog_* 2>/dev/null | head -1)"
  [[ -n "$slog" ]] && cp -a "$slog" "$cdir/" 2>/dev/null || true
  local nlog; nlog="$(ls -dt "$SESS_ROOT"/navlog_* 2>/dev/null | head -1)"
  [[ -n "$nlog" ]] && cp -a "$nlog" "$cdir/" 2>/dev/null || true
  snapshot_ros_logs "$HOME/.ros/log/latest" "$cdir/ros"

  # --- NUC side: resolve its bundle, prep (gzip ROS + meta) remotely, pull once ---
  echo "collect: pulling NUC logs from $NUC_HOST (single rsync) ..."
  local nuc_bundle
  nuc_bundle="$(ssh -o BatchMode=yes -o ConnectTimeout=8 "$NUC_HOST" \
      "readlink -f '$NUC_SESS_ROOT/current_session' 2>/dev/null" 2>/dev/null || true)"
  if [[ -n "$nuc_bundle" ]]; then
    ssh -o BatchMode=yes -o ConnectTimeout=8 "$NUC_HOST" 'bash -s' <<'REMOTE' 2>/dev/null || \
        echo "collect: WARN NUC prep step failed" >&2
set -u
L="$HOME/feeding-deployment/src/feeding_deployment/integration/log/system_logs/current_session"
[ -e "$L" ] || exit 0
B="$(readlink -f "$L")"; mkdir -p "$B/ros"
if [ -d "$HOME/.ros/log/latest" ]; then
  for f in "$HOME"/.ros/log/latest/*.log; do
    [ -e "$f" ] || continue; gzip -c "$f" > "$B/ros/$(basename "$f").gz"
  done
fi
printf '{"host":"%s","git":"%s","when":"%s"}\n' \
  "$(hostname)" "$(git -C "$HOME/feeding-deployment" rev-parse HEAD 2>/dev/null)" "$(date -Is)" \
  > "$B/nuc_meta.json"
REMOTE
    rsync -az -e ssh "$NUC_HOST:$nuc_bundle/" "$ndir/" 2>/dev/null \
      || echo "collect: WARN rsync from NUC failed (logs remain on the NUC)" >&2
  else
    echo "collect: WARN could not resolve NUC current_session -- is feeding-nuc.sh running there?" >&2
  fi

  write_run_meta "$bundle"
  write_manifest "$bundle"
  echo "collect: done -> $bundle"
  echo "         (tmux sessions left running; kill them when ready)"
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
  [[ -n "${NO_PANELOG:-}" ]] || tmux run-shell -b "TMUXLOG_ROOT='$SESS_ROOT/current_session/compute' '$HELPER' --event restart '$SESSION' 0 0"
}

# -------------------------------------------------------------------- build ---
do_build() {
  # Fresh build if the 'feeding' session isn't up yet.
  local fresh=1
  if tmux has-session -t "$SESSION" 2>/dev/null; then fresh=0; fi

  # New per-run bundle on a fresh build (or if the symlink vanished).
  if (( fresh )) || [[ ! -e "$SESS_ROOT/current_session" ]]; then
    new_bundle
  fi

  # Ensure the independent 'logger' session exists (built before we possibly
  # exec into an attach below). Set NO_LOGGER=1 to skip.
  if [[ -z "${NO_LOGGER:-}" ]]; then
    build_logger_session
  fi

  # A session is now alive (unless NO_LOGGER), so install hooks here to capture
  # the feeding panes below from birth. Defers (returns 1) if there's still no
  # session (NO_LOGGER=1); the fallback install after the grid is built catches
  # those panes via the backstop instead.
  local _hooked=0
  if install_pane_logging; then _hooked=1; fi

  if (( ! fresh )); then
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

  # Fallback install only if the earlier one deferred (NO_LOGGER: no session then).
  # Now that the feeding session exists, this sets the hooks and backstop-pipes the
  # 8 panes. Skipped when already hooked, so panes piped at birth aren't re-piped.
  (( _hooked )) || install_pane_logging

  echo "Built tmux session '$SESSION' (2x4 grid; commands pre-typed, no Enter)."
  echo "Restart bottom row (5-8) anytime with 'prefix + r'."
  attach_or_switch
}

case "${1:-build}" in
  build)   do_build ;;
  restart) do_restart ;;
  collect) do_collect ;;
  *) echo "usage: $0 [build|restart|collect]" >&2; exit 2 ;;
esac
