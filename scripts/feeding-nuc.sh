#!/usr/bin/env bash
# feeding-nuc.sh -- NUC 'robot' tmux session: arm / base / bulldog,
#                   plus a one-key (prefix + r) e-stop restart.
#
# Run this ON THE NUC (repo there: ~/feeding-deployment, user emprise).
#
#   ./feeding-nuc.sh           build the session (or attach if it exists)
#   ./feeding-nuc.sh restart   Ctrl+C all three, relaunch arm+base, bulldog +3s
#                              (this is what 'prefix + r' runs for you)
#
# Bringup order matters: arm + base must be up BEFORE bulldog -- bulldog refuses
# to start unless both RPC servers are running.
#
#   arm     -> launch_arm
#   base    -> launch_base
#   bulldog -> launch_remote_bulldog   (only after arm AND base are up)
#
# On first build nothing is auto-run: each pane has its command pre-typed and you
# press Enter top-to-bottom. The 'restart' path (prefix + r) DOES auto-fire, in
# order, for fast recovery after an e-stop.
#
# Permanence: the 'prefix + r' binding is installed at build time and lasts for
# the tmux server's life. To survive a full server restart, also add to your NUC
# ~/.tmux.conf:
#   bind R run-shell -b "~/feeding-deployment/scripts/feeding-nuc.sh restart"

set -euo pipefail

SESSION=robot
WIN=robot
TARGET="$SESSION:$WIN"

# Tunables (env-overridable).
RESTART_GRACE="${RESTART_GRACE:-2}"   # seconds after C-c before relaunch (port release)
BULLDOG_DELAY="${BULLDOG_DELAY:-3}"   # seconds after arm+base before bulldog

# Absolute path to this script, so the tmux binding can call it back.
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

# ----- session logging (NUC-local pane capture) ----------------------------- #
# Every pane (arm/base/bulldog + any manual splits) is piped to a per-run bundle
# on NUC-local disk -- no cross-network traffic during the run. Compute's
# `feeding-compute.sh collect` rsyncs this bundle in at teardown. NO_PANELOG=1
# disables. Paths are derived from this script's location so they work under the
# NUC's user/repo (emprise:~/feeding-deployment) without hardcoding.
SCRIPT_DIR="$(dirname "$SELF")"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
HELPER="$SCRIPT_DIR/tmux-pane-log.sh"
SESS_ROOT="$REPO_DIR/src/feeding_deployment/integration/log/system_logs"
SESSION_KEEP="${SESSION_KEEP:-10}"

# Fresh per-run bundle; point current_session at it; prune old bundles.
new_bundle() {
  local stamp bundle
  stamp="$(printf '%(%Y%m%d_%H%M%S)T' -1)"
  # SESSION_LABEL (forwarded by feeding_start over ssh) suffixes the bundle name.
  bundle="$SESS_ROOT/session_${stamp}${SESSION_LABEL:+_$SESSION_LABEL}"
  mkdir -p "$bundle/robot/tmux"
  printf '%(%Y-%m-%dT%H:%M:%S)T' -1 > "$bundle/.started_iso"
  # Point current_session at the new bundle. If a stray REAL dir sits there,
  # remove it first -- 'ln -sfn' onto a directory links INSIDE it, not over it.
  [[ -L "$SESS_ROOT/current_session" || ! -e "$SESS_ROOT/current_session" ]] || rm -rf "$SESS_ROOT/current_session"
  ln -sfn "$bundle" "$SESS_ROOT/current_session"
  local keep_target d
  keep_target="$(readlink -f "$SESS_ROOT/current_session" 2>/dev/null)"
  ls -dt "$SESS_ROOT"/session_* 2>/dev/null | tail -n +$((SESSION_KEEP+1)) | while read -r d; do
    [[ "$(readlink -f "$d")" == "$keep_target" ]] && continue
    rm -rf "$d"
  done
  echo "session bundle: $bundle"
}

# Global tmux hooks: pipe every pane (existing/new/manual) to the current bundle.
# Server-lifetime, like `bind r`; pipe-pane rides the pane through 'prefix + r'.
install_pane_logging() {
  [[ -n "${NO_PANELOG:-}" ]] && { echo "pane-logging: disabled (NO_PANELOG)"; return 0; }
  [[ -x "$HELPER" ]] || { echo "pane-logging: $HELPER not executable; skipping" >&2; return 0; }
  # set-hook/pipe-pane need a live session (an empty server exits immediately).
  tmux list-sessions >/dev/null 2>&1 || { echo "pane-logging: no live tmux session yet; deferring"; return 0; }
  local root="$SESS_ROOT/current_session/robot"
  # Plain pipe-pane (no -o): -o TOGGLES, so a pane hit by two hooks would open then
  # close. window-linked + after-split-window cover new sessions/windows + splits
  # with no overlap; plain pipe-pane always ends with the pipe OPEN (idempotent).
  local pipe="pipe-pane \"TMUXLOG_ROOT='$root' exec '$HELPER' '#{session_name}' '#{window_index}' '#{pane_index}' '#{pane_id}'\""
  local mark="run-shell -b \"TMUXLOG_ROOT='$root' '$HELPER' --event '#{hook}' '#{session_name}' '#{window_index}' '#{pane_index}'\""
  local ev
  for ev in window-linked after-split-window; do
    tmux set-hook -g  "$ev" "$pipe"
    tmux set-hook -ag "$ev" "$mark"
  done
  tmux set-hook -g pane-exited "run-shell -b \"TMUXLOG_ROOT='$root' '$HELPER' --event pane-exited '#{session_name}' '#{window_index}' '#{pane_index}'\""
  local p
  for p in $(tmux list-panes -a -F '#{pane_id}' 2>/dev/null); do
    tmux pipe-pane -t "$p" "TMUXLOG_ROOT='$root' exec '$HELPER' '#{session_name}' '#{window_index}' '#{pane_index}' '#{pane_id}'"
  done
  echo "pane-logging: hooks installed (-> $root/tmux)"
}

# ------------------------------------------------------------------ restart ---
do_restart() {
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "restart: session '$SESSION' not found -- run '$SELF' first." >&2
    exit 1
  fi

  # Resolve panes by POSITION (list-panes returns them in index order, which for
  # our even-vertical layout is top->bottom = arm, base, bulldog). We deliberately
  # do NOT match on pane title: programs like roscore set the terminal title and
  # overwrite our labels, which broke title-based lookup.
  local arm base bull
  mapfile -t _panes < <(tmux list-panes -t "$TARGET" -F '#{pane_id}')
  if (( ${#_panes[@]} < 3 )); then
    echo "restart: expected >=3 panes in $TARGET, found ${#_panes[@]}." >&2
    echo "  run '$SELF' to (re)build the session." >&2
    exit 1
  fi
  arm="${_panes[0]}"; base="${_panes[1]}"; bull="${_panes[2]}"

  # 1. Stop all three.
  tmux send-keys -t "$arm"  C-c
  tmux send-keys -t "$base" C-c
  tmux send-keys -t "$bull" C-c

  # 2. Let them exit and free their RPC ports.
  sleep "$RESTART_GRACE"

  # 3. Relaunch arm + base (C-u clears any partial input line first).
  tmux send-keys -t "$arm"  C-u 'launch_arm'  Enter
  tmux send-keys -t "$base" C-u 'launch_base' Enter

  # 4. Give the RPC servers a head start.
  sleep "$BULLDOG_DELAY"

  # 5. Relaunch bulldog.
  tmux send-keys -t "$bull" C-u 'launch_remote_bulldog' Enter

  tmux display-message "feeding-nuc: restarted arm+base, bulldog in ${BULLDOG_DELAY}s"
  [[ -n "${NO_PANELOG:-}" ]] || tmux run-shell -b "TMUXLOG_ROOT='$SESS_ROOT/current_session/robot' '$HELPER' --event restart '$SESSION' 0 0"
}

# -------------------------------------------------------------------- build ---
do_build() {
  # Fresh build if the 'robot' session isn't up yet.
  local fresh=1
  if tmux has-session -t "$SESSION" 2>/dev/null; then fresh=0; fi

  # New per-run bundle on a fresh build (or if the symlink vanished).
  if (( fresh )) || [[ ! -e "$SESS_ROOT/current_session" ]]; then
    new_bundle
  fi

  if (( ! fresh )); then
    install_pane_logging       # ensure hooks + pipe existing panes, then attach
    echo "session '$SESSION' already exists."
    [[ -n "${NO_ATTACH:-}" ]] && { echo "(NO_ATTACH set -- left detached)"; return 0; }
    exec tmux attach -t "$SESSION"
  fi

  # Create session + 3 stacked panes; capture pane ids (index-agnostic).
  local p_arm p_base p_bull
  p_arm="$(tmux new-session -d -s "$SESSION" -n "$WIN" -P -F '#{pane_id}')"
  p_base="$(tmux split-window -v -t "$p_arm"  -P -F '#{pane_id}')"
  p_bull="$(tmux split-window -v -t "$p_base" -P -F '#{pane_id}')"
  tmux select-layout -t "$TARGET" even-vertical

  # Labelled title bar on each pane.
  tmux set-option -w -t "$TARGET" pane-border-status top
  tmux set-option -w -t "$TARGET" pane-border-format ' #{pane_index} #{pane_title} '

  # Label + pre-type each command (no Enter -- fire them in order yourself).
  tmux select-pane -t "$p_arm"  -T arm
  tmux send-keys   -t "$p_arm"  'launch_arm'
  tmux select-pane -t "$p_base" -T base
  tmux send-keys   -t "$p_base" 'launch_base'
  tmux select-pane -t "$p_bull" -T bulldog
  tmux send-keys   -t "$p_bull" 'launch_remote_bulldog'

  tmux select-pane -t "$p_arm"

  # Install the e-stop restart hotkey: prefix + r.
  tmux bind-key r run-shell -b "$SELF restart"

  # Session is alive now: install hooks (future panes) + pipe the 3 panes.
  install_pane_logging

  echo "Built tmux session '$SESSION' (arm / base / bulldog)."
  echo "First bringup: run arm, run base, then -- once both are up -- run bulldog."
  echo "After an e-stop: press 'prefix + r' to restart all three (bulldog +${BULLDOG_DELAY}s)."
  [[ -n "${NO_ATTACH:-}" ]] && { echo "(NO_ATTACH set -- session built, left detached)"; return 0; }
  exec tmux attach -t "$SESSION"
}

case "${1:-build}" in
  build)   do_build ;;
  restart) do_restart ;;
  *) echo "usage: $0 [build|restart]" >&2; exit 2 ;;
esac
