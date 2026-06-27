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
}

# -------------------------------------------------------------------- build ---
do_build() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "session '$SESSION' already exists -- attaching."
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

  echo "Built tmux session '$SESSION' (arm / base / bulldog)."
  echo "First bringup: run arm, run base, then -- once both are up -- run bulldog."
  echo "After an e-stop: press 'prefix + r' to restart all three (bulldog +${BULLDOG_DELAY}s)."
  exec tmux attach -t "$SESSION"
}

case "${1:-build}" in
  build)   do_build ;;
  restart) do_restart ;;
  *) echo "usage: $0 [build|restart]" >&2; exit 2 ;;
esac
