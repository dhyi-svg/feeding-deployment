#!/usr/bin/env bash
# tmux-pane-log.sh -- pane-output logger for tmux `pipe-pane`, shared by
#                     feeding-compute.sh and feeding-nuc.sh.
#
# Captures everything a pane displays (typed commands AND their output),
# strips ANSI/OSC/CR escapes, prefixes each line with an ISO-8601 timestamp,
# and appends to a per-pane file. Output dir comes from $TMUXLOG_ROOT (the
# per-run bundle's compute/ or robot/ dir), so the persistent ~/.tmux.conf
# hooks can point at the stable `current_session` symlink without ever baking
# a timestamp into the config.
#
# Two modes:
#   Stream (default), fed by `pipe-pane`, pane bytes on stdin:
#       tmux-pane-log.sh <session> <win_idx> <pane_idx> <pane_id>
#   Event marker (one-shot, no stdin):
#       tmux-pane-log.sh --event <what> <session> <win_idx> <pane_idx>
#
# Timestamps use the bash printf builtin (%()T) -- no fork per line. ANSI is
# stripped with GNU `sed -u` (unbuffered). No gawk/ts dependency.

set -u

# Resolve output dir. Default keeps a stray pipe from dying if a hook forgot to
# set TMUXLOG_ROOT; the normal path is .../system_logs/current_session/<host>.
ROOT="${TMUXLOG_ROOT:-$HOME/tmux_pane_logs}"
dir="$ROOT/tmux"
mkdir -p "$dir" 2>/dev/null || { dir="$HOME/tmux_pane_logs_fallback"; mkdir -p "$dir"; }

# ---- event-marker mode: append a line to the shared tmux timeline ---------- #
if [[ "${1:-}" == "--event" ]]; then
  # $2=what  $3=session  $4=win_idx  $5=pane_idx
  printf '%(%Y-%m-%dT%H:%M:%S)T  %-16s %s:%s.%s\n' -1 "${2:-?}" "${3:-?}" "${4:-?}" "${5:-}" \
    >> "$dir/events.log"
  exit 0
fi

# ---- stream mode: one file per pane ---------------------------------------- #
# Filename keeps geometry (win.pane) for readability plus the stable pane id.
f="$dir/${1}_w${2}.p${3}_${4//%/}.log"
printf '%(%Y-%m-%dT%H:%M:%S)T  ==== pipe attached (%s:%s.%s %s) ====\n' -1 "$1" "$2" "$3" "$4" >> "$f"

# Strip: CSI (colors/cursor, ends in a letter), OSC (title, ends in BEL),
# charset-select, and bare CRs; then timestamp each line. Line-buffered.
sed -u -E 's/\x1b\[[0-9;?]*[a-zA-Z]//g; s/\x1b\][^\x07]*\x07//g; s/\x1b[()][AB0]//g; s/\r//g' \
  | while IFS= read -r line; do
      printf '%(%Y-%m-%dT%H:%M:%S)T  %s\n' -1 "$line"
    done >> "$f"
