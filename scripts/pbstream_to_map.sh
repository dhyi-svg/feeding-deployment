#!/usr/bin/env bash
# pbstream_to_map.sh -- convert a Cartographer .pbstream into a map_server
# pgm+yaml (image + resolution + origin), so it can be used as a plot background
# by plot_nav_traces.py or loaded by map_server.
#
# The plotter normally uses the <navlog>/map.yaml that nav_diag_logger captures
# per run; this helper is for (a) generating a background for an OLD log that
# predates that capture, or (b) inspecting a map offline.
#
#   scripts/pbstream_to_map.sh maps/aimee-7-1.pbstream            # -> maps/aimee-7-1.{pgm,yaml}
#   scripts/pbstream_to_map.sh maps/aimee-7-1.pbstream out/aimee 0.05
set -euo pipefail

PB="${1:?usage: pbstream_to_map.sh <in.pbstream> [out_filestem] [resolution]}"
STEM="${2:-${PB%.pbstream}}"
RES="${3:-0.05}"
BIN="$HOME/cartographer_ws/install_isolated/bin/cartographer_pbstream_to_ros_map"

[ -f "$PB" ]  || { echo "pbstream not found: $PB" >&2; exit 1; }
[ -x "$BIN" ] || { echo "converter not found: $BIN" >&2; exit 1; }

# Bring in cartographer_ros libs if the workspace setup is present.
if [ -f "$HOME/cartographer_ws/install_isolated/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/cartographer_ws/install_isolated/setup.bash"
fi

"$BIN" -pbstream_filename "$PB" -map_filestem "$STEM" -resolution "$RES"
echo "wrote ${STEM}.pgm and ${STEM}.yaml"
