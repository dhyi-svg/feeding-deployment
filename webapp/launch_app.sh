#!/usr/bin/env bash
# Launches the participant webapp (foreground, https://192.168.1.2:8080) plus
# the researcher intervention timer (background, http://192.168.1.2:8081).
# Ctrl-C tears both down. Invoked by the `launch_app` alias in ~/.bash_aliases.
set -u

source /home/isacc/miniconda3/etc/profile.d/conda.sh
conda activate feed
source "$HOME/deployment_ws/devel/setup.bash"

REPO="$HOME/deployment_ws/src/feeding-deployment"
TIMER="$REPO/src/feeding_deployment/integration/researcher_timer.py"
# One diagnostic log per launch (~= per study session), like the session_<stamp> bundles.
TIMER_LOG="$REPO/src/feeding_deployment/integration/log/system_logs/researcher_timer_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$(dirname "$TIMER_LOG")"

# A previous launch that died without cleanup leaves an orphan holding :8081.
pkill -f "python .*researcher_timer\.py" 2>/dev/null && sleep 1

python "$TIMER" >>"$TIMER_LOG" 2>&1 &
TIMER_PID=$!
trap 'kill "$TIMER_PID" 2>/dev/null' EXIT
echo "[launch_app] researcher timer: http://192.168.1.2:8081 (log: $TIMER_LOG)"

cd "$REPO/webapp"
npm run serve -- --host 192.168.1.2
