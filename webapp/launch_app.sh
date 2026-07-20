#!/usr/bin/env bash
# Launches the participant webapp (foreground, https://192.168.1.2:8080) plus
# the researcher intervention timer (background, http://192.168.1.2:8081) and
# the meal review page (background, http://192.168.1.2:8082).
# Ctrl-C tears all three down. Invoked by the `launch_app` alias in ~/.bash_aliases.
set -u

source /home/isacc/miniconda3/etc/profile.d/conda.sh
conda activate feed
source "$HOME/deployment_ws/devel/setup.bash"

REPO="$HOME/deployment_ws/src/feeding-deployment"
TIMER="$REPO/src/feeding_deployment/integration/researcher_timer.py"
REVIEW="$REPO/src/feeding_deployment/integration/review_meal.py"
# One diagnostic log per launch (~= per study session), like the session_<stamp> bundles.
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$REPO/src/feeding_deployment/integration/log/system_logs"
TIMER_LOG="$LOG_DIR/researcher_timer_$STAMP.log"
REVIEW_LOG="$LOG_DIR/review_meal_$STAMP.log"
mkdir -p "$LOG_DIR"

# A previous launch that died without cleanup leaves orphans holding :8081 / :8082.
pkill -f "python .*researcher_timer\.py" 2>/dev/null
pkill -f "python .*review_meal\.py" 2>/dev/null
sleep 1

python "$TIMER" >>"$TIMER_LOG" 2>&1 &
TIMER_PID=$!
python "$REVIEW" >>"$REVIEW_LOG" 2>&1 &
REVIEW_PID=$!
trap 'kill "$TIMER_PID" "$REVIEW_PID" 2>/dev/null' EXIT
echo "[launch_app] researcher timer: http://192.168.1.2:8081 (log: $TIMER_LOG)"
echo "[launch_app] meal review:      http://192.168.1.2:8082 (log: $REVIEW_LOG)"

cd "$REPO/webapp"
npm run serve -- --host 192.168.1.2
