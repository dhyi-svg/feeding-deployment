#!/usr/bin/env bash
# preflight_check.sh -- pre-meal recording + peripherals check.
#
# Run AFTER sensors/app/watchdog are up, BEFORE run.py (the interactive
# self-test needs the Feather LED serial port, which run.py's
# PerceptionInterface owns once it starts -- see misc/startup_selftest.py).
#
# Part 1 (automated, ~1 min): disk, NUC clock offset, sensor topic rates,
#   SVO recording growth.
# Part 2 (interactive): chains into misc/startup_selftest.py
#   (speaker / transfer button / LED / molmo). Extra args pass through,
#   e.g.:  ./preflight_check.sh --skip button
#
# Exit code 0 only if BOTH parts pass.

set -uo pipefail

INTEGRATION_DIR="$HOME/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration"
MISC_DIR="$HOME/deployment_ws/src/feeding-deployment/src/feeding_deployment/misc"
BAG_ROOT="${BAG_ROOT:-$INTEGRATION_DIR/log/bags}"
SVO_DIR="${SVO_DIR:-$INTEGRATION_DIR/log/svo}"
MIN_FREE_GB="${MIN_FREE_GB:-150}"
NUC_SSH="${NUC_SSH:-emprise@192.168.1.3}"
CLOCK_WARN_S="${CLOCK_WARN_S:-0.1}"
HZ_SECS="${HZ_SECS:-8}"            # sample window per topic-rate check
RATE_TOL="${RATE_TOL:-0.30}"       # +/-30%

FAILS=0
warn_or_fail() { echo "  [$1] $2"; [[ "$1" == FAIL ]] && FAILS=$((FAILS+1)); }
hr() { printf -- '-%.0s' {1..68}; echo; }

hr; echo "PRE-FLIGHT part 1: automated recording checks"

# 1. ROS master ---------------------------------------------------------------
if rostopic list >/dev/null 2>&1; then
  warn_or_fail PASS "ROS master reachable"
else
  warn_or_fail FAIL "no ROS master -- bring up roscore/sensors first"
  echo "aborting remaining ROS checks."; exit 1
fi

# 2. disk free ----------------------------------------------------------------
mkdir -p "$BAG_ROOT"
AVAIL="$(df -BG --output=avail "$BAG_ROOT" 2>/dev/null | tail -1 | tr -dc '0-9')"
if [[ -n "$AVAIL" && "$AVAIL" -ge "$MIN_FREE_GB" ]]; then
  warn_or_fail PASS "disk: ${AVAIL} GB free on bag filesystem (need >= ${MIN_FREE_GB})"
else
  warn_or_fail FAIL "disk: only ${AVAIL:-?} GB free (need >= ${MIN_FREE_GB}) -- offload/sweep first"
fi

# 3. NUC clock offset ---------------------------------------------------------
T0="$(date +%s.%N)"
REMOTE="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$NUC_SSH" 'date +%s.%N' 2>/dev/null || true)"
T1="$(date +%s.%N)"
if [[ -n "$REMOTE" ]]; then
  read -r OFFSET RTT < <(python3 -c "
t0,t1,r=$T0,$T1,$REMOTE
print(abs(r-(t0+t1)/2.0), t1-t0)")
  if python3 -c "exit(0 if $OFFSET <= $CLOCK_WARN_S else 1)"; then
    warn_or_fail PASS "NUC clock offset ~$(printf '%.0f' "$(python3 -c "print($OFFSET*1000)")") ms (rtt $(printf '%.0f' "$(python3 -c "print($RTT*1000)")") ms)"
  else
    warn_or_fail FAIL "NUC clock offset ~$(python3 -c "print(round($OFFSET,3))") s > ${CLOCK_WARN_S}s -- forque stamps will skew; sync clocks (chrony)"
  fi
else
  warn_or_fail FAIL "could not reach NUC ($NUC_SSH) for clock check"
fi

# 4. topic rates --------------------------------------------------------------
# name expected_hz   (checked to +/-RATE_TOL; estop is a floor: bulldog trips <50)
RATE_CHECKS=(
  "/camera/color/image_raw/compressed 15"
  "/camera/aligned_depth_to_color/image_raw/compressedDepth 15"
  "/forque/forqueSensor 1000"
  "/zed_mini/zed_node/imu/data 200"
  "/lidar_l/scan 8"
  "/lidar_r/scan 8"
  "/wheel_odom 10"
  "/odometry/fused_imu_wheel 20"
)
check_rate() {   # <topic> <expected>
  local topic="$1" want="$2" out rate
  out="$(timeout -s INT -k 3 "$HZ_SECS" rostopic hz -w 20 "$topic" 2>/dev/null || true)"
  rate="$(printf '%s' "$out" | grep -oP 'average rate: \K[0-9.]+' | tail -1)"
  if [[ -z "$rate" ]]; then
    warn_or_fail FAIL "$topic : NO MESSAGES (expected ~${want} Hz)"
    return
  fi
  if python3 -c "exit(0 if abs($rate-$want)/$want <= $RATE_TOL else 1)"; then
    warn_or_fail PASS "$topic : ${rate} Hz (expect ~${want})"
  else
    warn_or_fail FAIL "$topic : ${rate} Hz OUT OF RANGE (expect ~${want} +/-30%)"
  fi
}
for rc in "${RATE_CHECKS[@]}"; do check_rate $rc; done

# e-stop heartbeat: floor check (bulldog e-stops below 50/s)
ESTOP_OUT="$(timeout -s INT -k 3 "$HZ_SECS" rostopic hz -w 40 /experimentor_estop 2>/dev/null || true)"
ESTOP_RATE="$(printf '%s' "$ESTOP_OUT" | grep -oP 'average rate: \K[0-9.]+' | tail -1)"
if [[ -n "$ESTOP_RATE" ]] && python3 -c "exit(0 if $ESTOP_RATE >= 50 else 1)"; then
  warn_or_fail PASS "/experimentor_estop : ${ESTOP_RATE} Hz (floor 50)"
else
  warn_or_fail FAIL "/experimentor_estop : ${ESTOP_RATE:-none} Hz -- e-stop heartbeat missing/slow (Mac sender? UDP bridge?)"
fi

# 5. SVO recording growth -----------------------------------------------------
SVO="$(ls -t "$SVO_DIR"/*.svo2 2>/dev/null | head -1)"
if [[ -z "$SVO" ]]; then
  warn_or_fail FAIL "no SVO file under $SVO_DIR -- zed_svo_recorder not recording?"
else
  A="$(stat -c %s "$SVO")"; sleep 5; B="$(stat -c %s "$SVO")"
  if (( B > A )); then
    warn_or_fail PASS "SVO growing: $(basename "$SVO") (+$(( (B-A)/1024 )) KB / 5s)"
  else
    warn_or_fail FAIL "SVO NOT growing: $(basename "$SVO") -- ZED grab loop dead?"
  fi
fi

hr
if (( FAILS == 0 )); then echo "part 1: ALL CHECKS PASSED"
else echo "part 1: $FAILS CHECK(S) FAILED -- fix before recording a meal"; fi
hr

# Part 2: interactive peripherals self-test -----------------------------------
echo "PRE-FLIGHT part 2: interactive self-test (speaker/button/LED/molmo)"
echo "(run BEFORE run.py -- the LED serial port must be free)"
python "$MISC_DIR/startup_selftest.py" "$@"
SELFTEST_RC=$?

hr
if (( FAILS == 0 && SELFTEST_RC == 0 )); then
  echo "PRE-FLIGHT: PASS (recording checks + self-test)"; exit 0
else
  echo "PRE-FLIGHT: FAIL (automated fails: $FAILS, selftest rc: $SELFTEST_RC)"; exit 1
fi
