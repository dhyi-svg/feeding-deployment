#!/usr/bin/env bash
# Bring up everything press_button needs, in the background, checking each piece. NO ARM MOTION.
#
#   ./scripts/button_press/bringup.sh            # start (skips anything already running)
#   ./scripts/button_press/bringup.sh status     # health check only
#   ./scripts/button_press/bringup.sh stop       # stop everything this script started
#
# Then:  python3 -u -m feeding_deployment.button_press.press_button --target timer_clock [--execute]
#
# What it starts (docs/button_press_runbook.md §1-3 by hand; this is the 2026-09-27 sequence):
#   arm_server -> joint_state_bridge (within 10 s, it is the Kortex keepalive) -> stub base ->
#   bulldog_bypass (motion UNLOCKED from here) -> speed low -> robot_state_publisher (gen3, no
#   gripper: robotiq_description is too old for the xacro) -> hand-eye calibration tf ->
#   RealSense (IMU off -- it reset-loops otherwise; ns / so topics are /camera/...; big-image Fast
#   DDS profile; colour AE priority off). press_button does the detection itself (dome_pattern).
# PIDs and logs: ~/press_logs/bringup/<name>.{pid,log}
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN="$HOME/press_logs/bringup"
CALIB="${CALIB:-$HOME/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib}"
mkdir -p "$RUN"

# ROS's setup.bash reads unset variables (AMENT_TRACE_SETUP_FILES, ...), so not under set -u.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_large_images.xml"
export ARM_RPC_HOST=127.0.0.1
export PYTHONPATH="$REPO/src:${PYTHONPATH:-}"     # PREPEND: PYTHONPATH=src alone drops rclpy
cd "$REPO" || exit 1
# Same env for your own terminal: `source ~/press_logs/bringup/env.sh`. Needed because
# feeding_deployment is pip-installed (editable) from ~/feeding-deployment, which has no
# button_press -- without the prepend, `python3 -m feeding_deployment.button_press...` fails.
cat >"$RUN/env.sh" <<ENV
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE
export ARM_RPC_HOST=$ARM_RPC_HOST
export PYTHONPATH=$REPO/src:\$PYTHONPATH
cd $REPO
ENV

# Order matters for stop (reverse) -- keep in sync with do_start.
NAMES=(arm_server joint_state_bridge stub_base bulldog_bypass rsp calibration_tf camera)

alive() { [[ -f "$RUN/$1.pid" ]] && kill -0 "$(cat "$RUN/$1.pid")" 2>/dev/null; }
die() { echo "FAILED: $*"; echo "logs: $RUN"; exit 1; }

start() {   # start <name> <cmd...>
    local name=$1; shift
    if alive "$name"; then echo "  $name already running (pid $(cat "$RUN/$name.pid"))"; return 0; fi
    nohup "$@" >"$RUN/$name.log" 2>&1 &
    echo $! >"$RUN/$name.pid"
    echo "  $name started (pid $!)"
}

wait_log() {   # wait_log <name> <pattern> <timeout_s>
    local t=0
    until grep -q "$2" "$RUN/$1.log" 2>/dev/null; do
        alive "$1" || die "$1 exited -- tail $RUN/$1.log"
        sleep 0.5; t=$((t + 1)); (( t > $3 * 2 )) && die "$1: no '$2' after $3 s"
    done
}

check() {
    python3 - <<'EOF'
import sys, time
import rclpy, tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
ok = True
def report(good, msg):
    global ok
    ok &= good
    print(f"  {'ok  ' if good else 'FAIL'} {msg}")
try:
    from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
    ai = ArmInterfaceClient(); s = ai.get_state()
    name = ai._arm_interface.get_arm_state()["name"]; speed = ai.get_speed(); grip = float(s["gripper_pos"])
    report("SERVOING_READY" in str(name), f"arm state {name}")
    report(str(speed).lower() == "low", f"speed {speed}")
    report(grip > 0.7, f"gripper {grip:.2f} ({'closed' if grip > 0.7 else 'OPEN -- the press needs it closed'})")
except Exception as e:  # noqa: BLE001
    report(False, f"arm RPC: {e}")
rclpy.init()
n = rclpy.create_node("bringup_check")
counts = {"js": 0, "img": 0}
n.create_subscription(JointState, "/joint_states", lambda m: counts.__setitem__("js", counts["js"] + 1), 50)
n.create_subscription(Image, "/camera/color/image_raw", lambda m: counts.__setitem__("img", counts["img"] + 1), 5)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.05)
dt = time.time() - t0
report(counts["js"] / dt > 30, f"/joint_states {counts['js'] / dt:.0f} Hz (want ~50)")
report(counts["img"] / dt > 10, f"/camera/color/image_raw {counts['img'] / dt:.1f} Hz (want ~15)")
try:
    buf.lookup_transform("arm_base_link", "camera_color_optical_frame", Time(), timeout=Duration(seconds=1.0))
    report(True, "tf arm_base_link -> camera_color_optical_frame")
except Exception as e:  # noqa: BLE001
    report(False, f"tf arm_base_link -> camera_color_optical_frame: {type(e).__name__}")
rclpy.shutdown()
sys.exit(0 if ok else 1)
EOF
}

do_start() {
    echo "== arm =="
    start arm_server python3 -u src/feeding_deployment/control/robot_controller/arm_server.py
    wait_log arm_server "Arm manager server started" 30
    start joint_state_bridge python3 -u -m feeding_deployment.ros2.joint_state_bridge
    start stub_base python3 -u scripts/stub_base_server.py
    sleep 1
    start bulldog_bypass python3 -u scripts/bulldog_bypass.py
    sleep 2
    python3 scripts/session/arm_set_speed.py low >"$RUN/arm_set_speed.log" 2>&1 || die "arm_set_speed.py low -- see $RUN/arm_set_speed.log"
    echo "  speed set low"

    echo "== tf =="
    xacro "$(ros2 pkg prefix kortex_description)/share/kortex_description/robots/gen3.xacro" dof:=7 gripper:="" \
        >"$RUN/gen3_nogripper.urdf" 2>"$RUN/xacro.err" || die "xacro -- see $RUN/xacro.err"
    start rsp ros2 run robot_state_publisher robot_state_publisher \
        --ros-args -p robot_description:="$(cat "$RUN/gen3_nogripper.urdf")"
    start calibration_tf python3 -u -m feeding_deployment.ros2.calibration_tf --calib "$CALIB"

    echo "== camera =="
    start camera ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/ -r __node:=camera \
        -p align_depth.enable:=true \
        -p rgb_camera.color_profile:=640,480,15 -p depth_module.depth_profile:=640,480,15 \
        -p enable_gyro:=false -p enable_accel:=false -p enable_motion:=false \
        -p rgb_camera.auto_exposure_priority:=false
    wait_log camera "RealSense Node Is Up" 30
    sleep 3

    echo "== check =="
    if check; then
        echo "READY. In your terminal:  source $RUN/env.sh   then dry run first:"
        echo "  python3 -u -m feeding_deployment.button_press.press_button --target timer_clock"
        echo "  (watch: python3 -u scripts/button_press/view_detection.py --target timer_clock)"
    else
        die "health check (above)"
    fi
}

do_stop() {
    for (( i=${#NAMES[@]}-1; i>=0; i-- )); do
        local name=${NAMES[$i]}
        if alive "$name"; then
            kill -INT "$(cat "$RUN/$name.pid")" 2>/dev/null && echo "  stopped $name"
            sleep 0.5
        fi
        rm -f "$RUN/$name.pid"
    done
}

case "${1:-start}" in
    start)  do_start ;;
    status) check ;;
    stop)   do_stop ;;
    *) echo "usage: $0 [start|status|stop]"; exit 2 ;;
esac
