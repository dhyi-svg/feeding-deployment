#!/bin/bash
#
# Remote-button variant of run_bulldog.sh.
#
# Use this when the single (experimentor) e-stop button is plugged into a machine
# that cannot run ROS (e.g. a MacBook) running estop_sender.py, which streams the
# button state to this NUC over UDP. This launches estop_udp_bridge.py (UDP -> ROS)
# in place of estops_publisher.py; the bridge republishes the button onto
# /experimentor_estop, exactly the topic bulldog watches. bulldog.py and
# arm_server.py are UNCHANGED.
#
# Prereqs on the NUC:
#   - arm_server.py already running (bulldog connects to it over RPC at :5000)
#   - ISACC_PASSWORD exported (bulldog sys.exit(1)s without it)
#   - correct conda env active (same one run_bulldog.sh uses)
#   - UDP port 5005 allowed through the firewall
# Prereq on the Mac:
#   - estop_sender.py running and reaching this NUC's IP on udp/5005

WAIT_TIMEOUT=120  # seconds to wait for the Mac sender before aborting

# Function to clean up background processes (re-entrant safe).
cleanup() {
    trap - SIGINT SIGTERM EXIT  # disarm so this runs at most once
    echo "Stopping background processes..."
    kill $roscore_pid 2>/dev/null
    if kill -0 $bridge_pid 2>/dev/null; then
        kill $bridge_pid
    fi
}

# Trap Ctrl+C, termination, and normal exit and call cleanup.
trap cleanup SIGINT SIGTERM EXIT

# Start roscore
roscore &
roscore_pid=$!  # Store the PID of roscore

# Wait to ensure roscore has time to start
sleep 2

# Run the UDP -> ROS e-stop bridge in the background (replaces estops_publisher.py).
# The physical button is read on the Mac by estop_sender.py; the bridge republishes
# it onto /experimentor_estop, exactly the topic bulldog watches.
cd ~/feeding-deployment/src/feeding_deployment/safety
python estop_udp_bridge.py &
bridge_pid=$!  # Store the PID of the bridge

sleep 2

# IMPORTANT: do NOT start bulldog until the e-stop topic is actually flowing.
# bulldog's heartbeat check trips when it sees < 50 msgs in the last second, so
# if the topic is silent at startup (e.g. the Mac sender isn't running yet) it
# emergency-stops instantly. Wait for at least one message, then let the 1s
# heartbeat window fill before launching bulldog. Abort rather than hang forever.
echo "Waiting for /experimentor_estop to go live (start estop_sender.py on the Mac if you haven't)..."
half_seconds=0
until rostopic echo -n1 /experimentor_estop >/dev/null 2>&1; do
    sleep 0.5
    half_seconds=$((half_seconds + 1))
    if [ $((half_seconds % 10)) -eq 0 ]; then
        echo "  ... still waiting for the Mac sender ($((half_seconds / 2))s elapsed)."
    fi
    if [ $((half_seconds / 2)) -ge $WAIT_TIMEOUT ]; then
        echo "ERROR: /experimentor_estop never went live after ${WAIT_TIMEOUT}s."
        echo "       Is estop_sender.py running on the Mac and reaching this NUC on udp/5005? Aborting."
        exit 1  # triggers the EXIT trap -> cleanup
    fi
done
echo "/experimentor_estop is live; letting the 1s heartbeat window fill before starting bulldog..."
sleep 2

# Run bulldog (foreground; exits on first anomaly). When it exits, the bridge is
# killed by cleanup; the bridge also self-exits when /bulldog_status goes away, and
# the Mac sender self-terminates when it stops receiving the bridge's epoch acks.
python bulldog.py

cleanup  # Ensure cleanup is called when bulldog finishes
