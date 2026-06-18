#!/bin/bash
#
# Remote-button variant of run_bulldog.sh.
#
# Use this when the e-stop button is plugged into a machine that cannot run ROS
# (e.g. a MacBook) running estop_sender.py, which streams the button state to
# this NUC over UDP. This launches estop_udp_bridge.py (UDP -> ROS) in place of
# estops_publisher.py. bulldog.py and arm_server.py are UNCHANGED.
#
# Prereqs on the NUC:
#   - arm_server.py already running (bulldog connects to it over RPC at :5000)
#   - ISACC_PASSWORD exported (bulldog sys.exit(1)s without it)
#   - correct conda env active (same one run_bulldog.sh uses)
#   - UDP port 5005 allowed through the firewall
# Prereq on the Mac:
#   - estop_sender.py running and reaching this NUC's IP on udp/5005

# Function to clean up background processes
cleanup() {
    echo "Stopping background processes..."
    kill $roscore_pid 2>/dev/null
    if kill -0 $bridge_pid 2>/dev/null; then
        kill $bridge_pid
    fi
}

# Trap Ctrl+C and call cleanup
trap cleanup SIGINT

# Start roscore
roscore &
roscore_pid=$!  # Store the PID of roscore

# Wait to ensure roscore has time to start
sleep 2

# Run the UDP -> ROS e-stop bridge in the background (replaces estops_publisher.py).
# The physical button is read on the Mac by estop_sender.py; the bridge republishes
# it onto /user_estop and /experimentor_estop, exactly the topics bulldog watches.
cd ~/feeding-deployment/src/feeding_deployment/safety
python estop_udp_bridge.py &
bridge_pid=$!  # Store the PID of the bridge

sleep 2

# IMPORTANT: do NOT start bulldog until the e-stop topic is actually flowing.
# bulldog's heartbeat check trips when it sees < 50 msgs in the last second, so
# if the topic is silent at startup (e.g. the Mac sender isn't running yet) it
# emergency-stops instantly. Wait for at least one message, then let the 1s
# heartbeat window fill before launching bulldog.
echo "Waiting for /user_estop to go live (start estop_sender.py on the Mac if you haven't)..."
until rostopic echo -n1 /user_estop >/dev/null 2>&1; do
    sleep 0.5
done
echo "/user_estop is live; letting the 1s heartbeat window fill before starting bulldog..."
sleep 2

# Run bulldog (foreground; exits on first anomaly)
python bulldog.py

cleanup  # Ensure cleanup is called when bulldog finishes
wait $roscore_pid
wait $bridge_pid
