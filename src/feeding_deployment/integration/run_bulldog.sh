#!/bin/bash

# Function to clean up background processes
cleanup() {
    trap - SIGINT SIGTERM EXIT  # disarm so this runs at most once
    echo "Stopping background processes..."
    kill $roscore_pid 2>/dev/null
    if kill -0 $estops_pid 2>/dev/null; then
        kill $estops_pid
    fi
    if kill -0 $bulldog_pid 2>/dev/null; then
        kill $bulldog_pid
    fi
}

# Trap Ctrl+C (and termination/normal-exit) and call cleanup
trap cleanup SIGINT SIGTERM EXIT

# Start roscore
roscore &
roscore_pid=$!  # Store the PID of roscore

# Wait for 5 seconds to ensure roscore has time to start
sleep 2

# Run estops_publisher.py (single experimentor button) in the background
cd ~/feeding-deployment/src/feeding_deployment/safety
python estops_publisher.py --id 1 &
estops_pid=$!  # Store the PID of estops_publisher

# Wait for 1 second
sleep 2

# Run bulldog 
python bulldog.py 

cleanup  # Ensure cleanup is called when bulldog finishes
wait $roscore_pid
wait $estops_pid
