#!/bin/bash

SESSION="feeding_demo"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null || true

# === WINDOW 1: roscore + utensil + app + cluster (4 panes in 2x2 grid) ===
tmux new-session -d -s "$SESSION" -n "setup"

# Create 3 more panes in window 1 (total 4)
for i in {1..3}; do
    tmux split-window -d -t "$SESSION:setup"
done
tmux select-layout -t "$SESSION:setup" tiled

# === WINDOW 2: NUC arm + NUC bulldog + launch_sensors + watchdog + demo (5 panes) ===
tmux new-window -t "$SESSION" -n "demo"

# Create 4 more panes in window 2 (total 5)
for i in {1..4}; do
    tmux split-window -d -t "$SESSION:demo"
done
tmux select-layout -t "$SESSION:demo" tiled

# === Fire SSH connections first (all at once) ===
tmux send-keys -t "$SESSION:demo.0" "sshnuc" Enter
tmux send-keys -t "$SESSION:demo.1" "sshnuc" Enter
tmux send-keys -t "$SESSION:setup.3" "sshcluster" Enter

# === Start roscore (runs in parallel with SSH wait) ===
tmux send-keys -t "$SESSION:setup.0" "roscore" Enter

# Wait for SSH connections and roscore to start
sleep 6

# === Send NUC commands (SSH is now connected) ===
tmux send-keys -t "$SESSION:demo.0" "launch_arm" Enter
tmux send-keys -t "$SESSION:demo.1" "launch_bulldog" Enter

# === Start local ROS services (roscore is ready) ===
tmux send-keys -t "$SESSION:demo.2" "launch_sensors" Enter
tmux send-keys -t "$SESSION:setup.1" "launch_utensil" Enter
tmux send-keys -t "$SESSION:setup.2" "launch_app" Enter
tmux send-keys -t "$SESSION:demo.3" "launch_watchdog" Enter

# === Send cluster command ===
tmux send-keys -t "$SESSION:setup.3" "launch_molmo" Enter

# Wait for all services to initialize
sleep 8

# === Launch the demo ===
tmux send-keys -t "$SESSION:demo.4" "run_demo" Enter

# Attach to session (defaults to first window)
tmux attach-session -t "$SESSION:setup"
