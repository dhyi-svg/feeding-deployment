#!/bin/bash
# One keypoint of a teleop demo: bring the sheppy arm node up, log the pose, release the arm again.
#   microwave/tools/kp.sh "<label>" [demo.json]
source ~/feeding-deployment/microwave/sheppy/env.sh
DEMO=${2:-microwave/demos/push_open_2026-10-07.json}
( cd microwave/sheppy && sheppy up --manifest sheppy-manifest.yaml real-arm-camera >/dev/null 2>&1 )
sleep 4
python -u microwave/tools/keypoint.py --demo "$DEMO" --label "$1" 2>&1 | grep -v -E "b3Warning|not found|inertial|IKFast|warn|terminate|Destroyable"
( cd microwave/sheppy && sheppy up --manifest sheppy-manifest.yaml camera-only >/dev/null 2>&1 )
( cd microwave/sheppy && sheppy status | head -1 )
