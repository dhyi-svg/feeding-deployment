"""Run the resident detector. Start once per session, in its own terminal.

    DETECTION_LOG_DIR=~/captures/session $PY -u scripts/session/detection_server.py

Takes ~1-6 min to come up (model load + cold disk), then every detection costs only the
~30 s forward pass instead of another full startup. The task scripts pick it up
automatically when it is running; without it they load their own copy as before.

Needs microwave_bringup running (it subscribes to the camera). Does not touch the arm.
"""
import os
import sys

from feeding_deployment.perception.detection_service import serve

if __name__ == "__main__":
    try:
        serve(log_dir=os.environ.get("DETECTION_LOG_DIR"))
    except KeyboardInterrupt:
        print("\ndetection server stopped.")
        sys.exit(0)
