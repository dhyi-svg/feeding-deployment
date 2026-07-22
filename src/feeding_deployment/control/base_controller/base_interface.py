# This RPC server allows other processes (the cmd_vel bridge on the compute box,
# the teleop scripts, and bulldog) to drive the Vention base, whose Arduino is
# plugged into the NUC. It mirrors arm_interface.py so the base gets the SAME
# Bulldog-governed safety as the arm:
#   - bulldog registers a heartbeat; if bulldog dies, the base self-stops.
#   - on any anomaly / e-stop press, bulldog calls emergency_stop().
#
# In addition, this owns the single authoritative lost-command watchdog: the
# cmd_vel bridge is purely reactive (it only sends set_speeds in response to
# /cmd_vel), so if the command stream stops for ANY reason (publisher death,
# bridge crash, compute hang, network drop) we stop the base here. This is
# necessary because VentionBase re-sends the last setpoint at 1 Hz, which would
# otherwise keep the Arduino's own fresh-command watchdog (CMD_STALE_MS) fed with
# a stale nonzero speed.

import threading
import time

from multiprocess.managers import BaseManager as MPBaseManager

RPC_AUTHKEY = b"secret-key"
NUC_HOSTNAME = "192.168.1.3"
BASE_RPC_PORT = 5001
BULLDOG_HEARTBEAT_TIMEOUT = 1.0  # seconds
BASE_CMD_TIMEOUT = 0.3  # seconds; no set_speeds within this window -> stop the base


class BaseInterface:
    def __init__(self, base_instance):
        self.base = base_instance

        self.emergency_stop_active = False
        self.bulldog_ready = False
        self.last_bulldog_heartbeat = None
        self._bulldog_monitor_thread = None

        # Lost-command watchdog state. Stamped on EVERY set_speeds() call (before
        # VentionBase's internal dedup/rate-limit) so the timeout tracks the
        # command rate, not the serial-send rate.
        self.last_cmd_time = None
        self._cmd_monitor_thread = threading.Thread(target=self._cmd_monitor, daemon=True)
        self._cmd_monitor_thread.start()

    def is_alive(self):
        self.last_bulldog_heartbeat = time.time()
        return True

    def register_bulldog(self):
        self.bulldog_ready = True
        self.last_bulldog_heartbeat = time.time()
        self._bulldog_monitor_thread = threading.Thread(target=self._bulldog_monitor, daemon=True)
        self._bulldog_monitor_thread.start()
        print("Bulldog registered — base commands unlocked.")

    def _require_bulldog(self):
        assert self.bulldog_ready, "Bulldog is not running — base commands are locked"

    def _bulldog_monitor(self):
        while not self.emergency_stop_active:
            time.sleep(0.2)
            if self.last_bulldog_heartbeat is not None:
                if time.time() - self.last_bulldog_heartbeat > BULLDOG_HEARTBEAT_TIMEOUT:
                    print("ERROR: Bulldog heartbeat lost — triggering base emergency stop.")
                    if not self.emergency_stop_active:
                        self.emergency_stop()
                    break

    def _cmd_monitor(self):
        """Authoritative lost-command watchdog (see module docstring)."""
        while True:
            time.sleep(BASE_CMD_TIMEOUT / 3.0)
            if self.emergency_stop_active:
                # Already latched stopped; nothing to do.
                continue
            if self.last_cmd_time is not None:
                if time.time() - self.last_cmd_time > BASE_CMD_TIMEOUT:
                    # Transient stop: zero the setpoint so VentionBase's 1 Hz
                    # re-send pushes zeros. Resumes on the next set_speeds.
                    self.base.stop()

    def set_speeds(self, speed_a, speed_b):
        self._require_bulldog()
        if self.emergency_stop_active:
            # Latched: refuse to move. Keep the base forced to zero.
            self.base.stop()
            return
        self.last_cmd_time = time.time()
        try:
            self.base.set_speeds(int(speed_a), int(speed_b))
        except Exception as e:
            print(f"Error in set_speeds: {e}")
            raise Exception(f"Error in set_speeds: {str(e)}") from None

    def stop(self):
        """Transient stop (does NOT latch); base resumes on the next set_speeds."""
        try:
            self.base.stop()
        except Exception as e:
            print(f"Error in stop: {e}")
            raise Exception(f"Error in stop: {str(e)}") from None

    def get_encoders(self):
        """Read-only encoder snapshot (or None -- see VentionBase.read_encoders).

        Deliberately NOT bulldog-gated: reading counts is safe and useful even
        while base commands are locked. Never touches serial on this path --
        it returns the reader thread's cached snapshot, so RPC pollers (e.g.
        wheel_odom_publisher at 20 Hz) cannot perturb the command link.
        """
        try:
            return self.base.read_encoders()
        except Exception as e:
            print(f"Error in get_encoders: {e}")
            raise Exception(f"Error in get_encoders: {str(e)}") from None

    def emergency_stop(self):
        """Latched stop. Stays in effect until base_server is restarted."""
        if self.emergency_stop_active:
            # Idempotent: bulldog may detect the anomaly while the heartbeat
            # monitor also fires. Keep the base stopped and return quietly.
            self.base.stop()
            return
        self.emergency_stop_active = True
        try:
            self.base.stop()
        except Exception as e:
            print(f"Error in emergency_stop: {e}")
            raise Exception(f"Error in emergency_stop: {str(e)}") from None
        print("Base emergency stop activated, will not take any more commands.")

    def close(self):
        print("Close base command received")
        try:
            self.base.disconnect()
            print("Base disconnected")
        except Exception as e:
            print(f"Error in close: {e}")
            raise Exception(f"Error in close: {str(e)}") from None


class BaseManager(MPBaseManager):
    pass
