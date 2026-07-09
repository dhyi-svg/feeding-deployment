#!/usr/bin/env python3
"""
PC -> Arduino speed bridge (maintains your class-style layout).

Instead of using the Basicmicro Python library, this script sends commands to an Arduino
over USB serial. The Arduino then forwards the speeds to two motor drivers:

- Driver A (TX=11 RX=10) gets speed A
- Driver B (TX=9  RX=8 ) gets speed B

Protocol sent to Arduino (one line):
    A=<int> B=<int>\n
Example:
    A=-300 B=200

Your Arduino sketch must parse that format and set M1=M2 per driver accordingly
(which matches the working Arduino code you already have).

Usage examples:
    python3 pc_to_arduino_dual.py --port "/dev/serial/by-id/usb-Arduino_..." --speedA -300 --speedB 200 --run_s 5
    python3 pc_to_arduino_dual.py --port "/dev/serial/by-id/usb-Arduino_..." --mode rotate --w 150 --run_s 5
"""

import time
import logging
import argparse
import errno
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import serial
import serial.tools.list_ports


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


# ---- Defaults (override via CLI) ----
ARDUINO_PORT_DEFAULT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00"
ARDUINO_BAUD_DEFAULT = 115200


@dataclass
class ABCommand:
    a: int
    b: int

    def to_line(self) -> bytes:
        return f"A={int(self.a)} B={int(self.b)}\n".encode("utf-8")


class ArduinoSerialBridge:
    """
    Owns the single USB serial connection to Arduino and provides:
    - connect/disconnect/reconnect
    - safe_call() wrapper for robust retry on EIO / timeouts
    - send_ab(a, b)
    - a passive reader thread consuming the firmware's output stream:
        * "E <millis> <a1> <a2> <b1> <b2> <okA> <okB>"  -- encoder snapshot
          (firmware v7+; get_encoders() serves the latest one)
        * "Parsed A=<a> B=<b>"  -- per-accepted-command echo, used by
          VentionBase's echo-confirm re-send (a SoftwareSerial encoder read
          on the Arduino can mangle an inbound command line; the missing echo
          is how we detect that and re-send)
        * "WARN ..."/"ERROR ..." -- firmware diagnostics, logged (throttled)
          and counted. With v6 these were silently discarded.
      The reader NEVER reconnects -- on any serial error it backs off and
      re-attaches to whatever self.ser currently is; reconnects belong to
      safe_call() alone (two reconnecting threads would fight over the port
      and DTR-reset the Arduino twice).
    """

    def __init__(self, port_id: str, baud: int = 115200):
        self.port_id = port_id
        self.baud = baud
        self.ser: Optional[serial.Serial] = None

        # Reader-thread state (all guarded by _state_lock).
        self._state_lock = threading.Lock()
        self._enc: Optional[Dict[str, Any]] = None  # latest E-line snapshot
        self._enc_prev_millis: Optional[int] = None
        self._resets = 0  # banner sightings + firmware-millis regressions
        self._last_echo: Optional[Tuple[int, int]] = None
        self._warn_unparseable = 0
        self._warn_send_fail = 0
        self._last_warn_log_wall = 0.0

        self.connection_status = self.connect()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="arduino-reader", daemon=True
        )
        self._reader_thread.start()

    def _open(self) -> serial.Serial:
        s = serial.Serial(
            self.port_id,
            self.baud,
            timeout=0.2,
            write_timeout=0.2,
        )
        # Arduino resets on open; allow bootloader/firmware to start
        time.sleep(2.0)
        return s

    def connect(self) -> bool:
        logger.info(f"Connecting to Arduino at {self.port_id}, baud={self.baud}")
        try:
            try:
                self.disconnect()
            except Exception:
                pass

            self.ser = self._open()
            logger.info(f"Connected to Arduino on {self.port_id}")
            return True
        except Exception as e:
            logger.error(f"Connection error on {self.port_id}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None
            logger.info(f"Disconnected from Arduino ({self.port_id})")

    def reconnect(self, attempts: int = 10, delay_s: float = 0.25) -> bool:
        for k in range(1, attempts + 1):
            logger.warning(f"[{self.port_id}] Reconnecting attempt {k}/{attempts}...")
            if self.connect():
                return True
            time.sleep(delay_s)
        logger.error(f"[{self.port_id}] Failed to reconnect after {attempts} attempts")
        return False

    def safe_call(self, fn, *args, retries: int = 1, reconnect_attempts: int = 10, **kwargs):
        """
        Execute a call; on USB EIO (Errno 5) or write timeout, reconnect and retry.
        """
        last_exc = None
        for _attempt in range(retries + 1):
            try:
                return fn(*args, **kwargs)
            except OSError as e:
                last_exc = e
                if getattr(e, "errno", None) == errno.EIO:  # Errno 5
                    logger.warning(f"[{self.port_id}] USB EIO (Errno 5) in {fn.__name__}: {e}")
                    if not self.reconnect(attempts=reconnect_attempts):
                        raise
                    continue
                raise
            except (serial.SerialTimeoutException, serial.serialutil.SerialTimeoutException) as e:
                last_exc = e
                logger.warning(f"[{self.port_id}] Write-timeout in {fn.__name__}: {e}")
                if not self.reconnect(attempts=reconnect_attempts):
                    raise
                continue
            except Exception as e:
                # Some drivers throw generic exceptions containing "timeout"
                last_exc = e
                msg = str(e).lower()
                if "timeout" in msg:
                    logger.warning(f"[{self.port_id}] Timeout in {fn.__name__}: {e}")
                    if not self.reconnect(attempts=reconnect_attempts):
                        raise
                    continue
                raise
        raise last_exc

    # ---- reader thread (passive; never reconnects) ----
    def _reader_loop(self):
        while True:
            ser = self.ser
            if ser is None:
                time.sleep(0.2)
                continue
            try:
                line = ser.readline()  # blocks up to the port timeout (0.2 s)
            except Exception:
                # Port closed/swapped mid-read (reconnect owns recovery).
                time.sleep(0.2)
                continue
            if not line:
                continue
            if not line.endswith(b"\n"):
                # readline() returns partial fragments on timeout; a truncated
                # numeric field can still parse "successfully" -- drop it.
                continue
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                try:
                    self._handle_line(text)
                except Exception as e:
                    logger.debug(f"Arduino reader: bad line {text!r}: {e}")

    def _handle_line(self, text: str):
        if text.startswith("E "):
            parts = text.split()
            if len(parts) != 8:
                return
            try:
                millis = int(parts[1])
                a1, a2, b1, b2 = (int(p) for p in parts[2:6])
                ok_a, ok_b = parts[6] == "1", parts[7] == "1"
            except ValueError:
                return
            with self._state_lock:
                # Firmware millis going backward == Arduino rebooted (DTR reset
                # or power cycle): counts restarted at 0, consumers must
                # re-baseline instead of integrating the jump as motion.
                if self._enc_prev_millis is not None and millis < self._enc_prev_millis:
                    self._resets += 1
                self._enc_prev_millis = millis
                self._enc = {
                    "millis": millis,
                    "a1": a1, "a2": a2, "b1": b1, "b2": b2,
                    "ok_a": ok_a, "ok_b": ok_b,
                    "wall": time.time(),
                }
        elif text.startswith("Parsed A="):
            try:
                a = int(text.split("A=", 1)[1].split()[0])
                b = int(text.split("B=", 1)[1].split()[0])
            except (IndexError, ValueError):
                return
            with self._state_lock:
                self._last_echo = (a, b)
        elif text.startswith("Ready"):
            with self._state_lock:
                self._resets += 1
                self._enc_prev_millis = None
                self._last_echo = None
            logger.info(f"[{self.port_id}] Arduino banner: {text}")
        elif text.startswith("WARN unparseable"):
            with self._state_lock:
                self._warn_unparseable += 1
                n = self._warn_unparseable
            self._log_firmware_warn(f"{text} (unparseable total={n})")
        elif text.startswith("WARN") or text.startswith("ERROR"):
            with self._state_lock:
                self._warn_send_fail += 1
                n = self._warn_send_fail
            self._log_firmware_warn(f"{text} (send-fail total={n})")
        else:
            logger.debug(f"[{self.port_id}] Arduino: {text}")

    def _log_firmware_warn(self, msg: str):
        """Throttled: at most one firmware WARN/ERROR log per 2 s (a dead
        controller emits several per send attempt); counters keep the truth."""
        now = time.time()
        if now - self._last_warn_log_wall >= 2.0:
            self._last_warn_log_wall = now
            logger.warning(f"[{self.port_id}] firmware: {msg}")

    def get_encoders(self, max_age_s: float = 1.0) -> Optional[Dict[str, Any]]:
        """Latest encoder snapshot, or None if never seen / older than
        max_age_s (v6 firmware, serial down, or stream stalled)."""
        with self._state_lock:
            if self._enc is None:
                return None
            snap = dict(self._enc)
            snap["resets"] = self._resets
            snap["warn_unparseable"] = self._warn_unparseable
            snap["warn_send_fail"] = self._warn_send_fail
        age = time.time() - snap.pop("wall")
        if age > max_age_s:
            return None
        snap["age_s"] = max(0.0, age)
        return snap

    def encoders_fresh(self, max_age_s: float = 1.0) -> bool:
        with self._state_lock:
            return self._enc is not None and (time.time() - self._enc["wall"]) <= max_age_s

    def get_last_echo(self) -> Optional[Tuple[int, int]]:
        with self._state_lock:
            return self._last_echo

    # ---- low-level send ----
    def _write_line(self, payload: bytes):
        if self.ser is None:
            raise RuntimeError("Arduino serial not connected")
        # NOTE: no reset_input_buffer() here anymore -- the reader thread
        # consumes the firmware's output stream (E lines, echoes, WARNs).
        self.ser.write(payload)
        self.ser.flush()

    def send_ab(self, a: int, b: int):
        cmd = ABCommand(a=a, b=b)
        self.safe_call(self._write_line, cmd.to_line(), retries=2)


class VentionMotorDriverChannel:
    """
    Represents ONE motor driver channel (A or B), but sends via the shared Arduino bridge.
    It keeps a local setpoint and delegates combined (A,B) sends through the base object.
    """

    def __init__(self, channel: str, base: "VentionBase"):
        assert channel in ("A", "B")
        self.channel = channel
        self.base = base

    def speed_control(self, motor1_speed: int, motor2_speed: int, reset_encoders: bool = False):
        """
        Keep your old signature. Your requirement says both motors must have same speed,
        so we enforce that here.
        """
        if motor1_speed != motor2_speed:
            raise ValueError(
                f"Driver {self.channel}: motor1_speed must equal motor2_speed "
                f"(got {motor1_speed} vs {motor2_speed})."
            )
        self.set_speed(motor1_speed)

    def set_speed(self, speed: int):
        # Mutate + send under the base's send lock (see VentionBase motion
        # commands) so this single-field update can't tear against a full
        # (a, b) write from another thread.
        with self.base._send_lock:
            if self.channel == "A":
                self.base._setpoints.a = int(speed)
            else:
                self.base._setpoints.b = int(speed)
            self.base._send_setpoints()

    def stop(self):
        self.set_speed(0)


class VentionBase:
    """
    Maintains your class organization:
    - base_r and base_l exist (now they map to Driver A and Driver B)
    - translate/rotate keep same semantics as your old code
    - robust reconnect is handled in ArduinoSerialBridge.safe_call()
    """

    def __init__(self, port_id: str, baud: int = 115200):
        self.bridge = ArduinoSerialBridge(port_id, baud)
        self._setpoints = ABCommand(a=0, b=0)

        # Sends arrive from several threads (RPC per-connection threads,
        # BaseInterface._cmd_monitor, bulldog): serialize the check-then-act on
        # _last_sent, the (a, b) setpoint mutation, and the serial write.
        # Created before anything that could touch it (incl. the channels'
        # set_speed) so it always exists even on a failed connection.
        self._send_lock = threading.RLock()
        self._running = False   # keepalive-loop guard; set True after connect

        # Keep your naming: base_r/base_l
        # Here: base_r -> Driver A, base_l -> Driver B
        self.base_r = VentionMotorDriverChannel("A", self)
        self.base_l = VentionMotorDriverChannel("B", self)

        if not self.bridge.connection_status:
            logger.error("Failed to connect to Arduino. Exiting.")
            return
        logger.info("Successfully connected to Arduino speed bridge.")

        self._last_sent = None
        self._last_sent_time = 0.0
        # Confirmed + unchanged: slow keepalive (feeds the firmware CMD_STALE_MS
        # watchdog) -- see _send_setpoints.
        self._min_same_send_period = 1.0 / 5.0  # 5 Hz
        # Unconfirmed (v7 mangled the command during a SoftwareSerial encoder
        # read): re-send this fast until the firmware echoes it.
        self._echo_resend_after = 0.03  # s
        self._resend_warned_at = 0.0    # throttle the echo-missing log
        # Send the initial (stopped) setpoint, then start the persistent-retry
        # keepalive loop (drives _send_setpoints so a mangled start/stop is
        # re-sent until echoed even with no incoming command stream).
        self._send_setpoints()
        self._running = True
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, name="arduino-keepalive", daemon=True)
        self._keepalive_thread.start()

    def _send_setpoints(self):
        """Send the current setpoint, with persistent echo-confirm retry.

        The v7 firmware mangles inbound commands during its SoftwareSerial
        encoder reads, so a single send (start OR stop) is often dropped. Called
        by set_speeds/translate/rotate AND by the background keepalive loop, this:
          * sends a CHANGED setpoint immediately (no rate-limit drop);
          * if the firmware hasn't echoed the current setpoint (mangled),
            re-sends every _echo_resend_after until it does;
          * once echoed (or if the v7 echo stream isn't fresh), sends a slow
            same-setpoint keepalive to feed the firmware CMD_STALE_MS watchdog.
        """
        with self._send_lock:
            now = time.time()
            ab = (self._setpoints.a, self._setpoints.b)
            changed = ab != self._last_sent
            # Echo-confirm is only meaningful while the v7 "Parsed" echoes are
            # streaming; if not fresh, treat as confirmed (fall to keepalive).
            confirmed = (not self.bridge.encoders_fresh(1.0)
                         or self.bridge.get_last_echo() == ab)

            if changed:
                pass                                    # always send a change now
            elif not confirmed:
                if now - self._last_sent_time < self._echo_resend_after:
                    return                              # retrying, bounded
                if now - self._resend_warned_at >= 1.0:
                    self._resend_warned_at = now
                    logger.warning("[Arduino] echo missing for A=%d B=%d -- "
                                   "re-sending until echoed", ab[0], ab[1])
            else:
                if now - self._last_sent_time < self._min_same_send_period:
                    return                              # confirmed: slow keepalive

            self.bridge.send_ab(ab[0], ab[1])
            self._last_sent = ab
            self._last_sent_time = now

    def _keepalive_loop(self):
        """Drive _send_setpoints continuously (~50 Hz) so a mangled start/stop
        is re-sent until echoed, then held at the keepalive rate -- independent
        of any incoming command stream. _send_setpoints self-limits the actual
        serial rate; this only ticks it."""
        while self._running:
            time.sleep(1.0 / 50.0)
            try:
                self._send_setpoints()
            except Exception:
                pass   # transient serial errors handled in safe_call/reader
    # --- motion commands (same idea as your old code) ---
    # The (a, b) pair is mutated UNDER _send_lock so a concurrent writer
    # (e.g. an RPC set_speeds racing BaseInterface._cmd_monitor's stop) cannot
    # observe or send a torn half-and-half pair (a pivot). _send_setpoints
    # re-acquires the same RLock, so nesting is fine.
    def translate(self, linear_speed: int):
        """
        '+' forward, '-' backward.
        Both drivers same sign for translation.
        """
        with self._send_lock:
            self._setpoints.a = int(linear_speed)
            self._setpoints.b = int(linear_speed)
            self._send_setpoints()

    def rotate(self, angular_speed: int):
        """
        '+' CCW, '-' CW (in-place).
        Driver A gets +w, Driver B gets -w (matches your old logic).
        """
        with self._send_lock:
            self._setpoints.a = int(angular_speed)
            self._setpoints.b = int(-angular_speed)
            self._send_setpoints()

    def set_speeds(self, speed_a: int, speed_b: int):
        """
        Directly set different speeds to the two motor drivers.
        """
        with self._send_lock:
            self._setpoints.a = int(speed_a)
            self._setpoints.b = int(speed_b)
            self._send_setpoints()

    def stop(self):
        try:
            self.set_speeds(0, 0)
        except Exception as e:
            logger.warning(f"Stop failed (ignored): {e}")

    def disconnect(self):
        self._running = False   # stop the keepalive loop before closing the port
        self.stop()
        self.bridge.disconnect()

    def reconnect(self):
        return self.bridge.reconnect()

    def read_encoders(self) -> Optional[Dict[str, Any]]:
        """
        Latest encoder snapshot streamed by firmware v7 ("E ..." lines), or
        None if there is no fresh data (v6 firmware, serial down, or stream
        stalled >1 s).

        Dict keys: millis (firmware clock, ms), a1/a2/b1/b2 (uint32 counts,
        driver A = right pair, driver B = left pair; wrap-aware deltas are the
        consumer's job), ok_a/ok_b (that side's last read succeeded -- False
        also while a dead controller is being retried at 1 Hz), age_s
        (NUC-side staleness; never compare millis across machines), resets
        (Arduino reboot counter -- on change, re-baseline counts), and
        warn_unparseable / warn_send_fail firmware diagnostics counters.
        """
        return self.bridge.get_encoders(max_age_s=1.0)


def list_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(f"{p.device} | {p.description} | {p.hwid}")


def main():
    parser = argparse.ArgumentParser(description="Send different speeds to two motor drivers via Arduino bridge.")
    parser.add_argument("--port", type=str, default=ARDUINO_PORT_DEFAULT,
                        help="Arduino port (stable): /dev/serial/by-id/... or /dev/serial/by-path/...")
    parser.add_argument("--baud", type=int, default=ARDUINO_BAUD_DEFAULT, help="Arduino baud (default: 115200)")
    parser.add_argument("--list_ports", action="store_true", help="List available serial ports and exit.")

    # Direct speed mode
    parser.add_argument("--speedA", type=int, default=0, help="Driver A speed (counts/sec)")
    parser.add_argument("--speedB", type=int, default=0, help="Driver B speed (counts/sec)")

    # Convenience motion modes (optional)
    parser.add_argument("--mode", choices=["direct", "translate", "rotate"], default="direct")
    parser.add_argument("--v", type=int, default=0, help="Translate speed for mode=translate")
    parser.add_argument("--w", type=int, default=0, help="Rotate speed for mode=rotate")

    parser.add_argument("--run_s", type=float, default=5.0, help="Run duration seconds")
    parser.add_argument("--hz", type=float, default=10.0, help="Send rate (Hz) (default: 10)")

    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    if "REPLACE_ME" in args.port:
        logger.error("Set --port to your Arduino by-id/path. Try: ls -l /dev/serial/by-id/")
        return

    base = VentionBase(args.port, args.baud)
    if not base.bridge.connection_status:
        return

    try:
        dt = 1.0 / max(args.hz, 1e-6)
        start = time.time()

        while time.time() - start < args.run_s:
            if args.mode == "direct":
                base.set_speeds(args.speedA, args.speedB)
            elif args.mode == "translate":
                base.translate(args.v)
            elif args.mode == "rotate":
                base.rotate(args.w)

            time.sleep(dt)

        base.stop()

    finally:
        base.disconnect()


if __name__ == "__main__":
    main()