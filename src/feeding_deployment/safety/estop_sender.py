"""Mac-side e-stop sender (no ROS).

The single (experimentor) e-stop button is plugged into a machine that cannot
run ROS (e.g. a MacBook). This script reads the button locally with pyaudio (via
the existing `Button` class) and streams its state over UDP to
`estop_udp_bridge.py` running on the NUC, which republishes it onto the
`/experimentor_estop` ROS topic that `bulldog.py` watches.

This is the Mac equivalent of `estops_publisher.py`: same 100Hz loop, but the
sink is a UDP socket instead of a ROS publisher.

Two fail-safe couplings, both ending in an arm e-stop:

  * Forward heartbeat -- the stream of packets IS the heartbeat. If this script
    dies, the Mac sleeps, or the network drops, packets stop arriving at the
    bridge, `/experimentor_estop` goes quiet, and bulldog's "<50 msgs/sec" check
    stops the arm within ~1s. So the network link is part of the monitored
    safety heartbeat.

  * Reverse lifecycle coupling -- the bridge streams a per-launch EPOCH token
    back to us while bulldog is alive. Mirroring how run_bulldog.sh kills the
    local estops_publisher when bulldog exits, this sender SELF-TERMINATES (and
    must be manually relaunched) the moment bulldog/the bridge dies or restarts:
      - no token for ACK_TIMEOUT seconds  -> bridge/bulldog gone   -> exit
      - token with a different EPOCH       -> bridge/bulldog restarted -> exit

Run on the lab WiFi is fine (fail-safe priority: better to stop the robot on a
network glitch than risk anything unsafe), but keep the Mac awake -- macOS sleep
or App Nap freezes this loop and will trip a (fail-safe) stop. Suggested:
    caffeinate -dimsu python estop_sender.py --id <N> --host 192.168.1.3

Example (button on Mac audio device index 1, NUC at 192.168.1.3):
    python estop_sender.py --id 1 --host 192.168.1.3
"""

import argparse
import socket
import struct
import time

try:
    from feeding_deployment.safety.button import Button
except ImportError:  # allow running directly from the safety/ directory
    from button import Button

# Forward packet, Mac -> NUC (network byte order, no padding):
#   Q  uint64  monotonically increasing sequence number (debug/logging only)
#   ?  bool    experimentor e-stop pressed
PACKET_FORMAT = "!Q?"

# Reverse ack, NUC -> Mac:
#   Q  uint64  the bridge's per-launch EPOCH token
ACK_FORMAT = "!Q"
ACK_SIZE = struct.calcsize(ACK_FORMAT)

# After the FIRST ack is seen, exit if no ack arrives for this long. Generous
# relative to the ~100 acks/sec we normally get, so ordinary WiFi jitter does
# not force a relaunch, but a dead/killed bridge does within ~1.5s.
ACK_TIMEOUT_S = 1.5


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--id", type=int, required=True,
                   help="audio device index of the experimentor e-stop button")
    p.add_argument("--host", default="192.168.1.3",
                   help="NUC IP address running estop_udp_bridge.py")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--rate", type=float, default=100.0,
                   help="send rate in Hz (must keep the bridge's topic above bulldog's 50/sec threshold)")
    # Detection thresholds: quiet Mac USB dongles spike to only ~+/-200 on a
    # press, unlike the lab's NUC hardware (~+/-10000). See button.py.
    p.add_argument("--max_threshold", type=int, default=200)
    p.add_argument("--min_threshold", type=int, default=-200)
    args = p.parse_args()

    button = Button(args.id, max_threshold=args.max_threshold,
                    min_threshold=args.min_threshold)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)  # never block the send loop waiting for an ack
    dest = (args.host, args.port)
    period = 1.0 / args.rate

    print(f"E-stop sender: id={args.id}, sending to {args.host}:{args.port} "
          f"at {args.rate:.0f} Hz")
    print("Streaming... (Ctrl+C to stop). Stopping this WILL trigger an arm e-stop.")

    seq = 0
    last_report = time.monotonic()
    # Reverse-coupling state.
    first_epoch = None          # EPOCH of the first ack we ever saw
    last_ack = None             # time.monotonic() of the most recent ack
    stop_reason = None          # set when the watchdog decides to exit

    try:
        while True:
            start = time.monotonic()

            pressed = button.check()
            sock.sendto(struct.pack(PACKET_FORMAT, seq, pressed), dest)
            seq += 1

            if pressed:
                print(f"PRESS sent: experimentor={pressed} (seq={seq})")

            # Drain any pending acks (non-blocking). Each ack carries the
            # bridge's EPOCH; a new EPOCH means the bridge/bulldog restarted.
            while True:
                try:
                    data, _addr = sock.recvfrom(64)
                except (BlockingIOError, socket.error):
                    break  # nothing more to read this cycle
                if len(data) != ACK_SIZE:
                    continue
                (epoch,) = struct.unpack(ACK_FORMAT, data)
                if first_epoch is None:
                    first_epoch = epoch
                    print(f"Bridge ack received (epoch={epoch}); lifecycle coupling armed.")
                elif epoch != first_epoch:
                    stop_reason = ("bridge/bulldog RESTARTED "
                                   f"(epoch {first_epoch} -> {epoch})")
                    break
                last_ack = start

            if stop_reason is not None:
                break

            # Lifecycle watchdog: only after the first ack (grace before then,
            # e.g. while waiting for the NUC bridge to come up).
            if last_ack is not None and (start - last_ack) > ACK_TIMEOUT_S:
                stop_reason = f"no bridge ack for {ACK_TIMEOUT_S:.1f}s (bridge/bulldog gone)"
                break

            # Heartbeat sanity log, like bulldog's.
            if start - last_report >= 5.0:
                armed = "armed" if first_epoch is not None else "waiting for bridge"
                print(f"... alive, {seq} packets sent ({armed})")
                last_report = start

            time.sleep(max(0.0, period - (time.monotonic() - start)))
    except KeyboardInterrupt:
        stop_reason = "Ctrl+C"
    finally:
        if stop_reason is not None:
            print(f"\nE-stop sender stopping: {stop_reason}.")
            print("NUC bridge/bulldog gone or restarted -- relaunch estop_sender.py "
                  "after the NUC stack is back up.")
        sock.close()
        button.close()


if __name__ == "__main__":
    main()
