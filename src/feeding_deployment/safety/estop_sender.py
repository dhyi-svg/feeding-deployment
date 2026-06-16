"""Mac-side e-stop sender (no ROS).

The e-stop button is plugged into a machine that cannot run ROS (e.g. a
MacBook). This script reads the button locally with pyaudio (via the existing
`Button` class) and streams its state over UDP to `estop_udp_bridge.py` running
on the NUC, which republishes it onto the `/user_estop` / `/experimentor_estop`
ROS topics that `bulldog.py` watches.

This is the Mac half of what `estops_publisher.py` does on a Linux machine:
same 100Hz loop, but the sink is a UDP socket instead of a ROS publisher.

Fail-safe: the stream of packets IS the heartbeat. If this script dies, the Mac
crashes, or the network drops, packets stop arriving at the bridge, the ROS
topic goes quiet, and bulldog's "<50 msgs/sec" check stops the arm within ~1s.
So the network link is included in the monitored safety heartbeat. Run on a
WIRED connection -- WiFi stalls/drops would cause spurious or delayed stops.

Example (one button on Mac audio device index 1, NUC at 192.168.1.3):
    python estop_sender.py --user_id 1 --host 192.168.1.3
"""

import argparse
import socket
import struct
import time

try:
    from feeding_deployment.safety.button import Button
except ImportError:  # allow running directly from the safety/ directory
    from button import Button

# Packet layout (network byte order, no padding):
#   Q  uint64  monotonically increasing sequence number (debug/logging only)
#   ?  bool    user e-stop pressed
#   ?  bool    experimentor e-stop pressed
PACKET_FORMAT = "!Q??"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--user_id", type=int, required=True,
                   help="audio device index of the user e-stop button")
    p.add_argument("--exp_id", type=int, default=None,
                   help="audio device index of the experimentor e-stop button (optional)")
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

    user_button = Button(args.user_id, max_threshold=args.max_threshold,
                         min_threshold=args.min_threshold)
    exp_button = (Button(args.exp_id, max_threshold=args.max_threshold,
                         min_threshold=args.min_threshold)
                  if args.exp_id is not None else None)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    period = 1.0 / args.rate

    print(f"E-stop sender: user_id={args.user_id}, exp_id={args.exp_id}, "
          f"sending to {args.host}:{args.port} at {args.rate:.0f} Hz")
    print("Streaming... (Ctrl+C to stop). Stopping this WILL trigger an arm e-stop.")

    seq = 0
    last_report = time.time()
    try:
        while True:
            start = time.time()

            user_pressed = user_button.check()
            exp_pressed = exp_button.check() if exp_button is not None else False

            packet = struct.pack(PACKET_FORMAT, seq, user_pressed, exp_pressed)
            sock.sendto(packet, dest)
            seq += 1

            if user_pressed or exp_pressed:
                print(f"PRESS sent: user={user_pressed} exp={exp_pressed} (seq={seq})")

            # Heartbeat sanity: confirm we're keeping rate, like bulldog logs do.
            if start - last_report >= 5.0:
                print(f"... alive, {seq} packets sent")
                last_report = start

            time.sleep(max(0.0, period - (time.time() - start)))
    except KeyboardInterrupt:
        print("\nE-stop sender stopped. (Bridge will see the heartbeat die.)")
    finally:
        sock.close()
        user_button.close()
        if exp_button is not None:
            exp_button.close()


if __name__ == "__main__":
    main()
