"""NUC-side e-stop bridge: UDP -> ROS.

This is the NUC half of what `estops_publisher.py` does on a Linux machine with
a locally-plugged button. Here the single (experimentor) e-stop button lives on
a Mac that cannot run ROS; `estop_sender.py` on the Mac streams the button state
to us over UDP, and we republish it onto the ROS topic `bulldog.py` watches:
`/experimentor_estop`.

Launched by `run_bulldog_remote.sh` in place of `estops_publisher.py`. `bulldog.py`
and `arm_server.py` need NO changes.

Forward fail-safe -- ONE received packet = ONE published message:
  * Normal flow: Mac sends ~100 packets/sec -> ~100 msgs/sec on /experimentor_estop,
    well above bulldog's 50/sec threshold -> healthy.
  * Mac dies / app crashes / cable unplugged / WiFi drops: packets stop, we publish
    nothing, the topic goes quiet, and bulldog's "<50 msgs/sec" frequency check stops
    the arm within ~1s. The network link is thus part of the monitored heartbeat.

Reverse lifecycle coupling -- mirrors how run_bulldog.sh kills the local
estops_publisher when bulldog exits. We mint a per-launch random EPOCH token and
stream it back to the Mac, but ONLY while bulldog is actually alive (judged from
the `/bulldog_status` topic bulldog publishes every cycle). The instant bulldog
dies (anomaly, Ctrl+C, or arm-server `kinova.py` death -> bulldog's is_alive RPC
raises and it exits), `/bulldog_status` goes stale/False, we stop acking and exit
ourselves. The Mac sender, seeing the acks stop (or a NEW epoch on restart),
self-terminates and must be manually relaunched. This fires regardless of how
bulldog dies, so it does not depend on the launch script's signal trap.
"""

import os
import socket
import struct

import rospy
from std_msgs.msg import Bool

# Forward packet, Mac -> NUC. Must match estop_sender.py.
PACKET_FORMAT = "!Q?"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# Reverse ack, NUC -> Mac: the per-launch EPOCH token.
ACK_FORMAT = "!Q"

UDP_PORT = 5005
# If no packet arrives within this window, recvfrom() returns and we loop again
# (publishing nothing -> topic rate falls -> bulldog trips). Keep it short
# relative to bulldog's ~1s window so a genuine stop is fast, and short enough
# that we re-check bulldog liveness frequently even when packets stall.
RECV_TIMEOUT_S = 0.05

# bulldog publishes /bulldog_status every cycle (~1000Hz). If we have not seen a
# fresh True within this window, treat bulldog as dead and exit. Generous.
BULLDOG_STATUS_TIMEOUT_S = 0.5


class EStopUDPBridge:
    def __init__(self, port: int = UDP_PORT, timeout_s: float = RECV_TIMEOUT_S) -> None:
        self.exp_pub = rospy.Publisher("/experimentor_estop", Bool, queue_size=1)

        # Per-launch identity. A restarted bridge gets a new epoch, which the Mac
        # sender detects and refuses to keep running against (forces relaunch).
        self.epoch = int.from_bytes(os.urandom(8), "big")
        self.ack = struct.pack(ACK_FORMAT, self.epoch)

        # bulldog liveness, updated from the /bulldog_status callback thread.
        self.first_status_seen = False
        self.last_status_time = 0.0       # rospy.Time.now().to_sec()
        self.last_status_value = False
        rospy.Subscriber("/bulldog_status", Bool, self._bulldog_status_cb, queue_size=1)

        # Session source-pinning: lock onto the first sender, ignore the rest.
        self.sender_addr = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", port))
        self.sock.settimeout(timeout_s)

        self.received = 0
        rospy.loginfo(f"E-stop UDP bridge listening on udp/{port} (epoch={self.epoch})")
        print(f"E-stop UDP bridge listening on udp/{port}. "
              f"Waiting for packets from the Mac sender...")

    def _bulldog_status_cb(self, msg: Bool) -> None:
        self.first_status_seen = True
        self.last_status_time = rospy.Time.now().to_sec()
        self.last_status_value = bool(msg.data)

    def _bulldog_alive(self) -> bool:
        # Before bulldog has ever published, we are still starting up: stay alive
        # so /experimentor_estop flows and bulldog can launch against it.
        if not self.first_status_seen:
            return True
        fresh = (rospy.Time.now().to_sec() - self.last_status_time) <= BULLDOG_STATUS_TIMEOUT_S
        return fresh and self.last_status_value

    def run(self) -> None:
        report_at = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown():
            # Couple our life to bulldog's: once bulldog has been alive and then
            # goes stale/False, stop acking and exit (the Mac then self-exits).
            if self.first_status_seen and not self._bulldog_alive():
                rospy.logwarn("bulldog is no longer alive (/bulldog_status stale or False); "
                              "bridge exiting so the Mac sender self-terminates.")
                print("bulldog gone -- bridge exiting; the Mac sender will stop and need a relaunch.")
                return

            try:
                data, addr = self.sock.recvfrom(64)
            except socket.timeout:
                # No packet this window -> publish nothing -> heartbeat decays.
                continue

            if len(data) != PACKET_SIZE:
                rospy.logwarn_throttle(1.0, f"Ignoring malformed packet ({len(data)} bytes)")
                continue

            # Pin to the first sender we hear from; ignore any other source so a
            # stray/rogue LAN sender cannot mask a real press or fake the heartbeat.
            if self.sender_addr is None:
                self.sender_addr = addr
                rospy.loginfo(f"locked onto e-stop sender at {addr[0]}:{addr[1]}")
            elif addr != self.sender_addr:
                rospy.logwarn_throttle(5.0, f"Ignoring packet from unexpected source {addr}")
                continue

            seq, exp_pressed = struct.unpack(PACKET_FORMAT, data)

            # One packet -> one message (keeps /experimentor_estop's heartbeat alive).
            self.exp_pub.publish(Bool(data=exp_pressed))

            # Ack the sender with our epoch ONLY while bulldog is alive. (If it
            # weren't, we'd have exited above; this also covers the startup grace.)
            self.sock.sendto(self.ack, self.sender_addr)

            self.received += 1
            if exp_pressed:
                rospy.loginfo(f"PRESS relayed: experimentor={exp_pressed} (seq={seq})")

            now = rospy.Time.now()
            if now >= report_at:
                rospy.loginfo(f"bridge alive: {self.received} packets relayed")
                report_at = now + rospy.Duration(5.0)


if __name__ == "__main__":
    rospy.init_node("estop_udp_bridge", anonymous=True)
    EStopUDPBridge().run()
