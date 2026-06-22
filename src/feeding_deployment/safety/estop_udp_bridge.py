"""NUC-side e-stop bridge: UDP -> ROS.

This is the NUC half of what `estops_publisher.py` does on a Linux machine with
a locally-plugged button. Here the button lives on a Mac that cannot run ROS;
`estop_sender.py` on the Mac streams the button state to us over UDP, and we
republish it onto the ROS topics `bulldog.py` already watches:
`/user_estop` and `/experimentor_estop`.

Drop-in replacement for `estops_publisher.py` in `run_bulldog.sh` when the
button is remote. `bulldog.py` and `arm_server.py` need NO changes.

Fail-safe design -- ONE received packet = ONE published message on each topic:
  * Normal flow: Mac sends ~100 packets/sec  -> ~100 msgs/sec on each topic,
    well above bulldog's 50/sec threshold -> healthy.
  * Mac dies / app crashes / cable unplugged / network drops: packets stop,
    we publish nothing, both topics go quiet, and bulldog's "<50 msgs/sec"
    frequency check stops the arm within ~1s. The network link is thus part of
    the monitored heartbeat.

We MUST publish BOTH topics every packet even if only one physical button
exists: bulldog checks the heartbeat frequency of BOTH /user_estop and
/experimentor_estop, so silence on either would itself trigger a stop.
"""

import socket
import struct

import rospy
from std_msgs.msg import Bool

# Must match estop_sender.py
PACKET_FORMAT = "!Q??"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

UDP_PORT = 5005
# If no packet arrives within this window, recvfrom() returns and we simply
# publish nothing that cycle -> topic rate falls -> bulldog trips. Keep it
# short relative to bulldog's ~1s window so a genuine stop is fast.
RECV_TIMEOUT_S = 0.05


class EStopUDPBridge:
    def __init__(self, port: int = UDP_PORT, timeout_s: float = RECV_TIMEOUT_S) -> None:
        self.user_pub = rospy.Publisher("/user_estop", Bool, queue_size=1)
        self.exp_pub = rospy.Publisher("/experimentor_estop", Bool, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", port))
        self.sock.settimeout(timeout_s)

        self.received = 0
        self.last_seq = None
        rospy.loginfo(f"E-stop UDP bridge listening on udp/{port}")
        print(f"E-stop UDP bridge listening on udp/{port}. "
              f"Waiting for packets from the Mac sender...")

    def run(self) -> None:
        report_at = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown():
            try:
                data, _addr = self.sock.recvfrom(64)
            except socket.timeout:
                # No packet this window -> publish nothing -> heartbeat decays.
                continue

            if len(data) != PACKET_SIZE:
                rospy.logwarn_throttle(1.0, f"Ignoring malformed packet ({len(data)} bytes)")
                continue

            seq, user_pressed, exp_pressed = struct.unpack(PACKET_FORMAT, data)

            # One packet -> one message on EACH topic (keeps both heartbeats alive).
            self.user_pub.publish(Bool(data=user_pressed))
            self.exp_pub.publish(Bool(data=exp_pressed))

            self.received += 1
            self.last_seq = seq
            if user_pressed or exp_pressed:
                rospy.loginfo(f"PRESS relayed: user={user_pressed} exp={exp_pressed} (seq={seq})")

            now = rospy.Time.now()
            if now >= report_at:
                rospy.loginfo(f"bridge alive: {self.received} packets relayed, last seq={self.last_seq}")
                report_at = now + rospy.Duration(5.0)


if __name__ == "__main__":
    rospy.init_node("estop_udp_bridge", anonymous=True)
    EStopUDPBridge().run()
