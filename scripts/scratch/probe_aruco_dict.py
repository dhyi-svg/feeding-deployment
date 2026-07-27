"""One-off: figure out which ArUco dictionary the physical calibration board uses.
Not recorded anywhere since the original calibration script no longer exists."""

import cv2
import cv2.aruco as aruco
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("probe_aruco_dict", anonymous=True, disable_signals=True)
bridge = CvBridge()
msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10)
rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
cv2.imwrite("/tmp/aruco_probe_frame.png", rgb)
gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

dict_names = [n for n in dir(aruco) if n.startswith("DICT_") and "APRILTAG" not in n]
best = None
for name in dict_names:
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, name))
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, params)
    corners, ids, _ = detector.detectMarkers(gray)
    n = 0 if ids is None else len(ids)
    if n > 0:
        print(name, n, sorted(ids.flatten().tolist()))
        if best is None or n > best[1]:
            best = (name, n, sorted(ids.flatten().tolist()))

print("BEST:", best)
rospy.signal_shutdown("done")
