"""Minimal MJPEG HTTP server for a live view of /camera/color/image_raw, since this
container's cv2 build is headless (no imshow) and no ROS image-viewer package
(image_view/rqt_image_view) is installed. Browser-viewable at http://localhost:8095/.
One-off calibration-session tool, not wired into anything else.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

latest_jpeg = None
lock = threading.Lock()


def image_cb(msg):
    global latest_jpeg
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        with lock:
            latest_jpeg = buf.tobytes()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while not rospy.is_shutdown():
                with lock:
                    jpeg = latest_jpeg
                if jpeg is not None:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass


bridge = CvBridge()
rospy.init_node("mjpeg_preview", anonymous=True, disable_signals=True)
rospy.Subscriber("/camera/color/image_raw", Image, image_cb, queue_size=1)

server = HTTPServer(("0.0.0.0", 8095), Handler)
print("[mjpeg_preview] serving on http://localhost:8095/")
server_thread = threading.Thread(target=server.serve_forever, daemon=True)
server_thread.start()
rospy.spin()
