
import time
import argparse
import pyaudio

import rospy
from std_msgs.msg import Bool

from feeding_deployment.safety.button import Button as EStop
ESTOP_FREQUENCY = 100

class EStopsPublisher:
    """Reads the single (experimentor) e-stop button locally on the NUC and
    publishes it on /experimentor_estop at ESTOP_FREQUENCY Hz.

    The stream of messages IS the heartbeat that bulldog.py watches: if this
    process dies or the button hardware disconnects, the topic goes quiet and
    bulldog's "<50 msgs/sec" check stops the arm within ~1s. See estop_sender.py
    / estop_udp_bridge.py for the remote-button (Mac) variant.
    """

    def __init__(self, experimentor_estop_id: int):

        self.experimentor_estop = EStop(experimentor_estop_id)

        self.experimentor_estop_pub = rospy.Publisher("/experimentor_estop", Bool, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            experimentor_estop_pressed = self.experimentor_estop.check()

            self.experimentor_estop_pub.publish(Bool(data=experimentor_estop_pressed))

            if experimentor_estop_pressed:
                print("Experimentor E-Stop pressed")

            time.sleep(max(0, 1.0/ESTOP_FREQUENCY - (time.time() - start_time)))

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, default=None,
                        help="audio device index of the experimentor e-stop button")

    args = parser.parse_args()

    if args.id is None:
        audio = pyaudio.PyAudio()
        device_indices = []
        for i in range(audio.get_device_count()):
            device_info = audio.get_device_info_by_index(i)
            if device_info["maxInputChannels"] > 0:  # Only consider input devices
                device_indices.append(i)
                print(f"Device {i}: {device_info['name']}")
        raise ValueError("Please provide the input device index")

    rospy.init_node("estop_publisher")
    estop_publisher = EStopsPublisher(experimentor_estop_id=args.id)
    estop_publisher.run()
