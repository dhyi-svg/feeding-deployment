from feeding_deployment.ros2_utils import node_handle
from feeding_deployment.ros2_utils import rospy_compat
from std_msgs.msg import String
import tempfile
from gtts import gTTS
from playsound import playsound

class Speak:
    def __init__(self):
        # TODO(ros2): anonymous=True dropped -- no rclpy equivalent under the
        # singleton-node model this migration uses.
        node_handle.init_node('Speak')
        node_handle.get_node().create_subscription(String, "/speak", self.callback, 10)
        print("Speak node initialized")

    # Speak the text
    def callback(self, msg):        
        text = msg.data
        print("Speaking: ", text)

        # Convert text to speech and play
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=True) as voice:
            gTTS(text=text, lang="en").write_to_fp(voice)
            voice.flush()  # Ensure data is written to file
            playsound(voice.name)

if __name__ == "__main__":
    try:
        Speak()
        rospy_compat.spin()
    # TODO(ros2): rospy_compat.ROSInterruptException is a stand-in only -- rclpy.spin()
    # does not raise it on shutdown (raises KeyboardInterrupt instead, or just returns).
    # This except clause is effectively dead; left in place rather than guessing a
    # replacement catch here.
    except rospy_compat.ROSInterruptException:
        pass