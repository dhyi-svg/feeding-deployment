<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Adjust how the robot is feeding you</div>
      </div>
    </div>

    <div class="bd talk-bd">
      <p class="talk-lbl">What would you like to edit?</p>
      <div class="talk-box">
        <textarea
          v-model="transcript"
          placeholder="Typing..."
          ref="textarea"
          class="talk-input"
        ></textarea>
        <button @click="startSpeechRecognition"
                class="icon-btn"
                :class="{ 'amber-ic': isRecognizing }"
                :disabled="isRecognizing"
                title="voice">
          <img alt="voice" src="../assets/voice.png">
        </button>
        <button @click="cleartheinput" class="icon-btn" title="clear">
          <img alt="clear" src="../assets/clear.png">
        </button>
        <button @click="sendToRosFromTextBox" class="icon-btn amber-ic" title="send">
          <img alt="send" src="../assets/send.png">
        </button>
      </div>

      <p class="talk-lbl">Robot's response</p>
      <div class="response-box">{{ customOrder || 'Waiting for the text response...' }}</div>

      <button class="btn sm ghost w100" style="margin-top:auto" @click="$router.push('/task_selection')">
        ← Task Selection
      </button>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
      recognition: null,
      transcript: '',
      isRecognizing: false,
      customOrder: '',
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initRosConnection()
  },
  beforeRouteLeave (to, from, next) {
    if (this.listener) {
      this.listener.unsubscribe();
      this.listener = null;
    }

    if (this.publisher) {
      this.publisher.unadvertise();
      this.publisher = null;
    }

    next();
  },
  methods: {
    sendToRosFromTextBox() {
      if (this.publisher && this.transcript.trim() !== '') {
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'adaptability_request',
            status: this.transcript
          })
        });
        this.publisher.publish(message);
        this.transcript = '';
      } else {
      }
    },
    initRosConnection() {

      const listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      });

      listener.subscribe((message) => {
        const parsedMessage = JSON.parse(message.data)
        if (parsedMessage.state === 'adaptability_response') {
          this.customOrder = parsedMessage.status;
        }
      });
      this.listener = listener;
    },
    cleartheinput() {
      this.transcript = '';
    },
    startSpeechRecognition() {

      if (!this.recognition) {

        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition = new SpeechRecognition();
        this.recognition.lang = 'en-US';
        this.recognition.continuous = false;

        this.recognition.onresult = (event) => {

          this.transcript += event.results[0][0].transcript;
          this.isRecognizing = false;
        };

        this.recognition.onerror = (event) => {
          this.isRecognizing = false;
          this.focusTextarea();
        };

        this.recognition.onend = () => {
          this.isRecognizing = false;
          this.focusTextarea();
        };
      }

      if (this.isRecognizing) {
        this.recognition.stop();
      }

      this.isRecognizing = true;
      this.recognition.start();

      this.$nextTick(() => {
        this.focusTextarea();
      });
    },
    focusTextarea() {

      this.$refs.textarea.focus();
    },
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
  }
}
</script>
