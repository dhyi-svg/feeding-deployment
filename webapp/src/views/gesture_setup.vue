<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Describe the new gesture</div>
      </div>
    </div>

    <div class="bd">
      <div class="field-stack">
        <span class="field-lbl">Gesture name</span>
        <div class="field-box">
          <input
            type="text"
            v-model="transcript"
            placeholder="Typing..."
            ref="textarea1"
            class="field-input"
          >
          <button @click="startSpeechRecognition1"
                  class="icon-btn"
                  :class="{ 'amber-ic': isRecognizing1 }"
                  :disabled="isRecognizing1"
                  title="voice">
            <img alt="voice" src="../assets/voice.png">
          </button>
          <button @click="cleartheinput1" class="icon-btn" title="clear">
            <img alt="clear" src="../assets/clear.png">
          </button>
        </div>

        <span class="field-lbl">Description</span>
        <div class="field-box tall">
          <textarea
            v-model="transcriptDes"
            placeholder="Typing..."
            ref="textarea2"
            class="field-input"
          ></textarea>
          <button @click="startSpeechRecognition2"
                  class="icon-btn"
                  :class="{ 'amber-ic': isRecognizing2 }"
                  :disabled="isRecognizing2"
                  title="voice">
            <img alt="voice" src="../assets/voice.png">
          </button>
          <button @click="cleartheinput2" class="icon-btn" title="clear">
            <img alt="clear" src="../assets/clear.png">
          </button>
        </div>

        <div class="field-actions">
          <button class="btn md amber" style="width:35%" @click="confirmSelection">Next</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';
export default {
  data () {
    return {
      ros: null,
      username: USER,
      isRecognizing1: false,
      isRecognizing2: false,
      recognition1: null,
      recognition2: null,
      listener: null,
      publisher: null,
      transcript: '',
      transcriptDes: '',
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
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
    cleartheinput1() {
      this.transcript = '';
    },

    cleartheinput2() {
      this.transcriptDes = '';
    },

    startSpeechRecognition1() {
      if (this.isRecognizing1) {
        return;
      }
      if (!this.recognition1) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition1 = new SpeechRecognition();
        this.recognition1.lang = 'en-US';
        this.recognition1.continuous = false;

        this.recognition1.onresult = (event) => {
          this.transcript += event.results[0][0].transcript;
          this.isRecognizing1 = false;
        };

        this.recognition1.onerror = () => {
          this.isRecognizing1 = false;
        };

        this.recognition1.onend = () => {
          this.isRecognizing1 = false;
        };
      }

      if (this.isRecognizing1) {
        this.recognition1.stop();
      }

      this.isRecognizing1 = true;
      this.recognition1.start();
    },

    startSpeechRecognition2() {
      if (this.isRecognizing2) {
        return;
      }
      if (!this.recognition2) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition2 = new SpeechRecognition();
        this.recognition2.lang = 'en-US';
        this.recognition2.continuous = false;

        this.recognition2.onresult = (event) => {
          this.transcriptDes += event.results[0][0].transcript;
          this.isRecognizing2 = false;
        };

        this.recognition2.onerror = () => {
          this.isRecognizing2 = false;
        };

        this.recognition2.onend = () => {
          this.isRecognizing2 = false;
        };
      }

      if (this.isRecognizing2) {
        this.recognition2.stop();
      }

      this.isRecognizing2 = true;
      this.recognition2.start();
    },
    handleRosMessage(message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          if (typeof route === 'string') {
            this.$router.push(route);
          } else if (typeof route === 'object') {
            this.$router.push(route);
          }
        }
      } catch (error) {
      }
    },

    confirmSelection () {
      if (this.transcript !== '' && this.transcriptDes !== '') {
        const voiceMessage = new ROSLIB.Message({
          data: JSON.stringify({
            state: this.transcript,
            status: this.transcriptDes
          })
        });
        this.publisher.publish(voiceMessage);
      }
      this.transcript = '';
      this.transcriptDes = '';
      this.$router.push('/robot_executing');
    },
    initSubscriber() {

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })
      this.listener.subscribe((message) => {
        this.handleRosMessage(message);
      });
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

<style scoped>
.field-stack {
  max-width: 820px;
  width: 100%;
  margin: 0 auto;
}
</style>
