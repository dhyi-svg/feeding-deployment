<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Ask the robot about this meal</div>
      </div>
    </div>

    <div class="bd talk-bd">
      <p class="talk-lbl">What would you like to know?</p>
      <div class="talk-box">
        <textarea
          v-model="transcript"
          placeholder="Typing..."
          ref="textarea"
          class="talk-input"
          @focus="handleFocus"
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
      <p class="voice-status" :class="{ empty: !voiceStatus }" aria-live="polite">
        <img alt="" src="../assets/voice.png">
        <span v-if="voiceStatus">{{ voiceStatus }}</span>
        <span v-else>&nbsp;</span>
      </p>

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
      // On-screen status for the dictation mic so failures aren't silent on the
      // iPad (no dev console). Shows the raw recognition error / lifecycle.
      voiceStatus: '',
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initRosConnection()
    this.releaseTakeoverMic()
  },
  beforeRouteLeave (to, from, next) {
    if (this.recognition && this.isRecognizing) {
      this.recognition.stop();
    }

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
    handleFocus() {
      window.scrollBy(0, window.innerHeight * 0.09);
    },
    sendToRosFromTextBox() {
      if (this.publisher && this.transcript.trim() !== '') {
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'transparency_request',
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
        if (parsedMessage.state === 'transparency_response') {
          this.customOrder = parsedMessage.status;
        }
      });

      this.listener = listener;
    },
    cleartheinput() {
      this.transcript = '';
    },

    releaseTakeoverMic() {
      return new Promise((resolve) => {
        let settled = false;
        const done = () => {
          if (settled) return;
          settled = true;
          resolve();
        };

        window.dispatchEvent(new CustomEvent('release-takeover-mic', {
          detail: { done }
        }));

        setTimeout(done, 300);
      });
    },

    async startSpeechRecognition() {
      if (this.isRecognizing) {
        return;
      }

      if (this.$refs.textarea) {
        this.$refs.textarea.blur();
      }

      await this.releaseTakeoverMic();

      const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
      if (!SpeechRecognition) {
        this.voiceStatus = 'speech recognition not supported in this browser';
        return;
      }

      if (!this.recognition) {
        this.recognition = new SpeechRecognition();
        this.recognition.lang = 'en-US';
        this.recognition.continuous = false;

        this.recognition.onstart = () => {
          this.voiceStatus = 'listening…';
        };

        this.recognition.onresult = (event) => {

          this.transcript += event.results[0][0].transcript;
          this.isRecognizing = false;
          this.voiceStatus = '';
        };

        this.recognition.onerror = (event) => {
          this.isRecognizing = false;
          // Surface the raw error so we can tell the failure mode apart:
          // 'not-allowed'/'service-not-allowed' -> mic blocked (often the
          // takeover button still holding it, or no HTTPS/permission);
          // 'no-speech' -> mic worked but heard nothing; 'aborted' -> stopped.
          this.voiceStatus = 'error: ' + (event.error || 'unknown') +
            (event.message ? ' — ' + event.message : '');
          // eslint-disable-next-line no-console
          console.error('[transparency] speech recognition error:', event.error, event.message);
          this.focusTextarea();
        };

        this.recognition.onend = () => {
          this.isRecognizing = false;
          if (this.voiceStatus === 'listening…') this.voiceStatus = 'no speech captured';
          this.focusTextarea();
        };
      }

      this.isRecognizing = true;
      this.voiceStatus = 'starting…';
      try {
        this.recognition.start();
      } catch (e) {
        // start() throws if called while a previous session is still active.
        this.isRecognizing = false;
        this.voiceStatus = 'start failed: ' + (e && e.message ? e.message : e);
        // eslint-disable-next-line no-console
        console.error('[transparency] recognition.start() threw:', e);
      }
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
