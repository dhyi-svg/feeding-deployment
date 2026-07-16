<template>
  <div class="page" @click="cancelAutocontinue">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">{{ currentSlog }}</div>
      </div>
    </div>

    <div class="bd det-bd">
      <div class="det-instruction" v-html="currentInstruction"></div>

      <div v-if="currentLegend" class="legend-row">
        <span v-for="(item, i) in currentLegend" :key="i" class="legend-chip">
          <span
            class="swatch"
            :class="{ 'swatch-box': item.swatch === 'box' }"
            :style="item.swatch === 'box'
              ? { borderColor: item.color || '#ffbe3c' }
              : { backgroundColor: item.color }"
          ></span>
          {{ item.label }}
        </span>
      </div>

      <div class="cam cam-wide">
        <img v-if="imageSrc" :src="imageSrc" alt="Detection visualization" />
        <div v-else class="cam-placeholder">Waiting for detection image...</div>
      </div>

      <p v-if="!userInteracted && countdown !== null" class="cdown">Auto-confirming in <span>{{ countdown }}s</span></p>
      <div class="det-actions">
        <button class="btn md amber" @click="confirmDetection">Looks Correct</button>
        <button class="btn md ghost" @click="redoDetection">Redo</button>
        <button v-if="detectionType === 'attachment'" class="btn md teal" @click="correctColor">Correct Color</button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap';
import { ROS_URL, USER } from '@/config/parameterConfig';

const DETECTION_MESSAGES = {
  handle: {
    slog: 'Please verify the handle detection.',
    instruction:
      "Does the robot's handle detection look correct?<br>" +
      "Click 'Looks Correct' to proceed, or 'Redo' to detect again.",
    legend: [
      { swatch: 'box', label: 'Detected door (opens toward you)' },
      { color: '#ff3b30', label: 'Handle — where the robot grabs' },
      { color: '#3498db', label: 'Hinge — the edge the door pivots on' }
    ]
  },
  button: {
    slog: 'Please verify the microwave button detection.',
    instruction:
      "Does the robot's microwave button detection look correct?<br>" +
      "If the wrong spot is marked, tap 'Redo'.",
    legend: [
      { color: '#ff3b30', label: 'Start / 30 secs button to press' }
    ]
  },
  sink: {
    slog: 'Please verify the sink placement detection.',
    instruction:
      "Does the robot's sink detection look correct?<br>" +
      "If it's off, tap 'Redo' to detect again.",
    legend: [
      { swatch: 'box', label: 'Detected sink area' },
      { color: '#ff3b30', label: 'Where the plate will be placed' }
    ]
  },
  plate: {
    slog: 'Please verify the plate placement detection.',
    instruction:
      "Does the robot's plate placement detection look correct?<br>" +
      "If it's off, tap 'Redo' to detect again.",
    legend: [
      { swatch: 'box', label: 'Detected placement marker' },
      { color: '#ff3b30', label: 'Where the plate will be placed' }
    ]
  },
  attachment: {
    slog: 'Please verify the attachment detection.',
    instruction:
      "Does the robot's attachment detection look correct?<br>" +
      "If the wrong area is highlighted, tap 'Correct Color' to adjust the color filter.",
    legend: [
      { color: '#ff3b30', label: 'Color pixels used' },
      { color: '#00c8c8', label: 'Color matches rejected' },
      { swatch: 'box', label: 'Detected attachment (corners + center)' }
    ]
  }
};

export default {
  data () {
    return {
      ros: null,
      username: USER,
      detectionType: 'handle',
      listener: null,
      imageListener: null,
      publisher: null,
      imageSrc: null,
      countdown: null,
      countdownInterval: null,
      userInteracted: false,
      // Set ONLY on countdown expiry: responses carry user_action tap|autocontinue.
      autoSubmit: false,
    }
  },
  computed: {
    currentMessage () {
      return DETECTION_MESSAGES[this.detectionType] || DETECTION_MESSAGES.handle;
    },
    currentSlog () {
      return this.currentMessage.slog;
    },
    currentInstruction () {
      return this.currentMessage.instruction;
    },
    currentLegend () {
      return this.currentMessage.legend || null;
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
  },
  beforeRouteLeave (to, from, next) {
    this.stopCountdown()
    if (this.listener) {
      this.listener.unsubscribe();
      this.listener = null;
    }
    if (this.imageListener) {
      this.imageListener.unsubscribe();
      this.imageListener = null;
    }
    if (this.publisher) {
      this.publisher.unadvertise();
      this.publisher = null;
    }
    next();
  },
  methods: {
    handleRosMessage (message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        
        if (parsedMessage.state === 'detection_confirm' && parsedMessage.detection_type) {
          this.detectionType = parsedMessage.detection_type;
          // The 'info' message carries the countdown (autocontinue_seconds
          // <= 0 means wait for the user). A redo suppresses restarting it:
          // the user is clearly attending.
          if (Number.isFinite(parsedMessage.autocontinue_seconds) && parsedMessage.autocontinue_seconds > 0 && this.countdownInterval === null && !this.userInteracted) {
            this.startCountdown(Math.round(parsedMessage.autocontinue_seconds))
          }
        }
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          this.$router.push(route);
        }
      } catch (error) {
      }
    },
    initPublisher () {
      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
    initSubscriber () {

      this.imageListener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/camera/image/compressed',
        messageType: 'sensor_msgs/CompressedImage'
      })
      this.imageListener.subscribe((message) => {
        try {
          this.imageSrc = 'data:image/jpeg;base64,' + message.data;
        } catch (error) {
        }
      });

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })
      this.listener.subscribe((message) => {
        this.handleRosMessage(message);
      });
    },
    publishResponse (status) {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'detection_confirm',
          status: status,
          detection_type: this.detectionType,
          user_action: this.autoSubmit ? 'autocontinue' : 'tap'
        })
      })
      this.publisher.publish(message);
      this.autoSubmit = false;
    },
    startCountdown (seconds) {
      this.countdown = seconds
      this.countdownInterval = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown -= 1;
        } else {
          this.stopCountdown()
          // Unattended: accept the detection.
          this.autoSubmit = true
          this.confirmDetection()
        }
      }, 1000);
    },
    stopCountdown () {
      if (this.countdownInterval) {
        clearInterval(this.countdownInterval);
        this.countdownInterval = null;
      }
    },
    cancelAutocontinue () {
      // Any tap on the page cancels the auto-confirm countdown; the user must
      // then explicitly confirm/redo. userInteracted also stops a resent "info"
      // from re-arming the countdown.
      this.userInteracted = true
      this.stopCountdown()
    },
    confirmDetection () {
      this.userInteracted = true
      this.stopCountdown()
      this.publishResponse('confirm');
    },
    redoDetection () {
      this.userInteracted = true
      this.stopCountdown()
      this.imageSrc = null;
      this.publishResponse('redo');
    },
    correctColor () {
      this.userInteracted = true
      this.stopCountdown()
      this.publishResponse('correct_color');
    }
  }
}
</script>

<style scoped>
.legend-row {
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  gap: 10px 16px;
  margin: 10px 0 2px;
}
.legend-chip {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  font-size: 0.95rem;
  background: rgba(255, 255, 255, 0.06);
  border-radius: 999px;
  padding: 6px 12px;
}
.swatch {
  width: 16px;
  height: 16px;
  border-radius: 50%;
  flex: 0 0 auto;
}
.swatch-box {
  border-radius: 3px;
  background: transparent !important;
  border: 3px solid #ffbe3c; /* fallback; per-chip color set inline (mirrors the box) */
}
</style>

