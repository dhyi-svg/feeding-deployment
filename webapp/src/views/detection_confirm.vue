<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">{{ currentSlog }}</div>
      </div>
    </div>

    <div class="bd det-bd">
      <div class="det-instruction" v-html="currentInstruction"></div>

      <div class="cam cam-wide">
        <img v-if="imageSrc" :src="imageSrc" alt="Detection visualization" />
        <div v-else class="cam-placeholder">Waiting for detection image...</div>
      </div>

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
      "Does the robot's handle detection look correct? <br>" +
      "Green dot = handle, Blue dot = hinge. Red dot = where to place plate. <br>" +
      "Click 'Looks Correct' to proceed, or 'Redo' to detect again."
  },
  button: {
    slog: 'Please verify the microwave button detection.',
    instruction:
      "Does the robot's microwave button detection look correct? <br>" +
      "The marked point should be on the start / 30 secs button to press. <br>" +
      "Click 'Looks Correct' to proceed, or 'Redo' to detect again."
  },
  plate: {
    slog: 'Please verify the plate placement detection.',
    instruction:
      "Does the robot's plate placement detection look correct? <br>" +
      "Red dot = where the plate will be placed on the table. <br>" +
      "Click 'Looks Correct' to proceed, or 'Redo' to detect again."
  },
  attachment: {
    slog: 'Please verify the attachment detection.',
    instruction:
      "Does the robot's attachment detection look correct? <br>" +
      "The highlighted region shows the detected attachment point. <br>" +
      "Click 'Looks Correct' to proceed, 'Redo' to detect again, or 'Correct Color' to adjust the color filter."
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
          detection_type: this.detectionType
        })
      })
      this.publisher.publish(message);
    },
    confirmDetection () {
      this.publishResponse('confirm');
    },
    redoDetection () {
      this.imageSrc = null;
      this.publishResponse('redo');
    },
    correctColor () {
      this.publishResponse('correct_color');
    }
  }
}
</script>

