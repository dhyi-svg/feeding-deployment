<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class="usertext">
        <div class="username">{{ username }}</div>
        <div class="userslog">{{ currentSlog }}</div>
      </div>
    </div>
  </div>

  <div class="content">
    <div class="instruction" v-html="currentInstruction"></div>

    <div class="image-container">
      <img v-if="imageSrc" :src="imageSrc" class="detection-image" alt="Detection visualization" />
      <div v-else class="waiting">Waiting for detection image...</div>
    </div>

    <div class="buttons">
      <button class="continue-button" @click="confirmDetection">Looks Correct</button>
      <button class="retry-button" @click="redoDetection">Redo</button>
      <button v-if="detectionType === 'attachment'" class="color-button" @click="correctColor">Correct Color</button>
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
        name: this.subscribeTopic,
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

<style scoped>
.top {
  height: 9vh;
  background: #eee;
  display: flex;
  align-items: unset;
  justify-content: space-between;
  padding: 5px;
  margin-bottom: 5px;
}
.left {
  display: flex;
  align-items: center;
  padding: 15px;
}
.usertext {
  align-items: baseline;
  display: flex;
  justify-content: center;
  flex-flow: column;
  margin-left: 5px;
}
.username {
  font-family: Verdana;
  font-size: 20px;
  font-weight: 400;
  line-height: 18px;
  text-align: left;
}
.userslog {
  font-family: Verdana;
  font-size: 16px;
  font-weight: 400;
  line-height: 18px;
  text-align: left;
}
.content {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  height: 85vh;
}
.instruction {
  margin-bottom: 20px;
  text-align: center;
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
  line-height: 30px;
  width: 60vw;
}
.image-container {
  display: flex;
  justify-content: center;
  align-items: center;
  margin-bottom: 20px;
  min-height: 30vh;
}
.detection-image {
  max-width: 50vw;
  max-height: 45vh;
  border: 2px solid #6e7e8e;
  border-radius: 8px;
}
.waiting {
  font-family: Verdana;
  font-size: 20px;
  color: #6e7e8e;
}
.buttons {
  display: flex;
  gap: 20px;
  margin-top: 10px;
}
.continue-button,
.retry-button,
.color-button {
  border: none;
  color: black;
  cursor: pointer;
  background-color: #FFE699;
  border-radius: 20px;
  width: 20vw;
  height: 12vh;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: 20px;
  font-family: Verdana;
  font-size: 30px;
  font-weight: 400;
  line-height: 24px;
  text-align: center;
  padding: 10px;
}
.color-button {
  background-color: #B3D9FF;
}
</style>
