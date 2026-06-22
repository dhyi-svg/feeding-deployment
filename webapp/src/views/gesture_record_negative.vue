<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class = "usertext">
        <div class="username">{{ username }}</div>
        <div class = "userslog">Enjoy your mealtime now!</div>
      </div>
    </div>
  </div>
  <div class="video-recorder">
    <h2>Negative Video Recorder</h2>
    <div class="layout-container">
      <div class="recorder-container">
        <div class="controls">
          <button class=buttonchoose @click="startRecording('positive')">Start Recording</button>
          <button class=buttonchoose @click="stopRecording('negative')">Stop Recording</button>
          <button class=buttonchoose @click="deletRecording">Delete Last Recording</button>
          <button class=buttonchoose @click="goToNextPage">Return</button>
        </div>
        <span class="left_text">Robot Text Response:</span>
        <div class="option-container">
            <textarea
              v-model="customOrder"
              placeholder="Waiting for the text response..."
              class="custom-input-box"
              readonly
            ></textarea>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib';
import { useRouter } from 'vue-router';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data() {
    return {
      username: USER,
      listener: null,
      mediaRecorder: null,
      recordedChunks: [],
      recordedVideos: [],
      isRecording: false,
      currentLabel: '',
      positiveCount: 1,
      negativeCount: 1,
      ros: null,
      videoTopic: null,
      customOrder: '',
    }
  },
  setup() {
    const router = useRouter();
    return {router};
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
    this.initRosConnection()
    
    this.initROS();
  },
  beforeUnmount() {
  },
  beforeRouteLeave(to, from, next) {
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
    cleartheinput() {
      this.transcript = '';
    },
    initRosConnection() {

      const listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp', 
        messageType: 'std_msgs/String'
      });

      listener.subscribe((message) => {
        const parsedMessage = JSON.parse(message.data)
        if (parsedMessage.state === 'gesture_response') {
          this.customOrder = parsedMessage.status;
        }
      });

      this.listener = listener;
    },

    initROS() {

      this.videoTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/video_stream',
        messageType: 'sensor_msgs/CompressedImage',
      });
    },

    startRecording(label) {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_record_negative',
          status: 'start'
        }) 
      })
      this.publisher.publish(message);
    },

    stopRecording() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_record_negative',
          status: 'stop'
        }) 
      })
      this.publisher.publish(message);
    },

    deletRecording() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_record_negative',
          status: 'delete'
        }) 
      })
      this.publisher.publish(message);
    },

    deleteVideo(index) {
      this.recordedVideos.splice(index, 1);
    },

    publishToROS(video) {

      fetch(video.url)
        .then((response) => response.blob())
        .then((blob) => {

          const reader = new FileReader();
          reader.readAsArrayBuffer(blob);
          reader.onloadend = () => {
            const uint8Array = new Uint8Array(reader.result);

            const message = new ROSLIB.Message({
              format: 'jpeg',
              data: Array.from(uint8Array),
            });

            this.videoTopic.publish(message);
          };
        })
        .catch((error) => {
        });
    },
    
    goToNextPage() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_record_negative',
          status: 'back'
        }) 
      })
      this.publisher.publish(message);
      this.router.push('/robot_executing');
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
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
    initSubscriber() {

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })

      this.listener.subscribe((msg) => {

        try {
          const parsedMessage = JSON.parse(msg.data);
          if (parsedMessage.state === 'prepare_bite' && parsedMessage.status === 'completed') {
            this.$router.push('/meal_setup'); 
          }
        } catch (error) {
        }
      })
      this.listener.subscribe((message) => {
        this.handleRosMessage(message);
      });
    },

    redirectToChangeItem() {
      this.$router.push('/gesture_setup')
    },
  }
}
</script>

<style scoped>
.left_text{
  font-family: Verdana;
  font-size: 18px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;
  display: block;
  margin-top: 1vh;
  margin-bottom: 1vh;
}
.option-container {
  width: 100%;
  height: 20vh; 
  display: flex;
  justify-content: center;
  align-items: center;
}
.video-recorder {
  height: 90vh;
  max-width: 1200px;
  margin: auto;
  text-align: center;
}
.video-recorder h2 {
  font-size: 20px;   
  margin-bottom: 10px;
}

.layout-container {
  display: flex;
  gap: 20px;
  height: 60vh;
}

.recorder-container {
  flex: 1;
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  border: 1px solid #ccc;
  padding: 10px;
  border-radius: 8px;
  max-width: 90vw; 
  height: 70vh;
  margin: 0 auto;   
}

.recorder-container video {
  width: 100%;
  height: auto;
  max-height: 80vh; 
  border-radius: 8px;
  object-fit: cover;
}

.controls {
  margin-top: 10px;
  display: flex;
  align-items: center;
}
.buttonchoose {
  background-color: #FFE699; 
  color: #000000;           
  border-radius: 20px;
  padding: 12px 20px;
  font-size: 25px;
  border: none;
  cursor: pointer;
  margin: 5px;
  width: 250px;              
  height: 100px;
  transition: all 0.3s ease;
}

button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

.preview-container {
  flex: 1;
  display: flex;
  flex-direction: column;
  border: 1px solid #ccc;
  padding: 10px;
  border-radius: 8px;
  max-height: 90vh; 
  overflow-y: auto;  
}

.video-list {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  justify-content: center;
}

.video-item {
  position: relative;
  text-align: center;
}
.custom-input-box {
  width: 90%; 
  height: 100%; 
  font-size: 1.7vw;
  border: 1px solid #ccc;
  border-radius: 10px;
  padding: 10px;
  resize: none; 
  box-sizing: border-box;
}
.video-item video {
  width: 200px;
  height: 150px;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
}

.button-group {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 10px; 
  margin-top: 8px;
  width: 100%; 
}

.delete-button {
  background-color: #ff4d4f;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 8px 12px;
  font-size: 12px;
  cursor: pointer;
  width: 40%; 
  text-align: center;
  height: 30%;
}

.delete-button:hover {
  background-color: #d9363e;
}

.ros-button {
  background-color: #28a745;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 8px 12px;
  font-size: 12px;
  cursor: pointer;
  width: 40%; 
  text-align: center;
  height: 30%;
}

.ros-button:hover {
  background-color: #218838;
}

.next-page-button {
  position: fixed;
  bottom: 20px;
  right: 20px;
  padding: 15px 25px;
  background-color: #FFE699; 
  color: #000000;
  font-size: 16px;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
}

.next-page-button:hover {
  background-color: #0056b3;
}
.top {
  height: 9vh;
  background: #eee;
  display: flex;
  align-items: unset;
  justify-content: space-between;
  padding: 5px;
  margin-bottom: 5px;
  .food {
    width: 500px;
    height: 200px;
    top: 179px;
    left: 68px;
    gap: 0px;
    opacity: 0px;
  }
  .right {
    display: flex;
    justify-content: center;
    align-items: center;
    .settings-button-text{
      font-family: Verdana;
      font-size: 18px;
      font-weight: 400;
      line-height: 24px;
      letter-spacing: 0.17499999701976776px;
      text-align: left;
    }
    .setting-container {
      position: relative;
    }
    .settings-button {
      background-color: #6e7e8e;
      border: none;
      border-radius: 8px;
      color: white;
      padding: 10px 20px;
      margin-left: 10px;
      cursor: pointer;
      font-size: 16px;
      display: flex;
      align-items: center;
      height: 50px;
      white-space: nowrap;
    }
    .settings-button span {
      margin-left: 5px;
    }
    .settings-panel {
      position: absolute;
      top: 120%;
      left: 50%;
      transform: translateX(-50%);
      width: calc(90%); 
      max-width: 200px; 
      background-color: #6e7e8e;
      border-radius: 8px;
      color: white;
      padding: 15px;
      box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
      text-align: left;
    }
    .settings-panel h3 {
      margin-top: 0;
    }
    .settings-panel label {
      margin-left: 5px;
      font-size: 14px;
    }
  }
  .left {
    display: flex;
    justify-content: space-between;
    padding:15px
  }
  .usertext{
    align-items: baseline;
    display: flex;
    justify-content: center;
    flex-flow: column;
    margin-left: 5px;
  }
  .username{
    font-family: Verdana;
    font-size: 20px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
  .userslog{
    font-family: Verdana;
    font-size: 16px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
}

.left {
  display: flex;
  align-items: center;
}

.right {
  display: flex;
  align-items: center;
}

.setting-container {
  position: relative;
}

.settings-button {
  background-color: #6e7e8e;
  border: none;
  border-radius: 8px;
  color: white;
  padding: 10px 20px;
  margin-left: 10px;
  cursor: pointer;
  font-size: 16px;
  display: flex;
  align-items: center;
  height: 50px;
}

.settings-button span {
  margin-left: 5px;
}

.settings-panel {
  position: absolute;
  top: 120%;
  left: 50%;
  transform: translateX(-50%);
  width: calc(90%);
  max-width: 200px;
  background-color: #6e7e8e;
  border-radius: 8px;
  color: white;
  padding: 15px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
  text-align: left;
}

.settings-panel h3 {
  margin-top: 0;
}

.settings-panel label {
  margin-left: 5px;
  font-size: 14px;
}

.content {
  display: flex;
  justify-content: center;
  align-items: center;
  height: 70vh;
}

.message {
  font-weight: bold;
  font-family: Verdana;
  font-size: 40px;
  line-height: 24px;
  letter-spacing: 0.17499999701976776px;
  text-align: center;
}

.footer {
  display: flex;
  justify-content: center;
  padding: 20px;
}

.succeed-button {
  background-color:rgb(179, 181, 184);
  border: none;
  border-radius: 8px;
  color: white;
  padding: 10px 20px;
  cursor: pointer;
  font-size: 16px;
  display: flex;
  align-items: center;
  height: 50px;
}
</style>

