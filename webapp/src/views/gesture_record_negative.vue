<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Record examples of NOT the gesture</div>
      </div>
    </div>

    <div class="bd rec-frame">
      <div class="rec-preview">🎥 Recording happens on the robot's camera</div>
      <div class="rec-controls">
        <button class="btn sm amber" @click="startRecording('positive')">Start</button>
        <button class="btn sm ghost" @click="stopRecording('negative')">Stop</button>
        <button class="btn sm ghost" @click="deletRecording">Delete Last</button>
        <button class="btn sm teal" @click="goToNextPage">Return</button>
      </div>
      <span class="field-lbl">Robot's response</span>
      <div class="response-box" style="flex:0 0 auto;min-height:10vh">{{ customOrder || 'Waiting for the text response...' }}</div>
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


