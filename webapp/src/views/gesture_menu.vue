<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Teach or test a gesture</div>
      </div>
    </div>

    <div class="bd">
      <div class="choice-row">
        <button class="choice-card" @click="navigateToAddGesture">
          <div class="cc-ico">➕</div>
          <div class="cc-lbl">Add Gesture</div>
          <div class="cc-sub">record a new one</div>
        </button>
        <button class="choice-card" @click="navigateToTestGesture">
          <div class="cc-ico">🧪</div>
          <div class="cc-lbl">Test Gesture</div>
          <div class="cc-sub">try it on the robot</div>
        </button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from "@/router/routeMap";
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data() {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initSubscriber()
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
    navigateToAddGesture() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_menu',
          status: 'add'
        })
      })
      this.publisher.publish(message)
      this.$router.push('/gesture_setup')
    },
    navigateToTestGesture() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_menu',
          status: 'test'
        })
      })
      this.publisher.publish(message)
      this.$router.push('/robot_executing')
    },
  }
}
</script>
