<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Ready to transfer?</div>
      </div>
    </div>

    <div class="bd">
      <div class="confirm-body">
        <div class="cf-left">
          <strong>Did the robot grab the bite successfully?</strong>
          <p>If the pickup looks correct, continue and the robot will bring it to your mouth.<br><br>If not, retry and it will try again.</p>
        </div>
        <div class="cf-right">
          <button class="btn lg amber w100" @click="handleButtonClick">Continue — Transfer Bite</button>
          <button class="btn lg ghost w100" @click="handleButtonClick2">Retry Pickup</button>
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
      listener: null, 
      publisher: null,
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
    publishMessage() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_confirm_transfer',
          status: 'confirm'
        }) 
      })
      this.publisher.publish(message);
    },
    publishMessage2() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_confirm_transfer',
          status: 'cancel'
        }) 
      })

      this.publisher.publish(message);
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
    handleButtonClick() {
      this.publishMessage();
      this.$router.push('/robot_executing');
    },
    handleButtonClick2() {
      this.publishMessage2();
      this.$router.push('/robot_executing');
    },
    redirectToChangeItemCon () {
      this.$router.push('/robot_executing')
    },
    redirectToChangeItemRetry () {
      this.$router.push('/bite_selection')
    },
  }
}
</script>

<style scoped>
.confirm-body {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 3vw;
  align-items: center;
  flex: 1;
}
.cf-left strong {
  display: block;
  font: normal 3.4vh/1.3 Georgia, serif;
  color: var(--t);
  margin-bottom: 1.5vh;
}
.cf-left p {
  font-size: 2vh;
  color: var(--tm);
  line-height: 1.6;
}
.cf-right {
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
}
</style>
