<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Time to eat!</div>
      </div>
    </div>

    <div class="bd">
      <div class="simple-confirm">
        <p>Your meal is on the table!<br>Please take a seat, and press the button whenever you're ready to begin.</p>
        <button class="btn lg amber" style="min-width:24vw" @click="handleButtonClick">Yes, I'm ready</button>
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

        // The backend re-sends the feeding_ready jump until answered; ignore
        // our own resends. This page never auto-continues -- it only leaves
        // on the user's tap (or an explicit jump elsewhere).
        if (parsedMessage.state === 'feeding_ready') {
          return
        }

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
    handleButtonClick() {
      this.publishMessage();
      this.$router.push('/robot_executing');
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
          state: 'feeding_ready',
          status: 'confirm',
          user_action: 'tap'
        })
      })

      this.publisher.publish(message);
    }
  }
}
</script>
