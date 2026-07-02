<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Ready to place the plate?</div>
      </div>
    </div>

    <div class="bd">
      <div class="simple-confirm">
        <p>The robot is holding the plate {{ locationText }}.<br>Press 'Continue' when you're ready for it to let go.</p>
        <button class="btn lg amber" style="min-width:24vw" @click="handleButtonClick">Continue</button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

const LOCATION_TEXT = {
  microwave: 'in the microwave',
  table: 'on the table',
  sink: 'in the sink'
}

export default {
  data () {
    return {
      ros: null,
      username: USER,
      location: '',
      listener: null,
      publisher: null,
    }
  },
  computed: {
    locationText () {
      return LOCATION_TEXT[this.location] || 'at its destination'
    }
  },
  mounted () {
    this.location = this.$route.query.location || ''
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
          state: 'plate_release_confirm',
          status: 'confirm',
          location: this.location
        })
      })

      this.publisher.publish(message);
    }
  }
}
</script>
