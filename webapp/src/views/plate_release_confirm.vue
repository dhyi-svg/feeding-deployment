<template>
  <div class="page" @click="cancelAutocontinue">
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
        <p class="cdown" :class="{ 'cdown-hidden': userInteracted || countdown === null }">Auto-continuing in <span>{{ countdown }}s</span></p>
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
      countdown: null,
      countdownInterval: null,
      userInteracted: false,
      // Set ONLY on countdown expiry: responses carry user_action tap|autocontinue.
      autoSubmit: false,
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
    this.stopCountdown()
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

        // The backend re-sends the plate_release_confirm jump (status = the
        // location) until answered; use it to pick up the countdown and the
        // location, and don't re-route on our own resends
        // (autocontinue_seconds <= 0 means wait for the user).
        if (parsedMessage.state === 'plate_release_confirm') {
          if (!this.location && typeof parsedMessage.status === 'string') {
            this.location = parsedMessage.status
          }
          if (Number.isFinite(parsedMessage.autocontinue_seconds) && parsedMessage.autocontinue_seconds > 0 && this.countdownInterval === null && !this.userInteracted) {
            this.startCountdown(Math.round(parsedMessage.autocontinue_seconds))
          }
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
    startCountdown(seconds) {
      this.countdown = seconds
      this.countdownInterval = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown -= 1;
        } else {
          this.stopCountdown()
          // Unattended: confirm and let the robot release the plate.
          this.autoSubmit = true
          this.handleButtonClick()
        }
      }, 1000);
    },
    stopCountdown() {
      if (this.countdownInterval) {
        clearInterval(this.countdownInterval);
        this.countdownInterval = null;
      }
    },
    cancelAutocontinue() {
      // Any tap on the page cancels the auto-continue countdown; the user must
      // then confirm explicitly. userInteracted also stops a resent jump from
      // re-arming the countdown.
      this.userInteracted = true
      this.stopCountdown()
    },
    handleButtonClick() {
      this.userInteracted = true
      this.stopCountdown()
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
          location: this.location,
          user_action: this.autoSubmit ? 'autocontinue' : 'tap'
        })
      })

      this.publisher.publish(message);
      this.autoSubmit = false;
    }
  }
}
</script>
