<template>
  <div class="page" @click="cancelAutocontinue">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Ready to transfer?</div>
      </div>
    </div>

    <div class="bd">
      <div class="simple-confirm">
        <p>The robot has picked up the drink.<br>Click 'Continue' to transfer the drink when ready.</p>
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
import { ref } from 'vue'
const allowedMessageType = 'std_msgs/String'
export default {
  data () {
    return {
      ros: null,
      username: USER,
      receivedMessage: '',
      inputMessage: '',
      listener: null,
      publisher: null,
      countdown: null,
      countdownInterval: null,
      userInteracted: false,
      // Set ONLY on countdown expiry: responses carry user_action tap|autocontinue.
      autoSubmit: false,
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

        // The routing jump is consumed by the previous page; the backend
        // re-sends it until answered so this page can pick up the countdown
        // (autocontinue_seconds <= 0 means wait for the user).
        if (parsedMessage.state === 'drink_confirm_transfer' && parsedMessage.status === 'jump') {
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
          // Unattended: confirm and let the transfer proceed.
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
    redirectToChangeItem () {
      this.$router.push('/robot_executing')
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
          state: 'drink_confirm_transfer',
          status: 'confirm',
          user_action: this.autoSubmit ? 'autocontinue' : 'tap'
        })
      })

      this.publisher.publish(message);
      this.autoSubmit = false;
    }
  }
}
</script>

