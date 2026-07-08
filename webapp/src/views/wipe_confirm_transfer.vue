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
        <p>The robot has picked up the mouth wipe.<br>Click 'Continue' to wipe your mouth when ready.</p>
        <p v-if="!userInteracted && countdown !== null" class="cdown">Auto-continuing in <span>{{ countdown }}s</span></p>
        <button class="btn lg amber" style="min-width:24vw" @click="redirectToChangeItem()">Continue</button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      ros: null,
      username: USER,
      publisher: null,
      countdown: null,
      countdownInterval: null,
      userInteracted: false,
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher();
    this.initSubscriber()
  },
  beforeRouteLeave (to, from, next) {
    this.stopCountdown()
    next();
  },
  methods: {
    initSubscriber() {

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })

      this.listener.subscribe((msg) => {

        try {
          const parsedData = JSON.parse(msg.data);

          // The routing jump is consumed by the previous page; the backend
          // re-sends it until answered so this page can pick up the countdown
          // (autocontinue_seconds <= 0 means wait for the user).
          if (parsedData.state === 'wipe_confirm_transfer' && parsedData.status === 'jump') {
            if (Number.isFinite(parsedData.autocontinue_seconds) && parsedData.autocontinue_seconds > 0 && this.countdownInterval === null && !this.userInteracted) {
              this.startCountdown(Math.round(parsedData.autocontinue_seconds))
            }
            return
          }

          if (parsedData.state === 'prepare_mouth_wiping' && parsedData.status === 'completed') {
            this.$router.push('/wipe_confirm_transfer');
          }
          const route = routeMap[parsedData.state]?.[parsedData.status];
          if (route) {
            if (typeof route === 'string') {
              this.$router.push(route);
            } else if (typeof route === 'object') {
              this.$router.push(route);
            }
          }
        } catch (error) {
        }
      })
    },
    startCountdown(seconds) {
      this.countdown = seconds
      this.countdownInterval = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown -= 1;
        } else {
          this.stopCountdown()
          // Unattended: confirm and let the transfer proceed.
          this.redirectToChangeItem()
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
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot', 
        messageType: 'std_msgs/String' 
      });

    },

    publishReturnToMain() {
      if (this.publisher) {
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'wipe_confirm_transfer',
            status: 'confirm'
          })
        });
        this.publisher.publish(message);
      } else {
      }
    },
    redirectToChangeItem() {
      this.userInteracted = true
      this.stopCountdown()
      this.publishReturnToMain();
      this.$router.push('/robot_executing');
    }
  }
}
</script>


