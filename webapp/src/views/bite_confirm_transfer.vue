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
      <div class="confirm-body">
        <div class="cf-left">
          <strong>Did the robot grab the bite successfully?</strong>
          <p>If the pickup looks correct, continue and the robot will bring it to your mouth.<br><br>If not, retry and it will try again.</p>
          <p class="cdown" :class="{ 'cdown-hidden': userInteracted || countdown === null }">Auto-continuing in <span>{{ countdown }}s</span></p>
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
    // NOTE: the physical button is intentionally NOT wired here. It is handled only
    // on the robot_executing page (to avoid confusing the user about whether to use
    // the on-screen button or the physical one). handleButtonClick() below is still
    // used by the on-screen "Continue — Transfer Bite" tap and the auto-continue
    // countdown.
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
    handleRosMessage(message) {

      try {
        const parsedMessage = JSON.parse(message.data);

        // The routing jump is consumed by the previous page; the backend
        // re-sends it until answered so this page can pick up the countdown
        // (autocontinue_seconds <= 0 means wait for the user).
        if (parsedMessage.state === 'bite_confirm_transfer' && parsedMessage.status === 'jump') {
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
          status: 'confirm',
          user_action: this.autoSubmit ? 'autocontinue' : 'tap'
        })
      })
      this.publisher.publish(message);
      this.autoSubmit = false;
    },
    publishMessage2() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_confirm_transfer',
          status: 'cancel',
          user_action: 'tap'
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
      this.userInteracted = true
      this.stopCountdown()
      this.publishMessage();
      this.$router.push('/robot_executing');
    },
    handleButtonClick2() {
      this.userInteracted = true
      this.stopCountdown()
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
  font: normal 4vh/1.3 Georgia, serif;
  color: var(--t);
  margin-bottom: 1.5vh;
}
.cf-left p {
  font-size: 2.4vh;
  color: var(--tm);
  line-height: 1.6;
}
.cf-right {
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
}
</style>
