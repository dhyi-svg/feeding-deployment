<template>
  <div class="page" @click="cancelAutocontinue">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Robot arrived{{ location ? ' at the ' + location : '' }}</div>
      </div>
    </div>

    <div class="bd">
      <div class="confirm-body">
        <div class="cf-left">
          <strong>Is the robot parked where you want it?</strong>
          <p>If the position looks right, continue.<br><br>If not, choose adjust and drive the robot to exactly where you want it — it will remember the correction for next time.</p>
          <p class="cdown" :class="{ 'cdown-hidden': userInteracted || countdown === null }">Auto-confirming position in <span>{{ countdown }}s</span></p>
        </div>
        <div class="cf-right">
          <button class="btn lg amber w100" @click="handleOk">Position OK</button>
          <button class="btn lg ghost w100" @click="handleAdjust">Adjust Position</button>
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
      location: '',
      countdown: null,
      countdownInterval: null,
      userInteracted: false,
      // Set ONLY on countdown expiry: ok/adjust responses carry user_action tap|autocontinue.
      autoSubmit: false,
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
    // Tell the backend we've mounted/subscribed so it can stop re-sending the
    // (non-latched) jump. Location + autocontinue arrive via the jump message.
    this.publishStatus('ready')
    // any tap anywhere (incl. App.vue chrome/overlays outside .page) cancels autocontinue
    window.addEventListener('pointerdown', this.cancelAutocontinue, true)
  },
  beforeUnmount () {
    window.removeEventListener('pointerdown', this.cancelAutocontinue, true)
    this.stopCountdown()
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

        // The backend re-sends the jump until it sees our ready, then sends a
        // 'data' message (the first jump is consumed by the page that routed
        // here, so this page may only ever see the data message). Both carry
        // the location and the autocontinue duration.
        if (parsedMessage.state === 'nav_adjust' && (parsedMessage.status === 'jump' || parsedMessage.status === 'data')) {
          if (parsedMessage.status === 'jump') {
            this.publishStatus('ready')
          }
          if (parsedMessage.location) {
            this.location = parsedMessage.location
          }
          // autocontinue_seconds <= 0 means "wait for the user" (the
          // confirm_navigation_arrival preference is 'yes (wait for me)').
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
    initPublisher() {
      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
    publishStatus(status) {
      if (!this.publisher) {
        return
      }
      const payload = {
        state: 'nav_adjust',
        status: status
      }
      // 'ready' is a mount handshake, not a user decision -- no user_action.
      if (status !== 'ready') {
        payload.user_action = this.autoSubmit ? 'autocontinue' : 'tap'
        this.autoSubmit = false
      }
      const message = new ROSLIB.Message({
        data: JSON.stringify(payload)
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
    startCountdown(seconds) {
      this.countdown = seconds
      this.countdownInterval = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown -= 1;
        } else {
          this.stopCountdown()
          // Unattended: default to the safe no-op.
          this.autoSubmit = true
          this.handleOk()
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
    handleOk() {
      this.userInteracted = true
      this.stopCountdown()
      this.publishStatus('ok');
      this.$router.push('/robot_executing');
    },
    handleAdjust() {
      this.userInteracted = true
      this.stopCountdown()
      this.publishStatus('adjust');
      // Stay on this page: the backend routes us to the navigation teleop
      // screen (navigation_teleop / recover) via the routeMap handler above.
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
