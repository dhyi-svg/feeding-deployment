<template>
  <div class="page" @click="cancelAutocontinue">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">What would you like next?</div>
      </div>
    </div>

    <div class="bd">
      <div class="task-body">
        <div class="tg">
          <div class="tc-col">
            <div class="tc hi" @click="handleButtonClick">
              <div class="tc-i"><img src="../assets/for.png" alt="Bite"></div>
              <div class="tc-l">Take a Bite</div>
            </div>
            <p v-if="!autocontinueCancelled" class="cdown">Auto-confirming in <span>{{ countdown }}s</span></p>
          </div>
          <div class="tc" @click="handleButtonClickR">
            <div class="tc-i"><img src="../assets/drin.png" alt="Sip"></div>
            <div class="tc-l">Take a Sip</div>
          </div>
          <div class="tc" @click="handleButtonClickMouth">
            <div class="tc-i"><img src="../assets/Frame.png" alt="Wipe"></div>
            <div class="tc-l">Wipe Mouth</div>
          </div>
        </div>
        <div class="brow">
          <button class="btn md ghost w100" @click="$router.push('/personalization')">Personalization</button>
          <button class="btn md danger w100" @click="handleFinishFeeding">Finish Feeding</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      ros: null,
      username: USER,
      countdown: 1000,
      countdownInterval: null,
      // Set when the user opens the settings overlay: cancels the on-screen
      // autocontinue for this page visit (not re-engaged on close).
      autocontinueCancelled: false,
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.startCountdown();
    this.initSubscriber()
    this.initPublisher()
    window.addEventListener('settings-open', this.onSettingsOpen)
  },
  beforeUnmount () {
    window.removeEventListener('settings-open', this.onSettingsOpen)
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval);
      this.countdownInterval = null;
    }
  },
  beforeRouteLeave (to, from, next) {
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval);
      this.countdownInterval = null;
    }
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
    startCountdown() {
      // Don't re-arm once the user has opened settings on this page visit.
      if (this.autocontinueCancelled) return;
      this.countdownInterval = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown -= 1;
        } else {
          clearInterval(this.countdownInterval);
          this.handleButtonClick();
        }
      }, 1000);
    },
    cancelAutocontinue() {
      // Any tap on the page (or opening settings) cancels the on-screen
      // autocontinue for this visit; it is not re-armed.
      this.autocontinueCancelled = true;
      if (this.countdownInterval) {
        clearInterval(this.countdownInterval);
        this.countdownInterval = null;
      }
    },
    onSettingsOpen() {
      // User entered settings: stop auto-advancing and wait for an explicit tap.
      this.cancelAutocontinue();
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
    handleRosMessage(message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        if (parsedMessage.state === 'auto_time' && parsedMessage.status) {
          if (this.countdownInterval) {
            clearInterval(this.countdownInterval);
            this.countdownInterval = null;
          }

          this.countdown = parseInt(parsedMessage.status, 10);

          this.startCountdown();
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
    handleButtonClickR() {
      this.publishMessageD();
      this.$router.push('/robot_executing');
    },
    handleButtonClick() {
      this.publishMessageR();
      this.$router.push('/robot_executing');
    },
    handleButtonClickMouth() {
      this.publishMessagePhysical();
      this.$router.push('/robot_executing');
    },
    publishMessageD() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'take_sip'
        })
      })
      this.publisher.publish(message);
    },
    publishMessageR() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'take_bite'
        })
      })
      this.publisher.publish(message);
    },
    publishMessagePhysical() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'mouth_wiping'
        })
      })
      this.publisher.publish(message);
    },
    handleFinishFeeding() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'finish_feeding'
        })
      })
      this.publisher.publish(message);
      this.$router.push('/robot_executing');
    },
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      });
    },
  }
}
</script>
