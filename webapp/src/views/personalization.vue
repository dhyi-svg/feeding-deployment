<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Customize how the robot works</div>
      </div>
    </div>

    <div class="bd">
      <div class="pers-body">
        <div class="pcard" @click="navigateToTrans()">
          <div class="p-ico"><img src="../assets/trans.png" alt="Transparency"></div>
          <div>
            <div class="p-nm">Transparency</div>
            <div class="p-ds">Ask the robot why it made a choice</div>
          </div>
        </div>
        <div class="pcard" @click="navigateToAda()">
          <div class="p-ico"><img src="../assets/ada.png" alt="Adaptability"></div>
          <div>
            <div class="p-nm">Adaptability</div>
            <div class="p-ds">Change how the robot behaves</div>
          </div>
        </div>
        <div class="pcard" @click="navigateToGes()">
          <div class="p-ico"><img src="../assets/ges.png" alt="Gestures"></div>
          <div>
            <div class="p-nm">Gestures</div>
            <div class="p-ds">Control the robot with gestures</div>
          </div>
        </div>
        <button class="btn sm ghost w100" style="margin-top:1vh" @click="$router.push('/task_selection')">
          ← Back to Task Selection
        </button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER } from '@/config/parameterConfig';

export default {
  data() {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
  },
  beforeRouteLeave(to, from, next) {
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
          this.$router.push(route);
        }
      } catch (error) {
      }
    },
    initPublisher() {
      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      });
    },
    navigateToTrans() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'transparency'
        })
      })
      this.publisher.publish(message);
      this.$router.push('/transparency');
    },
    navigateToAda() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'adaptability'
        })
      })
      this.publisher.publish(message);
      this.$router.push('/adaptability');
    },
    navigateToGes() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'gesture'
        })
      })
      this.publisher.publish(message);
      this.$router.push('/gesture_menu');
    },
  }
}
</script>

<style scoped>
.pers-body {
  flex: 1;
  display: flex;
  flex-direction: column;
  justify-content: center;
  gap: 2.2vh;
  max-width: 1000px;
  width: 100%;
  margin: 0 auto;
}

.pcard {
  background: var(--s2);
  border-radius: var(--rl);
  height: 15vh;
  display: flex;
  align-items: center;
  gap: 2vw;
  padding: 0 2.5vw;
  cursor: pointer;
  border: 2px solid transparent;
}

.p-ico {
  width: 9vh;
  height: 9vh;
  border-radius: 50%;
  background: rgba(240, 165, 0, .13);
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.p-ico img {
  width: 50%;
  height: 50%;
  object-fit: contain;
  filter: invert(1);
}

.p-nm {
  font-size: 2.9vh;
  font-weight: 700;
  color: var(--t);
  margin-bottom: 0.4vh;
}

.p-ds {
  font-size: 2.1vh;
  color: var(--tm);
}

.back-btn {
  margin-top: 1vh;
}
</style>
