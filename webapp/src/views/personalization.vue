<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class="usertext">
        <div class="username">{{ username }}</div>
        <div class="userslog">Enjoy your mealtime now!</div>
      </div>
    </div>
  </div>

  <div class="content">
    <div class="buttons">
      <button class="pers-button" @click="navigateToTrans()">
        <img class="button-icon" src="../assets/trans.png" alt="Transparency Icon" />
        Transparency
      </button>
      <button class="pers-button" @click="navigateToAda()">
        <img class="button-icon" src="../assets/ada.png" alt="Adaptability Icon" />
        Adaptability
      </button>
      <button class="pers-button" @click="navigateToGes()">
        <img class="button-icon" src="../assets/ges.png" alt="Gestures Icon" />
        Gestures
      </button>
    </div>
    <div class="back-row">
      <button class="back-button" @click="$router.push('/task_selection')">
        Back to Task Selection
      </button>
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
.top {
  height: 9vh;
  background: #eee;
  display: flex;
  align-items: unset;
  justify-content: space-between;
  padding: 5px;
  margin-bottom: 5px;
  .left {
    display: flex;
    justify-content: space-between;
    padding: 15px;
  }
  .usertext {
    align-items: baseline;
    display: flex;
    justify-content: center;
    flex-flow: column;
    margin-left: 5px;
  }
  .username {
    font-family: Verdana;
    font-size: 20px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
  .userslog {
    font-family: Verdana;
    font-size: 16px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
}

.content {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  height: 80vh;
  gap: 30px;
}

.buttons {
  display: flex;
  gap: 20px;
  max-width: 90vw;
  width: 100%;
  justify-content: space-between;
}

.pers-button {
  background-color: #FFE699;
  border: none;
  border-radius: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-flow: column;
  flex: 1;
  height: 40vh;
  font-family: Verdana;
  font-size: 3vw;
  font-weight: 400;
  cursor: pointer;
}

.button-icon {
  height: 18vh;
  width: 10vw;
  margin-bottom: 10px;
}

.back-row {
  width: 100%;
  max-width: 90vw;
  display: flex;
  justify-content: flex-start;
}

.back-button {
  background-color: #d9d9d9;
  border: none;
  border-radius: 12px;
  padding: 12px 28px;
  font-family: Verdana;
  font-size: 1.5vw;
  cursor: pointer;
}
</style>
