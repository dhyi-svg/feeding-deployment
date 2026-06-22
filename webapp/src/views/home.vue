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
      <button class="continue-button" @click="handleButtonClick">Start Meal</button>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER } from '@/config/parameterConfig';

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
    handleRosMessage (message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          this.$router.push(route);
        }
      } catch (error) {
      }
    },
    handleButtonClick () {
      this.publishMessage();
      this.$router.push('/preference_context');
    },
    initPublisher () {
      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
    publishMessage () {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'home',
          status: 'move_to_above_plate'
        })
      })
      this.publisher.publish(message);
    },
    initSubscriber () {
      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })
      this.listener.subscribe((message) => {
        this.handleRosMessage(message);
      });
    }
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
  .right {
    display: flex;
    justify-content: center;
    align-items: center;
  }
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
}

.buttons {
  display: flex;
  gap: 20px;
  margin-top: 10px;
}

.continue-button {
  border: none;
  border-radius: 20px;
  color: black;
  background-color: #FFE699;
  width: 20vw;
  height: 12vh;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  font-family: Verdana;
  font-size: 30px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;
  padding: 10px;
}
</style>
