<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="https://c.animaapp.com/jvBoNEN4/img/user.svg">
      <div class = "usertext">
        <div class="username">{{ username }}</div>
        <div class = "userslog">Enjoy your mealtime now!</div>
      </div>
    </div>
    <div class="right">
      <div class="setting-container">
<!--        <button @click="toggleSettings" class="settings-button">-->
<!--          <img class = "icon" alt="food" src="../assets/Vector.png">-->
<!--          <span class = "settings-button-text">Task Selection</span>-->
<!--        </button>-->
      </div>
      <button class="finish-button">
        <img class = "icon" alt="food" src="../assets/finish.png">
        <span class = "finish-button-text" @click ="redirectToChangeItemF">Finish Feeding</span>
      </button>
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
import { ROS_URL, USER} from '@/config/parameterConfig';
const allowedMessageType = 'std_msgs/String'
export default {
  data () {
    return {
      username: USER,
      showSettings: false,
      speed: 'moderate',
      receivedMessage: '', // Receive ROS messages
      inputMessage: '', // the input box used to send the ROS message
      subscribeTopic: '/ServerComm', // Subscribed ROS topics
      publishTopic: '/WebAppComm', // pubilsh
      listener: null, // listener
      publisher: null, // publish
    }
  },
  mounted () {
    this.initSubscriber()
    this.initPublisher()
    window.addEventListener('keydown', this.handleKeyDown) // jump tonotify caregiver
  },
  beforeUnmount () {
    window.removeEventListener('keydown', this.handleKeyDown) // jump tonotify caregiver
  },
  beforeRouteLeave (to, from, next) {
    if (this.listener) {
      console.log('Unsubscribing from listener...');
      this.listener.unsubscribe();
      this.listener = null;
    }

    if (this.publisher) {
      console.log('Unadvertising publisher...');
      this.publisher.unadvertise();
      this.publisher = null;
    }

    if (this.publisher && this.publisher.ros) {
      console.log('Closing ROS connection...');
      this.publisher.ros.close();
      this.publisher.ros = null;

      setTimeout(() => {
        console.log('ROS connection should be fully closed now.');
      }, 1000);
    }
    next();
  },
  methods: {
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
        console.error('Failed to parse ROS message:', error);
      }
    },
    toggleSettings() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'jump'
        })
      })
      this.publisher.publish(message)
      this.$router.push('/task_selection')
    },
    publishSpeedSetting() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          command: 'set_speed',
          value: this.speed
        })
      })
      this.publisher.publish(message)
    },
    handleButtonClick() {
      this.publishMessage();
      this.$router.push('/preparepickup');
    },
    handleKeyDown (event) { // jump tonotify caregiver
      if (event.key === 'e' || event.key === 'E') {
        this.$router.push({ name: 'physical' })
      }
    },
    redirectToChangeItem () {
      this.$router.push('/preparepickup')
    },
    redirectToChangeItemF () {
      this.$router.push('/notify')
    },
    initPublisher() {
      const ros = new ROSLIB.Ros({
        url: ROS_URL
      })
      this.publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/WebAppComm',
        messageType: 'std_msgs/String'
      })
    },
    publishMessage() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'home',
          status: 'move_to_above_plate'
        })
      })
      console.log(message.data)
      this.publisher.publish(message);
    },
    initSubscriber() {
      const ros = new ROSLIB.Ros({
        url: ROS_URL
      })
      this.listener = new ROSLIB.Topic({
        ros: ros,
        name: this.subscribeTopic, // 订阅的 ROS 话题
        messageType: 'std_msgs/String' // 订阅 std_msgs/String 类型的消息
      })
      this.listener.subscribe((message) => {
        console.log('Received message:', message.data);
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
  .food {
    width: 500px;
    height: 200px;
    top: 179px;
    left: 68px;
    gap: 0px;
    opacity: 0px;
  }
  .right {
    display: flex;
    justify-content: center;
    align-items: center;
    .settings-button-text{
      font-family: Verdana;
      font-size: 18px;
      font-weight: 400;
      line-height: 24px;
      letter-spacing: 0.17499999701976776px;
      text-align: left;
    }
    .finish-button-text{
      font-family: Verdana;
      font-size: 18px;
      font-weight: 400;
      line-height: 24px;
      letter-spacing: 0.17499999701976776px;
      text-align: left;
    }
    .setting-container {
      position: relative;
    }
    .settings-button,
    .finish-button {
      background-color: #6e7e8e;
      border: none;
      border-radius: 8px;
      color: white;
      padding: 10px 20px;
      margin-left: 10px;
      cursor: pointer;
      font-size: 16px;
      display: flex;
      align-items: center;
      height: 50px;
    }
    .settings-button span,
    .finish-button span {
      margin-left: 5px;
    }
    .settings-panel {
      position: absolute;
      top: 120%;
      left: 50%;
      transform: translateX(-50%);
      width: calc(90%); /* 宽度设置为 Setting 按钮宽度的 90% */
      max-width: 200px; /* 可以设置一个最大宽度以防止过大 */
      background-color: #6e7e8e;
      border-radius: 8px;
      color: white;
      padding: 15px;
      box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
      text-align: left;
    }
    .settings-panel h3 {
      margin-top: 0;
    }
    .settings-panel label {
      margin-left: 5px;
      font-size: 14px;
    }
  }
  .left {
    display: flex;
    justify-content: space-between;
    padding:15px
  }
  .usertext{
    align-items: baseline;
    display: flex;
    justify-content: center;
    flex-flow: column;
    margin-left: 5px;
  }
  .username{
    font-family: Verdana;
    font-size: 20px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
  .userslog{
    font-family: Verdana;
    font-size: 16px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
}

.left {
  display: flex;
  align-items: center;
}

.right {
  display: flex;
  align-items: center;
}

.setting-container {
  position: relative;
}

.settings-button,
.finish-button {
  background-color: #6e7e8e;
  border: none;
  border-radius: 8px;
  color: white;
  padding: 10px 20px;
  margin-left: 10px;
  cursor: pointer;
  font-size: 16px;
  display: flex;
  align-items: center;
  height: 50px;
}

.settings-button span,
.finish-button span {
  margin-left: 5px;
}

.settings-panel {
  position: absolute;
  top: 120%;
  left: 50%;
  transform: translateX(-50%);
  width: calc(90%);
  max-width: 200px;
  background-color: #6e7e8e;
  border-radius: 8px;
  color: white;
  padding: 15px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
  text-align: left;
}

.settings-panel h3 {
  margin-top: 0;
}

.settings-panel label {
  margin-left: 5px;
  font-size: 14px;
}

.content {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  height: 80vh;
}

.instruction {
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
  line-height: 30px;
  letter-spacing: 0.17499999701976776px;
  text-align: center;
  width: 80vw
}

.buttons {
  display: flex;
  gap: 20px;
  margin-top: 10px;
}

.continue-button,
.retry-button {
  //background-color: #fce69e;
  border: none;
  border-radius: 8px;
  color: black;
  padding: 10px 20px;
  cursor: pointer;
  //font-size: 16px;
  display: flex;
  align-items: center;
  //height: 40px;
  background-color: #FFE699;
  border-radius: 20px;
  width: 20vw;
  height: 12vh;
  top: 740px;
  left: 924px;
  gap: 0px;
  opacity: 0px;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: 20px;
  .icon {
    margin-right: 8px;
  }
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
