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
<!--          <span class = "settings-button-text">Setting</span>-->
<!--        </button>-->
        <div v-if="showSettings" class="settings-panel">
          <h3>Speed:</h3>
          <div>
            <input type="radio" id="slow" name="speed" value="slow" />
            <label for="slow">Slow</label>
          </div>
          <div>
            <input type="radio" id="moderate" name="speed" value="moderate" checked />
            <label for="moderate">Moderate</label>
          </div>
          <div>
            <input type="radio" id="fast" name="speed" value="fast" />
            <label for="fast">Fast</label>
          </div>
        </div>
      </div>
      <button class="finish-button">
        <img class = "icon" alt="food" src="../assets/finish.png">
        <span class = "finish-button-text">Finish Feeding</span>
      </button>
    </div>
  </div>

  <div class="content">
    <div class="physical-button-pressed">
      Physical Button Pressed
    </div>
    <div class="instruction">
      Anomaly detected. How would you like to proceed?
    </div>
    <div class="buttons">
      <div class="button-container">
        <button class="icon-button">
          <img src="../assets/call-center.png" alt="Call Experimenter">
        </button>
        <button class="text-button" @click="redirectToChangeItem">Call Experimenter</button>
      </div>
      <div class="button-container">
        <button class="icon-button">
          <img class = "icon-for" src="../assets/for.png" alt="Resume Feeding">
        </button>
        <button class="text-button" @click="redirectToChangeItemBack">Resume Feeding</button>
      </div>
      <div class="button-container">
        <button class="icon-button">
          <img src="../assets/Frame.png" alt="Mouth Wiping">
        </button>
        <button class="text-button" @click="redirectToChangeItemNext">Mouth Wiping</button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib"
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      username: USER,
      showSettings: false,
      speed: 'moderate'
    }
  },
  mounted () {
    this.initSubscriber()
    this.initPublisher()
    window.addEventListener('keydown', this.handleKeyDown) // notify caregiver
  },
  beforeUnmount () {
    window.removeEventListener('keydown', this.handleKeyDown) // notify caregiver
  },
  methods: {
    initSubscriber() {
      const ros = new ROSLIB.Ros({
        url:  ROS_URL // changedURL
      })

      this.listener = new ROSLIB.Topic({
        ros: ros,
        name: '/ServerComm', // 订阅 /listener 话题
        messageType: 'std_msgs/String' // 订阅 std_msgs/String 类型的消息
      })
      this.listener.subscribe((message) => {
        console.log('Received message:', message.data);
        this.handleRosMessage(message);
      });
    },
    handleRosMessage(message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          if (typeof route === 'string') {
            this.$router.push(route); // string
          } else if (typeof route === 'object') {
            this.$router.push(route); // object
          }
        }
      } catch (error) {
        console.error('Failed to parse received message:', error);
      }
    },
    initPublisher() {
      const ros = new ROSLIB.Ros({
        url:  ROS_URL // changedURL
      });

      this.publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/WebAppComm', // 发布到指定话题
        messageType: 'std_msgs/String' // 发布 std_msgs/String 类型的消息
      });
    },
    beforeRouteLeave (to, from, next) {
      if (this.listener) {
        console.log('Unsubscribing from listener...');
        this.listener.unsubscribe();
        this.listener = null;
      }

      // 取消发布
      if (this.publisher) {
        console.log('Unadvertising publisher...');
        this.publisher.unadvertise();
        this.publisher = null;
      }

      // 断开ROS连接
      if (this.publisher && this.publisher.ros) {
        console.log('Closing ROS connection...');
        this.publisher.ros.close();
        this.publisher.ros = null;

        // 延迟一段时间确保连接彻底关闭
        setTimeout(() => {
          console.log('ROS connection should be fully closed now.');
        }, 1000);
      }
      next(); // 继续路由导航
    },

    publishResumeFeeding() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'emergency_stop',
          status: 'back'
        })
      });
      this.publisher.publish(message);
      console.log('Published message:', message);
    },
    publishSpeedSetting() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          command: 'set_speed',
          value: this.speed // 使用当前选择的速度
        }) // publish json
      })
      this.publisher.publish(message);
    },
    toggleSettings () {
      // 如果面板将被关闭，则发送速度设置消息
      if (this.showSettings) {
        this.publishSpeedSetting();
      }
      this.showSettings = !this.showSettings; // 切换显示与隐藏
    },
    handleKeyDown (event) { // notify caregiver
      if (event.key === 'e' || event.key === 'E') {
        this.$router.go(-1)
      }
    },
    publishCallCaregiver() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          status: 'call_caregiver'
        })
      });
      this.publisher.publish(message);
      console.log('Published message:', message);
    },
    publishCallNext() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          status: 'mouth_wiping'
        })
      });
      this.publisher.publish(message);
      console.log('Published message:', message);
    },
    redirectToChangeItem () {
      this.publishCallCaregiver();
      this.$router.push('/callbeforetransfer')
    },
    redirectToChangeItemBack () {
      this.publishResumeFeeding();
      this.$router.go(-1)
    },
    redirectToChangeItemNext () {
      this.publishCallNext();
      this.$router.push('/wipping')
    },
  }
}
</script>

<style scoped>
.icon-for{
  height: 20vh;
  width: 16vw;
}
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
  position: relative;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  height: 80vh;
  padding: 20px;
}

.physical-button-pressed {
  position: absolute;
  top: 0;
  left: 0;
  font-weight: bold;
  margin-bottom: 10px;
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 22px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;
  margin-left: 25px;
  margin-top: 23px;

}

.instruction {
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
  line-height: 60px;
  letter-spacing: 0.17499999701976776px;
  text-align: center;
  width: 80vw
}

.buttons {
  display: flex;
  gap: 20px;
}

.button-container {
  display: flex;
  flex-direction: column;
  align-items: center;
}

.icon-button {
  background-color: #fce69e;
  border: none;
  border-radius: 8px;
  padding: 20px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 10px;
  height: 27.6vh;
  width: 29.5vw;
}

.text-button {
  background-color: #fce69e;
  border: none;
  border-radius: 8px;
  padding: 10px 20px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  width: 29.5vw;
  height: 12vh;
  font-family: Verdana;
  font-size: 32px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;

}
</style>
