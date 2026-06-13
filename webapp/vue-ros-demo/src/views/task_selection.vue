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
<!--      <div class="setting-container">-->
<!--        <button @click="toggleSettings" class="settings-button">-->
<!--          <img class = "icon" alt="food" src="../assets/Vector.png">-->
<!--          <span class="settings-button-text">Task Selection</span>-->
<!--        </button>-->
<!--        <div v-if="showSettings" class="settings-panel">-->
<!--          <h3>Speed:</h3>-->
<!--          <div>-->
<!--            <input type="radio" id="slow" name="speed" value="slow" />-->
<!--            <label for="slow">Slow</label>-->
<!--          </div>-->
<!--          <div>-->
<!--            <input type="radio" id="moderate" name="speed" value="moderate" checked />-->
<!--            <label for="moderate">Moderate</label>-->
<!--          </div>-->
<!--          <div>-->
<!--            <input type="radio" id="fast" name="speed" value="fast" />-->
<!--            <label for="fast">Fast</label>-->
<!--          </div>-->
<!--        </div>-->
<!--      </div>-->
      <button class="finish-button">
        <img class = "icon" alt="food" src="../assets/finish.png">
        <span class="finish-button-text" @click="redirectToChangeItemF">Finish Feeding</span>
      </button>
    </div>
  </div>

  <div class="content">
    <div class="buttons">
      <div class="button2">
        <button class="button3" @click="handleButtonClick">
          <img class="button-drink" alt="Vue logo" src="../assets/for.png">
          Take a Bite
        </button>
      </div>
      <div class="button2">
        <button class="button3" @click="handleButtonClickR">
          <img class="button-drink" alt="Vue logo" src="../assets/drin.png">
          Take a Sip
        </button>
      </div>
      <div class="button2">
        <button class="button3" @click="handleButtonClickMouth" :class="{ 'active': isActiveMouth }">
          <img class="button-drink" alt="Vue logo" src="../assets/Frame.png">
          Wipe Mouth
        </button>
      </div>
    </div>
    <div class="bottom-buttons">
      <button class="sub-button" @click="navigateToTrans()">
        <img class="sub-button-icon" src="../assets/trans.png" alt="Adaptability Icon" />
        Transparency
      </button>
      <button class="sub-button" @click="navigateToAda()">
        <img class="sub-button-icon" src="../assets/ada.png" alt="Adaptability Icon" />
        Adaptability
      </button>
      <button class="sub-button" @click="navigateToGes()">
        <img class="sub-button-icon" src="../assets/ges.png" alt="Adaptability Icon" />
        Gestures
      </button>
      <button class="sub-button" @click="navigateToTeleop()">
        <img class="sub-button-icon" src="../assets/ges.png" alt="Manual Control Icon" />
        Manual Control
      </button>
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
      username: USER,
      isActiveMouth: false,
      showSettings: false,
      speed: 'moderate',
      countdownInterval: null
    }
  },
  mounted () {
    this.initSubscriber()
    this.initPublisher()
    // this.publishSpeedSetting()
    window.addEventListener('keydown', this.handleKeyDown) // notify caregiver
  },
  beforeUnmount () {
    window.removeEventListener('keydown', this.handleKeyDown) // notify caregiver
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval); // Clear the interval when component is about to unmount
    }
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
    navigateToGes() {
      this.publishMessageG()
      this.$router.push('/gesturemain');
    },
    navigateToTrans() {
      this.publishMessageT()
      this.$router.push('/robotbehavior');
    },
    navigateToAda() {
      this.publishMessageA()
      this.$router.push('/fixedconfigurations');
    },
    navigateToTeleop() {
      this.publishMessageTeleop()
      this.$router.push('/teleop');
    },
    handleButtonClickR() {
      this.publishMessageD();
      this.$router.push('/swithtodrink');
    },
    handleButtonClick() {
      this.publishMessageR();
      this.$router.push('/preparepickup2');
    },
    handleButtonClickMouth() {
      this.publishMessagePhysical();
      this.$router.push('/wiping');
    },
    publishMessageG() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'gesture'
        }) // publish json
      })
      this.publisher.publish(message);
    },
    publishMessageD() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'take_sip'
        }) // publish json
      })
      this.publisher.publish(message);
    },
    publishMessageR() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'take_bite'
        }) // publish json
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
    publishMessageT() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'transparency'
        })
      })
      this.publisher.publish(message);
    },
    publishMessageA() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'adaptability'
        })
      })

      this.publisher.publish(message);
    },
    publishMessageTeleop() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'task_selection',
          status: 'teleop_recovery'
        })
      })
      this.publisher.publish(message);
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
    toggleSettings() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({ // 将消息内容转换为JSON字符串
          state: 'task_selection',
          status: 'jump' // 使用输入框的内容作为status字段的值
        })
      })
      this.publisher.publish(message)
      this.$router.push('/task_selection')
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
      this.$router.push('/swithtodrink')
    },
    redirectToChangeItemBack () {
      this.publishResumeFeeding();
      this.$router.go(-1)
    },
    redirectToChangeItemNext () {
      this.publishCallNext();
      this.$router.push('/wipping')
    },
    redirectToChangeItemF () {
      this.$router.push('/notify')
    },
  }
}
</script>

<style scoped>
.bottom-buttons {
  margin-top: 20px;
  display: flex;
  gap: 20px;
  max-width: 90vw;
  max-height: 90vh;
  width: 100%;
  align-items: baseline;
  justify-content: space-between;
}

.sub-button {
  background-color: #ffe699;
  border: none;
  border-radius: 10px;
  padding: 10px 20px;
  font-family: Verdana;
  font-size: 3vw;
  //font-weight: bold;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  min-width: 150px;
  width: 41vw;
  height: 19vh;
}

.sub-button-icon {
  height: 15vh;
  width: 7vw;
  margin-right: 10px;
}
.button-text {
  font-family: Verdana;
  font-size: 16px;
  color: black;
  text-align: left;
  width:85vw;
  padding-top: 10px; /* Adjust padding to better position the text under the button */
}
.button3 {
  background-color: #FFE699;
  border-radius: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #FFE699;
  border-radius: 20px;
  width: 25vw;
  height: 40vh;
  top: 740px;
  left: 924px;
  gap: 0px;
  opacity: 0px;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-flow: column;
  .icon {
    margin-right: 8px;
  }
  .button-drink{
    height:18vh;
    width: 14vw;
  }
  .button-setting{
    height:7vh;
    width: 4vw;
    margin:10px
  }
  border: none;
  font-family: Verdana;
  font-size: 3vw;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;

}
.button2 {
  background-color: #FFE699;
  border-radius: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #FFE699;
  border-radius: 20px;
  width: 29vw;
  height: 40vh;
  top: 740px;
  left: 924px;
  gap: 0px;
  opacity: 0px;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-flow: column;
  .icon {
    margin-right: 8px;
  }
  .button-drink{
    height:25vh;
    width: 20vw;
  }
  .button-setting{
    height:7vh;
    width: 4vw;
    margin:10px
  }
  font-family: Verdana;
  font-size: 20px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;
}
.icon-for{
  height: 40vh;
  width: 40vw;
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
  max-width: 90vw;
  max-height: 90vh;
  width: 100%;
  align-items: baseline;
  justify-content: space-between;
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
