<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="https://c.animaapp.com/jvBoNEN4/img/user.svg">
      <div class="usertext">
        <div class="username">{{ username }}</div>
        <div class="userslog">Enjoy your mealtime now!</div>
      </div>
    </div>
    <div class="right">
      <div class="setting-container">
<!--        <button @click="toggleSettings" class="settings-button">-->
<!--          <img class="icon" alt="food" src="../assets/Vector.png">-->
<!--          <span class="settings-button-text">Task Selection</span>-->
<!--        </button>-->
        <div v-if="showSettings" class="settings-panel">
          <h3>Speed:</h3>
          <div>
            <input type="radio" id="slow" name="speed" value="slow" v-model="speed" />
            <label for="slow">Slow</label>
          </div>
          <div>
            <input type="radio" id="moderate" name="speed" value="moderate" v-model="speed" checked />
            <label for="moderate">Moderate</label>
          </div>
          <div>
            <input type="radio" id="fast" name="speed" value="fast" v-model="speed" />
            <label for="fast">Fast</label>
          </div>
        </div>
      </div>
      <button class="finish-button" @click="redirectToChangeItemF">
        <img class="icon" alt="food" src="../assets/finish.png">
        <span class="finish-button-text">Finish Feeding</span>
      </button>
    </div>
  </div>

  <div class="content">
    <div class="buttons">
      <button class="action-button" @click="navigateToAddGesture">Add Gesture</button>
      <button class="action-button" @click="navigateToTestGesture">Test Gesture</button>
<!--      <button class="action-button" @click="navigateToQueryRobot">Edit or Query the Robot</button>-->
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from "@/router/routeMap";
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data() {
    return {
      username: USER,
      showSettings: false,
      speed: 'moderate',
      subscribeTopic: '/ServerComm',
      publishTopic: '/WebAppComm',
      listener: null,
      publisher: null,
    }
  },
  mounted () {
    this.initPublisher()
    this.initSubscriber()
    window.addEventListener('keydown', this.handleKeyDown)
  },
  beforeUnmount () {
    window.removeEventListener('keydown', this.handleKeyDown)
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

    if (this.publisher && this.publisher.ros) {
      this.publisher.ros.close();
      this.publisher.ros = null;

      setTimeout(() => {
        console.log('ROS connection should be fully closed now.');
      }, 1000);
    }
    next();
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
        url:  ROS_URL
      })

      this.publisher = new ROSLIB.Topic({
        ros: ros,
        name: this.publishTopic,
        messageType: 'std_msgs/String'
      })
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
    navigateToAddGesture() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({ // 将消息内容转换为JSON字符串
          state: 'gesture_main',
          status: 'add' // 使用输入框的内容作为status字段的值
        })
      })
      this.publisher.publish(message)
      this.$router.push('/gesturesetting')
    },
    navigateToTestGesture() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({ // 将消息内容转换为JSON字符串
          state: 'gesture_main',
          status: 'test' // 使用输入框的内容作为status字段的值
        })
      })
      this.publisher.publish(message)
      this.$router.push('/gesturemove2')
    },
    navigateToQueryRobot() {
      this.$router.push('/queryrobot')
    },
    redirectToChangeItemF() {
      this.$router.push('/notify')
    }
  }
}
</script>

<style scoped>
.buttons {
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.action-button {
  background-color: #FFE699;
  border: none;
  border-radius: 20px;
  color: black;
  padding: 10px 20px;
  cursor: pointer;
  font-family: Verdana;
  font-size: 30px;
  font-weight: 400;
  text-align: center;
  width: 30vw;
  height: 20vh;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.action-button:hover {
  background-color: #FFD966;
}
.voiceinputbox{
  font-family: Verdana;
  font-size: 18px;
  width: 70%; /* 输入框宽度调整为70%，可根据需要修改 */
  height: 40px;
  border-radius: 20px;
  padding: 5px 10px;
  border: 1px solid #ccc;
}

.voicebuttongroup{
  display: flex;
  justify-content: center;
  align-items: center;
}

.voice-start-button,
.voice-send-button {
  background-color: #FFE699;
  border-radius: 20px;
  width: 7vw;
  height: 8vh;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: 20px;
  border: none;
  cursor: pointer;
  transition: opacity 0.3s ease, transform 0.3s ease;
}

.voice-start-button img {
  width: 4vw;
  height: 5vh;
}

.voice-start-button:hover,
.voice-send-button:hover {
  opacity: 0.9;
  transform: scale(1.05);
}

.voice-start-button:active,
.voice-send-button:active {
  opacity: 0.8;
  transform: scale(0.98);
}
.custom-input-box {
  width: 30vw; /* 文本框宽度占页面宽度的 90% */
  height: 50vh; /* 文本框高度占容器的 100% */
  font-size: 1.2vw; /* 根据页面宽度自适应字体大小 */
  border: 1px solid #ccc;
  border-radius: 10px;
  padding: 10px;
  resize: none; /* 禁止用户手动调整大小 */
  box-sizing: border-box;
}

.option-container {
  max-height: 60vh; /* 设置选项框的最大高度 */
  overflow-y: auto;  /* 垂直滚动 */
  padding-right: 10px; /* 为滚动条留出空间 */
}

.voice{
  width: 59px;
  height: 57px;
  top: 611px;
  left: 1088px;
  gap: 0px;
  opacity: 0px;

}


.clear{
  width: 3vw;
  height: 4vh;
}
.tying-text{
  font-family: Verdana;
  font-size: 18px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;
  padding:10px;

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
.optionbox.selected {
  background-color: #6c7984; /* 选中后背景色 */
  color: white; /* 选中后文字颜色 */
}
.food{
  width: 43vw;
  height: 58vh;
}
.content{
  display: flex;
  //transform: scale(0.8);
  //height: 70vh;
  //align-items: center;
  justify-content: center;
  flex-flow: column;
}
.content-body {
  height: 80vh;
  display: flex;
  align-items: flex-start;
  //align-items: center;
  justify-content: space-between;
  //padding: 20px;
  margin-top: 0.5vh;
  .left{
    display: flex;
    flex-flow: column;
    justify-content: flex-start;
    align-items: flex-start;
    width: 37vw;
  }
  .left_text{
    font-family: Verdana;
    font-size: 18px;
    font-weight: 400;
    line-height: 24px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
    display: block;
    margin-top: 1vh;
    margin-bottom: 1vh;
  }
  .right{
    display: flex;
    flex-flow: column;
    justify-content: space-between;
    align-items: flex-start;
    margin-left: 2vw;
    width:50vw;
  }
  .right-first-title2{
    font-family: Verdana;
    margin-top: auto;
    font-size: 20px;
    font-weight: 700;
    //line-height: 25px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
    word-wrap: break-word; /* 自动换行 */
    white-space: normal;
    line-height: 1.2em;
    overflow-wrap: break-word;
  }
  .right-first-title{
    font-family: Verdana;
    font-size: clamp(12px, 2.5vh, 20px);
    margin-top: auto;
    font-weight: 700;
    //line-height: 25px;
    max-width: 12vw;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
    word-wrap: break-word; /* 自动换行 */
    white-space: normal;
    line-height: 1.2em;
    overflow-wrap: break-word;
  }
  .option-box-text{
    font-family: Verdana;
    font-size: 18px;
    font-weight: 700;
    line-height: 24px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
    padding: 10px;
  }
  .right-little-text{
    font-family: Verdana;
    font-size: 16px;
    font-weight: 700;
    line-height: 23px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
  .right-other-option-text{
    display: flex;
    align-items: flex-start;
    justify-content: flex-start;
    margin-top: 5px;
    flex-flow: column;
  }
  .right-other-option-little-text{
    font-family: Verdana;
    font-size: 18px;
    font-weight: 400;
    line-height: 23px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
  .buttonpart{
    width: 50vw;
    //height: 12vh;
    display: flex;
    //align-items: flex-start;
    align-items: center;
    justify-content: space-between;
    padding: 10px;
  }
}
.threeboxes{
  width: 43vw;
  height: 18vh;
  display: flex;
  //align-items: flex-start;
  align-items: start;
  justify-content: normal;
  //padding: 20px;
  overflow-x: auto;
}
.top {
  height: 10vh;
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
}
.button {
  background-color: #d9d9d9;
  border-radius: 20px;
  height: 48px;
  width: 112px;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: 20px;
  .icon {
    margin-right: 8px;
  }
}
.box{
  height:17vh;
  flex-grow: 1;
  flex-basis: 30%;
  aspect-ratio: 0.75;
  width: 12vw;
  //top: 200px;
  //left: 707px;
  min-height:15vh;
  min-width:12vw;
  max-width:15vw;
  margin: 1vh;
  display: flex;
  flex-flow: column;
  align-items: center;
  justify-content: space-between;
  //margin-right:15px;
  padding: 1px;
  gap: 0px;
  border-radius: 9px 9px 9px 9px;
  opacity: 0px;
  background: #F2F2F2;
  .metaballs{
    display: flex;
    flex-flow: column;
    align-items: center;
    //width: 100%;
    //height: 100%;
    margin: 0.5vh;
    object-fit: cover;
    //height: 13vh;
    //top: 210px;
    //left: 716px;
    gap: 0px;
    opacity: 0px;
    border-radius: 20px;
    padding: 0vh;
    align-self: center;
  }
}
.option{
  display: flex;
  //align-items: flex-start;
  justify-content: space-between;
  flex-flow: column;
  padding: 0px;

}
.optionbox{
  width: 50vw;
  //height: 12vh;
  top: 397px;
  left: 707px;
  gap: 0px;
  margin: 3px;
  border-radius: 20px 20px 20px 20px;
  border: 1px 0px 0px 0px;
  opacity: 0px;
  border: 1px solid #D3D3D3;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 5px;

}
.otheroption{
  display: flex;
  //align-items: flex-start;
  justify-content: space-between;
  flex-flow: column;
  padding: 0px;

}
.otheroptionbox{
  width: 50vw;
  top: 645px;
  left: 708px;
  gap: 0px;
  border-radius: 20px 20px 20px 20px;
  opacity: 0px;
  background: #D9D9D9;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 5px;
  margin: 0px;

}

.title{
  text-align: left;
  margin: 5px 5px 5px 100px;
}
.confirm-button-text{
  font-family: Verdana;
  font-size: 32px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;
  color: #000000;

}
.button1 {
  visibility: hidden;
  background-color: #d9d9d9;
  border-radius: 20px;
  height: 48px;
  width: 112px;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: 20px;
  .icon {
    margin-right: 8px;
  }
}

.button2 {
  margin-top: 2vh;
  background-color: #FFE699;
  border-radius: 20px;
  width: 26vw;
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
  cursor: pointer
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
  flex-direction: column;
  gap: 20px;
}


</style>
<!--<style scoped>-->
<!--.top {-->
<!--  height: 9vh;-->
<!--  background: #eee;-->
<!--  display: flex;-->
<!--  align-items: unset;-->
<!--  justify-content: space-between;-->
<!--  padding: 5px;-->
<!--  margin-bottom: 5px;-->
<!--}-->

<!--.left {-->
<!--  display: flex;-->
<!--  align-items: center;-->
<!--}-->

<!--.right {-->
<!--  display: flex;-->
<!--  align-items: center;-->
<!--}-->

<!--.usertext {-->
<!--  align-items: baseline;-->
<!--  display: flex;-->
<!--  justify-content: center;-->
<!--  flex-flow: column;-->
<!--  margin-left: 5px;-->
<!--}-->

<!--.username {-->
<!--  font-family: Verdana;-->
<!--  font-size: 20px;-->
<!--  font-weight: 400;-->
<!--  line-height: 18px;-->
<!--  text-align: left;-->
<!--}-->

<!--.userslog {-->
<!--  font-family: Verdana;-->
<!--  font-size: 16px;-->
<!--  font-weight: 400;-->
<!--  line-height: 18px;-->
<!--  text-align: left;-->
<!--}-->

<!--.setting-container {-->
<!--  position: relative;-->
<!--}-->

<!--.settings-button,-->
<!--.finish-button {-->
<!--  background-color: #6e7e8e;-->
<!--  border: none;-->
<!--  border-radius: 8px;-->
<!--  color: white;-->
<!--  padding: 10px 20px;-->
<!--  margin-left: 10px;-->
<!--  cursor: pointer;-->
<!--  font-size: 16px;-->
<!--  display: flex;-->
<!--  align-items: center;-->
<!--  height: 50px;-->
<!--}-->

<!--.settings-button span,-->
<!--.finish-button span {-->
<!--  margin-left: 5px;-->
<!--}-->

<!--.settings-panel {-->
<!--  position: absolute;-->
<!--  top: 120%;-->
<!--  left: 50%;-->
<!--  transform: translateX(-50%);-->
<!--  width: calc(90%);-->
<!--  max-width: 200px;-->
<!--  background-color: #6e7e8e;-->
<!--  border-radius: 8px;-->
<!--  color: white;-->
<!--  padding: 15px;-->
<!--  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);-->
<!--  text-align: left;-->
<!--}-->

<!--.settings-panel h3 {-->
<!--  margin-top: 0;-->
<!--}-->

<!--.settings-panel label {-->
<!--  margin-left: 5px;-->
<!--  font-size: 14px;-->
<!--}-->

<!--.content {-->
<!--  display: flex;-->
<!--  flex-direction: column;-->
<!--  justify-content: center;-->
<!--  align-items: center;-->
<!--  height: 80vh;-->
<!--}-->

<!--.buttons {-->
<!--  display: flex;-->
<!--  flex-direction: column;-->
<!--  gap: 20px;-->
<!--}-->

<!--.action-button {-->
<!--  background-color: #FFE699;-->
<!--  border: none;-->
<!--  border-radius: 20px;-->
<!--  color: black;-->
<!--  padding: 10px 20px;-->
<!--  cursor: pointer;-->
<!--  font-family: Verdana;-->
<!--  font-size: 20px;-->
<!--  font-weight: 400;-->
<!--  text-align: center;-->
<!--  width: 250px;-->
<!--  height: 60px;-->
<!--  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);-->
<!--}-->

<!--.action-button:hover {-->
<!--  background-color: #FFD966;-->
<!--}-->
<!--</style>-->

