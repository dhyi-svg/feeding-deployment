<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class = "usertext">
        <div class="username">{{ username }}</div>
        <div class = "userslog">Enjoy your mealtime now!</div>
      </div>
    </div>
    <div class="right">
      <div class="setting-container">

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
      <button class="finish-button">
        <img class = "icon" alt="food" src="../assets/finish.png">
        <span class = "finish-button-text">Finish Feeding</span>
      </button>
    </div>
  </div>

  <div class="content">
    <div class="content-body">
      
      <div class = "left">
        <span class = "left-text" style="display: block; text-align: left;">Well done, you finished!</span>
        <img class = "food" alt="food" src="../assets/food.png">
      </div>
      <div class = "right">
        <div class="notification-container">
          <div v-if="!messageSent">
            <p class="notification-text">Feeding complete!</p>
            <p class="notify-caregiver-text">Would you like to notify your caregiver?</p>
            <button class="notify-button" @click="handleButtonClick2">Notify Caregiver</button>
            <button class="notify-button" @click="handleButtonClick">Home</button>
          </div>
          <div v-else class="message-sent-container">
            <img src="../assets/deliver.png" alt="Message Icon" class="message-icon">
            <p class="message-sent-text">Message sent successfully. </p>
            <p class="message-sent-text">The caregiver is on their way.</p>
            <button class="notify-button" @click="handleButtonClick">Home</button>
          </div>
        </div>
      </div>

    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER} from '@/config/parameterConfig';
export default {
  data () {
    return {
      ros: null,
      username: USER,
      selectedOption: null,
      showSettings: false, 
      speed: 'moderate',
      messageSent: false,
      receivedMessage: '', 
      inputMessage: '', 
      subscribeTopic: '/robot_to_webapp', 
      publishTopic: '/talker', 
      listener: null, 
      publisher: null, 
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
    this.publishMessage()
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
          if (typeof route === 'string') {
            this.$router.push(route); 
          } else if (typeof route === 'object') {
            this.$router.push(route); 
          }
        }
        
      } catch (error) {
      }
    },
    publishSpeedSetting() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          command: 'set_speed',
          value: this.speed 
        }) 
      })
      this.publisher.publish(message);
    },
    handleButtonClick() {
      
      this.$router.push('/task_selection');
    },
    handleButtonClick2() {
      this.publishMessage2();
      this.messageSent = true
    },
    handleKeyDown (event) { 
      if (event.key === 'e' || event.key === 'E') {
        this.$router.go(-1)
      }
    },
    toggleSettings () {
      
      if (this.showSettings) {
        this.publishSpeedSetting();
      }
      this.showSettings = !this.showSettings; 
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
          
          status: 'finish_feeding'
        }) 
      })

      this.publisher.publish(message);
    },
    publishMessage2() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          status: 'call_caregiver'
        }) 
      })

      this.publisher.publish(message);
    }
  }
}
</script>

<style scoped>
.message-sent-text{
  font-family: Verdana;
  font-size: 26px;
  font-weight: 400;
  line-height: 5px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;

}
.message-sent-container{
  display: flex;
  flex-flow: column;
  align-items: flex-start;
  justify-content: space-between;
}
.notify-button{
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
  //margin-left: 20px;
  .icon {
    margin-right: 8px;
  }
  cursor: pointer;
  border: none;
  font-family: Verdana;
  font-size: 32px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;
  margin: 15px;
}
.notification-text{
  font-family: Verdana;
  font-size: 26px;
  font-weight: 400;
  line-height: 35px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;
}
.notify-caregiver-text{
  font-family: Verdana;
  font-size: 26px;
  font-weight: 700;
  line-height: 35px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;
}
.food{
  width: 57vw;
  height: 80vh;
}
.content{
  display: flex;
  //align-items: center;
  justify-content: center;
  flex-flow: column;
}
.content-body {
  display: flex;
  align-items: flex-start;
  //align-items: center;
  justify-content: space-between;
  //padding: 20px;
  margin-top: 23px;
  .left{
    display: flex;
    flex-flow: column;
    justify-content: flex-start;
    align-items: flex-start;
  }
  .left_text{
    font-family: Verdana;
    font-size: 18px;
    font-weight: 400;
    line-height: 24px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
    display: block;
    margin-top: 20px;
    margin-bottom: 7px;
  }
  .right{
    display: flex;
    flex-flow: column;
    align-items: flex-start;
    margin-left: 25px;
    justify-content: center;
    height: 50vh;
    width: 50vw;
  }
  .right-first-title{
    font-family: Verdana;
    font-size: 20px;
    font-weight: 700;
    line-height: 25px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
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
.left-text{
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 25px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;
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
.buttonpart{
  display: flex;
  //align-items: flex-start;
  align-items: center;
  justify-content: space-between;
  padding: 20px;
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
  background-color: #FFE699;
  border-radius: 20px;
  width: 215px;
  height: 63px;
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
}
</style>
