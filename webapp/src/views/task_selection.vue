<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class = "usertext">
        <div class="username">{{ username }}</div>
        <div class = "userslog">Enjoy your mealtime now!</div>
      </div>
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
      <button class="wide-button" @click="$router.push('/personalization')">
        Personalization
      </button>
      <button class="wide-button" @click="handleFinishFeeding">
        Finish Feeding
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
      ros: null,
      username: USER,
      isActiveMouth: false,
      countdownInterval: null
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
    
  },
  beforeUnmount () {
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval); 
    }
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
  }
}
</script>

<style scoped>
.bottom-buttons {
  margin-top: 20px;
  display: flex;
  gap: 20px;
  max-width: 90vw;
  width: 100%;
  justify-content: space-between;
}

.wide-button {
  background-color: #ffe699;
  border: none;
  border-radius: 16px;
  font-family: Verdana;
  font-size: 2.5vw;
  font-weight: 400;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  flex: 1;
  height: 12vh;
}
.button-text {
  font-family: Verdana;
  font-size: 16px;
  color: black;
  text-align: left;
  width:85vw;
  padding-top: 10px; 
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
