<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class="usertext">
        <div class="username">{{ username }}</div>
        <div class="userslog">Enjoy your mealtime now!</div>
      </div>
    </div>
    <div class="right">
      <div class="setting-container">
        <button @click="toggleSettings" class="settings-button">
          <img class="icon" alt="food" src="../assets/Vector.png">
          <span class="settings-button-text">Setting</span>
        </button>
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
        <img class="icon" alt="food" src="../assets/finish.png">
        <span class="finish-button-text">Finish Feature</span>
      </button>
    </div>
  </div>

  <div class="content">
    <div class="instruction">
      Message sent to Experimenter, they should be arriving shortly!
    </div>
    <div class="buttons">
      <button class="continue-button" @click="redirectToChangeItem">Return to Main Page</button>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';
export default {
  data () {
    return {
      ros: null,
      username: USER,
      showSettings: false,
      speed: 'moderate',
      publisher: null 
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber();
    this.initPublisher(); 
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
    initPublisher() {

      ros.on('connection', () => {
      });

      ros.on('error', (error) => {
      });

      ros.on('close', () => {
      });

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot', 
        messageType: 'std_msgs/String' 
      });

    },

    publishReturnToMain() {
      if (this.publisher) {
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'called_experimenter',
            status: 'return_to_main'
          })
        });
        this.publisher.publish(message);
      } else {
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

    redirectToChangeItem() {
      this.publishReturnToMain(); 
      this.$router.push('/bite_selection'); 
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
  margin-top: 25px;
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
  width: 34.1vw;
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
