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

      </div>
      <button class="finish-button">
        <img class = "icon" alt="food" src="../assets/finish.png">
        <span class = "finish-button-text" @click ="redirectToChangeItemF">Finish Feeding</span>
      </button>
    </div>
  </div>

  <div class="content">
    <div class="message">
      {{ displayedMessage }}
    </div>
  </div>

  <div class="footer">
    <button class="succeed-button" @click="redirectToChangeItem">Next</button>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';
export default {
  data () {
    return {
      ros: null,
      username: USER,
      defaultMessage: '',
      displayedMessage: '',
      showSettings: false,
      speed: 'moderate',
      publishTopic: '/webapp_to_robot',
      listener: null,
      subscribeTopic: '/robot_to_webapp'
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
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
    handleRosMessage(message) {
      
      try {
        const parsedMessage = JSON.parse(message.data);
        if (parsedMessage.state === 'explanation' && parsedMessage.status) {
          this.displayedMessage = parsedMessage.status || 'No status available';
        } else {
          this.displayedMessage = this.defaultMessage;
        }

        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          if (typeof route === 'string') {
            this.$router.push(route); 
          } else if (typeof route === 'object') {
            this.$router.push(route); 
          }
        }

        if (parsedMessage.state === 'emergency_stop' && parsedMessage.status === 'completed') {
          this.$router.push({ name: 'emergency_stop' });
        }
      } catch (error) {
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
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot', 
        messageType: 'std_msgs/String' 
      })
    },
    initSubscriber() {

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp', 
        messageType: 'std_msgs/String' 
      })

      this.listener.subscribe((msg) => {

        try {
          const parsedData = JSON.parse(msg.data);
          if (parsedData.state === 'drink_pickup' && parsedData.status === 'completed') {
            this.$router.push('/drink_confirm_transfer'); 
          }
        } catch (error) {
        }
      })
      this.listener.subscribe((message) => {
        this.handleRosMessage(message);
      });
    },
    publishMessage() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'confirm',
          status: 'completed'
        }) 
      })
      this.publisher.publish(message);
    },
    beforeUnmount() {
      if (this.listener) {
        this.listener.unsubscribe(); 
      }
    },

    handleKeyDown (event) { 
      if (event.key === 'e' || event.key === 'E') {
        this.$router.push({ name: 'emergency_stop' })
      }
    },
    redirectToChangeItem () {
      this.$router.push('/drink_confirm_transfer')
    },
    redirectToChangeItemF () {
      this.$router.push('/notify_caregiver')
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
  justify-content: center;
  align-items: center;
  height: 70vh;
}

.message {
  //font-size: 24px;
  font-weight: bold;
  font-family: Verdana;
  font-size: 40px;
  letter-spacing: 0.17499999701976776px;
  text-align: center;

}

.footer {
  display: flex;
  justify-content: center;
  padding: 20px;
}

.succeed-button {
  background-color:rgb(179, 181, 184);
  border: none;
  border-radius: 8px;
  color: white;
  padding: 10px 20px;
  cursor: pointer;
  font-size: 16px;
  display: flex;
  align-items: center;
  height: 50px;
}
</style>
