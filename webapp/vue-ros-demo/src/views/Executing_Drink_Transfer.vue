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
<!--          <span class="settings-button-text">Task Selection</span>-->
<!--        </button>-->
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
import { resolveDirective } from 'vue'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      username: USER,
      defaultMessage: '',
      displayedMessage: '',
      showSettings: false,
      speed: 'moderate',
      publishTopic: '/WebAppComm',
      listener: null
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
  methods: {
    handleRosMessage(message) {
      // 解析收到的JSON字符串
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
            this.$router.push(route); // string
          } else if (typeof route === 'object') {
            this.$router.push(route); // object
          }
        }
        // 根据解析后的消息内容跳转页面
        if (parsedMessage.state === 'emergency_stop' && parsedMessage.status === 'completed') {
          this.$router.push({ name: 'physical' });
        } else if (parsedMessage.state === 'some_other_state' && parsedMessage.status === 'some_status') {
          this.$router.push('/acquirebite');
        } else if (parsedMessage.state === 'another_state' && parsedMessage.status === 'another_status') {
          this.$router.push('/transferdrinks');
        }

        // 可以根据需要添加更多的条件来处理不同的消息
      } catch (error) {
        console.error('Failed to parse ROS message:', error);
      }
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
      const ros = new ROSLIB.Ros({
        url:  ROS_URL // changedURL
      })

      this.publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/WebAppComm', // 发布到 /talker 话题
        messageType: 'std_msgs/String' // 发布 std_msgs/String 类型的消息
      })
    },
    initSubscriber() {
      const ros = new ROSLIB.Ros({
        url:  ROS_URL // changedURL
      })

      this.listener = new ROSLIB.Topic({
        ros: ros,
        name: '/ServerComm', // 订阅 /listener 话题
        messageType: 'std_msgs/String' // 订阅 std_msgs/String 类型的消息
      })

      this.listener.subscribe((msg) => {
        console.log('Received message on /listener:', msg.data);

        try {
          const parsedData = JSON.parse(msg.data);
          if (parsedData.state === 'drink_transfer' && parsedData.status === 'completed') {
            this.$router.push('/afterdrinktransfer');
          }
        } catch (error) {
          console.error('Failed to parse received message:', error);
        }
      })
      this.listener.subscribe((message) => {
        console.log('Received message:', message.data);
        this.handleRosMessage(message);
      });
    },
    // handleKeyDown (event) {
    //   if (event.key === 'e' || event.key === 'E') {
    //     this.$router.push({ name: 'physical' })
    //   }
    // },
    beforeUnmount() {
      if (this.listener) {
        this.listener.unsubscribe(); // release
      }
    },
    // resolveDirective,
    // toggleSettings () {
    //   this.showSettings = !this.showSettings
    // },
    redirectToChangeItem () {
      this.$router.push('/afterdrinktransfer')
    },
    redirectToChangeItemF () {
      this.$router.push('/notify')
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
  justify-content: center;
  align-items: center;
  height: 70vh;
}

.message {
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
