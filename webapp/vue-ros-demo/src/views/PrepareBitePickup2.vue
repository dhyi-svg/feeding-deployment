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
        <span class = "finish-button-text" @click ="redirectToChangeItemF">Finish Feeding</span>
      </button>
    </div>
  </div>

  <div class="skill-plan" v-if="planSlots.current">
    <div class="skill-step past" v-if="planSlots.last">
      <span class="step-label">Previous</span>
      <span class="step-name">{{ skillLabel(planSlots.last) }}</span>
    </div>
    <div class="skill-arrow" v-if="planSlots.last">&#8594;</div>
    <div class="skill-step current">
      <span class="step-label">Now</span>
      <span class="step-name">{{ skillLabel(planSlots.current) }}</span>
    </div>
    <div class="skill-arrow" v-if="planSlots.next">&#8594;</div>
    <div class="skill-step upcoming" v-if="planSlots.next">
      <span class="step-label">Next</span>
      <span class="step-name">{{ skillLabel(planSlots.next) }}</span>
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
      username: USER,
      defaultMessage: '',
      displayedMessage: '',
      showSettings: false,
      speed: 'moderate',
      publishTopic: '/WebAppComm',
      listener: null,
      subscribeTopic: '/ServerComm',
      // Full ordered skill plan + index of the running skill, both set from
      // the backend's /ServerComm "skill_plan" messages. The plan can contain
      // any skill: navigation, fridge/microwave manipulation, plate handling,
      // and the bite/drink/wipe steps.
      skillPlan: [],
      currentSkillIndex: -1,
      // Human-readable labels for each skill (snake_case behavior-tree name ->
      // display name). Any skill not listed falls back to a title-cased name.
      skillLabels: {
        navigate_to_table: 'Drive to Table',
        navigate_to_fridge: 'Drive to Fridge',
        navigate_to_microwave: 'Drive to Microwave',
        navigate_to_sink: 'Drive to Sink',
        open_fridge: 'Open Fridge',
        close_fridge: 'Close Fridge',
        open_microwave: 'Open Microwave',
        close_microwave: 'Close Microwave',
        press_microwave_button: 'Start Microwave',
        pick_plate_from_fridge: 'Take Plate from Fridge',
        pick_plate_from_microwave: 'Take Plate from Microwave',
        pick_plate_from_table: 'Take Plate from Table',
        pick_plate_from_holder: 'Take Plate from Holder',
        place_plate_in_fridge: 'Put Plate in Fridge',
        place_plate_in_microwave: 'Put Plate in Microwave',
        place_plate_in_sink: 'Put Plate in Sink',
        place_plate_on_table: 'Put Plate on Table',
        place_plate_on_holder: 'Put Plate on Holder',
        gaze_at_table: 'Look at Table',
        emulate_transfer: 'Gesture Transfer',
        pick_utensil: 'Pick Up Utensil',
        acquire_bite: 'Acquire Bite',
        transfer_utensil: 'Transfer to Mouth',
        stow_utensil: 'Stow Utensil',
        pick_drink: 'Pick Up Drink',
        transfer_drink: 'Transfer Drink',
        stow_drink: 'Stow Drink',
        pick_wipe: 'Pick Up Wipe',
        transfer_wipe: 'Transfer Wipe',
        stow_wipe: 'Stow Wipe'
      }
    }
  },
  computed: {
    // The last / current / next skill to display, taken directly from the
    // backend-provided plan.
    planSlots () {
      const plan = this.skillPlan
      const idx = this.currentSkillIndex
      if (idx < 0 || idx >= plan.length) {
        return { last: null, current: null, next: null }
      }
      return {
        last: idx > 0 ? plan[idx - 1] : null,
        current: plan[idx],
        next: idx < plan.length - 1 ? plan[idx + 1] : null
      }
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
        // Skill-plan updates carry the ordered plan + index of the running
        // skill. Update the highlighted skill and leave the explanation text
        // untouched.
        if (parsedMessage.state === 'skill_plan') {
          if (Array.isArray(parsedMessage.plan) && typeof parsedMessage.current === 'number') {
            this.skillPlan = parsedMessage.plan;
            this.currentSkillIndex = parsedMessage.current;
          }
          return;
        }
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
      } catch (error) {
        console.error('Failed to parse ROS message:', error);
      }
    },
    skillLabel(name) {
      if (this.skillLabels[name]) {
        return this.skillLabels[name];
      }
      // Fallback: turn "pick_plate_from_table" into "Pick Plate From Table".
      return name
        .split('_')
        .map(word => word.charAt(0).toUpperCase() + word.slice(1))
        .join(' ');
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
          const parsedMessage = JSON.parse(msg.data);
          if (parsedMessage.state === 'prepare_bite' && parsedMessage.status === 'completed') {
            this.$router.push('/acquirebite');
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
    handleKeyDown (event) { // notify caregiver
      if (event.key === 'e' || event.key === 'E') {
        this.$router.push({ name: 'physical' })
      }
    },
    // resolveDirective,
    // toggleSettings () {
    //   this.showSettings = !this.showSettings
    // },
    redirectToChangeItem () {
      this.$router.push('/acquirebite')
    },
    redirectToChangeItemF () {
      this.$router.push('/notify')
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

.skill-plan {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 12px;
  padding: 16px 10px;
}

.skill-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  min-width: 140px;
  padding: 10px 18px;
  border-radius: 10px;
  background: #eee;
  color: #6e7e8e;
  border: 2px solid transparent;
}

.skill-step.current {
  background: #6e7e8e;
  color: white;
  border-color: #3d4a57;
  transform: scale(1.08);
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.15);
}

.skill-step .step-label {
  font-family: Verdana;
  font-size: 12px;
  text-transform: uppercase;
  letter-spacing: 1px;
  opacity: 0.8;
}

.skill-step .step-name {
  font-family: Verdana;
  font-size: 18px;
  font-weight: bold;
  margin-top: 4px;
}

.skill-arrow {
  font-size: 26px;
  color: #6e7e8e;
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
