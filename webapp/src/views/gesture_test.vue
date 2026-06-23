<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Pick a gesture to test</div>
      </div>
    </div>

    <div class="bd">
      <div class="gtest-body">
        <div class="opts gtest-list">
          <div
            class="oc"
            v-for="(option, index) in optionTexts"
            :key="index"
            :class="{ sel: selectedOption === index + 1 }"
            @click="selectOption(index + 1)"
          >
            <span class="ot">{{ option }}</span>
            <div class="och" v-if="selectedOption === index + 1">✓</div>
          </div>
        </div>
        <div class="gtest-col">
          <span class="field-lbl">Robot's response</span>
          <div class="response-box">{{ customOrder || 'Waiting for the text response...' }}</div>
          <div class="field-actions" style="margin-top:0">
            <button class="btn sm teal" style="flex:1" @click="ConfirmSelection">Send &amp; Test</button>
            <button class="btn sm ghost" style="flex:1" @click="ReturnSelection">Done</button>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      ros: null,
      username: USER,
      selectedOption: 0,
      listener: null,
      publisher: null,
      optionTexts: [
        'Waiting for content',
        'Waiting for content',
        'Waiting for content',
        'Waiting for content',
        'Waiting for content',
        'Waiting for content'
      ],
      transcript: '',
      customOrder: '',
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initRosConnection()
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
    initRosConnection() {

      const listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      });

      listener.subscribe((message) => {
        const parsedMessage = JSON.parse(message.data)
        if (parsedMessage.state === 'gesture_response') {
          this.customOrder = parsedMessage.status;
        }
      });
      this.listener = listener;
    },
    ConfirmSelection() {

      if (this.selectedOption !== null && this.selectedOption !== 0) {
        const selectedText = this.optionTexts[this.selectedOption - 1];

        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'gesture_test_selection',
            status: selectedText
          })
        });
        this.publisher.publish(message);
      } else {
      }

      if (this.selectedOption !== 0 || this.transcript !== '') {
        this.transcript = '';

      } else {
        alert('Please select');
      }
    },
    ReturnSelection() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_test_selection',
          status: 'back'
        })
      })
      this.publisher.publish(message)
      this.$router.push('/robot_executing');
    },
    selectOption(option) {
      this.selectedOption = this.selectedOption === option ? 0 : option;
    },
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
  }
}
</script>

<style scoped>
.gtest-body {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1.5vw;
  flex: 1;
  min-height: 0;
}

.gtest-list {
  overflow-y: auto;
  min-height: 0;
}

.gtest-col {
  display: flex;
  flex-direction: column;
  gap: 1vh;
  min-height: 0;
}
</style>
