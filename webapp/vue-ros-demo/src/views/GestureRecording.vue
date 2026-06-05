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
      <button class="finish-button">
        <img class="icon" alt="food" src="../assets/finish.png">
        <span class="finish-button-text" @click="redirectToChangeItemF">Finish Feeding</span>
      </button>
    </div>
  </div>
  <div class="video-recorder">
    <h2>Positive Video Recorder</h2>

    <!-- 左右布局容器 -->
    <div class="layout-container">
      <!-- 左侧：录制区域 -->
      <div class="recorder-container">
        <div class="controls">
          <button class=buttonchoose @click="startRecording('positive')">Start Recording</button>
          <button class=buttonchoose @click="stopRecording('negative')">Stop Recording</button>
          <button class=buttonchoose @click="deletRecording">Delete Last Recording</button>
          <button class=buttonchoose @click="goToNextPage">Next Page</button>
        </div>
        <span class="left_text">Robot Text Response:</span>
        <div class="option-container">
            <textarea
              v-model="customOrder"
              placeholder="Waiting for the text response..."
              class="custom-input-box"
              readonly
            ></textarea>
        </div>
      </div>
    </div>

    <!-- 固定在右下角的按钮 -->
<!--    <button class="next-page-button" @click="goToNextPage">Next Page</button>-->
  </div>

<!--  <div class="content">-->
<!--    <div class="message">-->
<!--      Looking at meal to identify food items-->
<!--    </div>-->
<!--  </div>-->

<!--  <div class="footer">-->
<!--    <button class="succeed-button" @click="redirectToChangeItem">Next</button>-->
<!--  </div>-->
</template>

<script>
import ROSLIB from 'roslib';
import { useRouter } from 'vue-router';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data() {
    return {
      username: USER,
      showSettings: false,
      speed: 'moderate',
      publishTopic: '/WebAppComm',
      listener: null,
      subscribeTopic: '/ServerComm',
      mediaRecorder: null,
      recordedChunks: [],
      recordedVideos: [],
      isRecording: false,
      currentLabel: '',
      positiveCount: 1,
      negativeCount: 1,
      ros: null,
      videoTopic: null,
      customOrder: '',
    }
  },
  setup() {
    const router = useRouter();
    return {router};
  },
  mounted() {
    this.initSubscriber()
    this.initPublisher()
    this.initRosConnection() // message
    // this.initMedia();
    this.initROS();
    window.addEventListener('keydown', this.handleKeyDown) // notify caregiver
  },
  beforeUnmount() {
    window.removeEventListener('keydown', this.handleKeyDown) // notify caregiver
  },
  beforeRouteLeave(to, from, next) {
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
    cleartheinput() {
      this.transcript = '';
    },
    initRosConnection() {
      const ros = new ROSLIB.Ros({
        url: ROS_URL // 替换为你的 ROS WebSocket 地址
      });

      // 初始化订阅器
      const listener = new ROSLIB.Topic({
        ros: ros,
        name: '/ServerComm', // 替换为你要监听的 Topic
        messageType: 'std_msgs/String'
      });

      // 监听消息
      listener.subscribe((message) => {
        console.log('Received ROS message:', message.data);
        const parsedMessage = JSON.parse(message.data)
        if (parsedMessage.state === 'gesture_response') {
          this.customOrder = parsedMessage.status;
        } else if (parsedMessage.state === 'some_other_state' && parsedMessage.status === 'some_status') {
          console.log('Pass');
        }
      });

      this.listener = listener;
    },
    // async initMedia() {
    //   try {
    //     const stream = await navigator.mediaDevices.getUserMedia({ video: true });
    //     this.$refs.videoElement.srcObject = stream;
    //     this.mediaRecorder = new MediaRecorder(stream);
    //
    //     this.mediaRecorder.ondataavailable = (event) => {
    //       if (event.data.size > 0) {
    //         this.recordedChunks.push(event.data);
    //       }
    //     };
    //
    //     this.mediaRecorder.onstop = () => {
    //       const videoBlob = new Blob(this.recordedChunks, { type: 'image/jpeg' });
    //       const videoUrl = URL.createObjectURL(videoBlob);
    //
    //       let videoName = '';
    //       if (this.currentLabel === 'positive') {
    //         videoName = `positive${this.positiveCount}`;
    //         this.positiveCount++;
    //       } else if (this.currentLabel === 'negative') {
    //         videoName = `negative${this.negativeCount}`;
    //         this.negativeCount++;
    //       }
    //
    //       this.recordedVideos.push({ url: videoUrl, name: videoName });
    //       this.recordedChunks = [];
    //     };
    //   } catch (error) {
    //     console.error('Error accessing media devices.', error);
    //   }
    // },

    // 初始化ROS连接
    initROS() {
      this.ros = new ROSLIB.Ros({
        url:  ROS_URL,
      });

      this.ros.on('connection', () => {
        console.log('Connected to ROS bridge!');
      });

      this.ros.on('error', (error) => {
        console.error('Error connecting to ROS bridge:', error);
      });

      this.ros.on('close', () => {
        console.log('Connection to ROS bridge closed.');
      });

      this.videoTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/video_stream',
        messageType: 'sensor_msgs/CompressedImage',
      });
    },

    // 开始录制
    startRecording(label) {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_add',
          status: 'start'
        }) // publish json
      })
      this.publisher.publish(message);
    },

    // 停止录制
    stopRecording() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_add',
          status: 'stop'
        }) // publish json
      })
      this.publisher.publish(message);
    },

    deletRecording() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_add',
          status: 'delete'
        }) // publish json
      })
      this.publisher.publish(message);
    },

    // 删除视频
    deleteVideo(index) {
      this.recordedVideos.splice(index, 1);
    },

    publishToROS(video) {
      console.log(`[INFO] Starting to publish video: ${video.name}`);

      fetch(video.url)
        .then((response) => response.blob())
        .then((blob) => {
          console.log(`[INFO] Blob size: ${blob.size} bytes`);

          const reader = new FileReader();
          reader.readAsArrayBuffer(blob);
          reader.onloadend = () => {
            const uint8Array = new Uint8Array(reader.result);
            console.log(`[INFO] Uint8Array length: ${uint8Array.length}`);

            // 打印前 20 个字节
            console.log('[INFO] First 20 bytes of Uint8Array:', uint8Array.slice(0, 20));

            // 发布到 ROS
            const message = new ROSLIB.Message({
              format: 'jpeg',
              data: Array.from(uint8Array),
            });

            this.videoTopic.publish(message);
            console.log(`[INFO] Published ${video.name} to ROS topic /video_stream`);
          };
        })
        .catch((error) => {
          console.error('[ERROR] Error publishing video to ROS:', error);
        });
    },
    // 跳转到下一页
    goToNextPage() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'gesture_add',
          status: 'next'
        }) // publish json
      })
      this.publisher.publish(message);
      this.router.push('/gesturerecording2');
    },

    handleRosMessage(message) {
      // 解析收到的JSON字符串
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
        if (parsedMessage.state === 'emergency_stop' && parsedMessage.status === 'completed') {
          this.$router.push({name: 'physical'});
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
          const parsedMessage = JSON.parse(msg.data);
          if (parsedMessage.state === 'prepare_bite' && parsedMessage.status === 'completed') {
            this.$router.push('/newmealpage'); // jump
          } else if (parsedMessage.state === 'home' && parsedMessage.status === 'jump') {
            this.$router.push('/home');
          } else if (parsedMessage.state === 'preparepickup' && parsedMessage.status === 'jump') {
            this.$router.push('/preparepickup');
          } else if (parsedMessage.state === 'newmealpage' && parsedMessage.status === 'jump') {
            this.$router.push('/newmealpage');
          } else if (parsedMessage.state === 'preparepickup2' && parsedMessage.status === 'jump') {
            this.$router.push('/preparepickup2');
          } else if (parsedMessage.state === 'acquirebite' && parsedMessage.status === 'jump') {
            this.$router.push('/acquirebite');
          } else if (parsedMessage.state === 'pickingup' && parsedMessage.status === 'jump') {
            this.$router.push('/pickingup');
          } else if (parsedMessage.state === 'transfermeal' && parsedMessage.status === 'jump') {
            this.$router.push('/transfermeal');
          } else if (parsedMessage.state === 'executingbitetransfer' && parsedMessage.status === 'jump') {
            this.$router.push('/executingbitetransfer');
          } else if (parsedMessage.state === 'afterbitetransfer' && parsedMessage.status === 'jump') {
            this.$router.push('/afterbitetransfer');
          } else if (parsedMessage.state === 'swithtodrink' && parsedMessage.status === 'jump') {
            this.$router.push('/swithtodrink');
          } else if (parsedMessage.state === 'transferdrinks' && parsedMessage.status === 'jump') {
            this.$router.push('/transferdrinks');
          } else if (parsedMessage.state === 'executingdrinktransfer' && parsedMessage.status === 'jump') {
            this.$router.push('/executingdrinktransfer');
          } else if (parsedMessage.state === 'wiping' && parsedMessage.status === 'jump') {
            this.$router.push('/wiping');
          } else if (parsedMessage.state === 'wipingtrans' && parsedMessage.status === 'jump') {
            this.$router.push('/wipingtrans');
          } else if (parsedMessage.state === 'wipingprocess' && parsedMessage.status === 'jump') {
            this.$router.push('/wipingprocess');
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
    handleKeyDown(event) { // notify caregiver
      if (event.key === 'e' || event.key === 'E') {
        this.$router.push({name: 'physical'})
      }
    },
    // resolveDirective,
    // toggleSettings () {
    //   this.showSettings = !this.showSettings
    // },
    redirectToChangeItem() {
      this.$router.push('/gesturesetting')
    },
    redirectToChangeItemF() {
      this.$router.push('/notify')
    },
  }
}
</script>

<style scoped>
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
.option-container {
  //max-height: 36vh; /* 设置选项框的最大高度 */
  ////overflow-x: auto;  /* 垂直滚动 */
  //padding-right: 10px; /* 为滚动条留出空间 */
  //display: flex;
  ////align-items: flex-start;
  //justify-content: space-between;
  //flex-flow: column;
  //padding: 0px;
  width: 100%;
  height: 20vh; /* 你可以根据需要调整高度 */
  display: flex;
  justify-content: center;
  align-items: center;
}
.video-recorder {
  height: 90vh;
  max-width: 1200px;
  margin: auto;
  text-align: center;
}
.video-recorder h2 {
  font-size: 20px;   /* 缩小标题字体 */
  margin-bottom: 10px;
}
/* 左右布局样式 */
.layout-container {
  display: flex;
  gap: 20px;
  height: 60vh;
}

/* 左侧：录制区域 */
.recorder-container {
  //flex: 1;
  //position: relative;
  //display: flex;
  //flex-direction: column;
  //align-items: center;
  //border: 1px solid #ccc;
  //padding: 10px;
  //border-radius: 8px;
  flex: 1;
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  border: 1px solid #ccc;
  padding: 10px;
  border-radius: 8px;
  max-width: 90vw; /* 设置最大宽度 */
  margin: 0 auto;   /* 居中显示 */
}

.recorder-container video {
  //width: 100%;
  //border-radius: 8px;
  width: 100%;
  height: auto;
  max-height: 80vh; /* 限制视频高度 */
  border-radius: 8px;
  object-fit: cover;
}

.controls {
  margin-top: 10px;
  display: flex;
  align-items: center;
}
.buttonchoose {
  background-color: #FFE699; /* 确认按钮的背景颜色 */
  color: #000000;           /* 确认按钮的字体颜色 */
  border-radius: 20px;
  padding: 12px 20px;
  font-size: 25px;
  border: none;
  cursor: pointer;
  margin: 5px;
  width: 250px;              /* 可根据需要调整宽度 */
  height: 100px;              /* 可根据需要调整高度 */
  transition: all 0.3s ease;
}
button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

/* 右侧：视频预览 */
.preview-container {
  flex: 1;
  display: flex;
  flex-direction: column;
  border: 1px solid #ccc;
  padding: 10px;
  border-radius: 8px;
  max-height: 90vh; /* 限制高度 */
  overflow-y: auto;  /* 启用垂直滚动条 */
}

.video-list {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  justify-content: center;
}

.video-item {
  position: relative;
  text-align: center;
}
.custom-input-box {
  width: 90%; /* 文本框宽度占页面宽度的 90% */
  height: 100%; /* 文本框高度占容器的 100% */
  font-size: 1.7vw;
  border: 1px solid #ccc;
  border-radius: 10px;
  padding: 10px;
  resize: none; /* 禁止用户手动调整大小 */
  box-sizing: border-box;
}
.video-item video {
  width: 200px;
  height: 150px;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
}

/* 按钮组样式 */
.button-group {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 10px; /* 按钮之间的间距 */
  margin-top: 8px;
  width: 100%; /* 确保按钮组占满父容器的宽度 */
}

/* 删除按钮样式 */
.delete-button {
  background-color: #ff4d4f;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 8px 12px;
  font-size: 12px;
  cursor: pointer;
  width: 40%; /* 设置按钮宽度为父容器的一半 */
  text-align: center;
  //padding: 5px 10px;
  //background-color: #ff4d4f;
  //color: white;
  //border: none;
  //border-radius: 4px;
  //cursor: pointer;
  //font-size: 12px;
  height: 30%;
}

.delete-button:hover {
  background-color: #d9363e;
}

/* ROS发布按钮样式 */
.ros-button {
  background-color: #28a745;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 8px 12px;
  font-size: 12px;
  cursor: pointer;
  width: 40%; /* 设置按钮宽度为父容器的一半 */
  text-align: center;
  height: 30%;
  //padding: 5px 10px;
  //background-color: #28a745;
  //color: white;
  //border: none;
  //border-radius: 4px;
  //cursor: pointer;
  //font-size: 12px;
}

.ros-button:hover {
  background-color: #218838;
}

/* 固定在右下角的按钮样式 */
.next-page-button {
  position: fixed;
  bottom: 20px;
  right: 20px;
  padding: 15px 25px;
  background-color: #FFE699; /* 确认按钮的背景颜色 */
  color: #000000;
  font-size: 16px;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
}

.next-page-button:hover {
  background-color: #0056b3;
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
      white-space: nowrap;
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
  line-height: 24px;
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











