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
      </div>
      <button class="finish-button">
        <img class="icon" alt="food" src="../assets/finish.png">
        <span class="finish-button-text" @click="redirectToChangeItemF">Finish Feeding</span>
      </button>
    </div>
  </div>

  <div class="content">
    <div class="content-body">
      <!-- 左边 -->
      <div class="left">
        <img ref="videoImage" class="food" :src="videoFrame">
<!--        <span class="left_text">I see the following food items on your plate:</span>-->
<!--        <div class="threeboxes">-->
<!--          <div v-for="(item, index) in foodItems" :key="index" class="box">-->
<!--            <img class="metaballs" :alt="item.name" :src="item.image" />-->
<!--            <span class="right-first-title" style="display: block; text-align: left;">{{ item.name }}</span>-->
<!--          </div>-->
<!--        </div>-->
      </div>
      <div class="right">
        <div class="right-other-option-text">
          <span class="right-little-text">Please list the food items on the plate</span>
<!--          <span class="right-other-option-little-text">Type/Speak your description here!</span>-->
        </div>
        <div class="otheroption">
          <div class="otheroptionbox">
            <input
              type="text"
              v-model="transcript"
              placeholder="Typing..."
              ref="textarea1"
              class="voiceinputbox"
            >
            <div class="voicebuttongroup">
              <button @click="startSpeechRecognition1"
                      class="voice-start-button"
                      :class="{'recognizing': isRecognizing1}"
                      :disabled="isRecognizing1">
                <img class="voice" alt="voice" src="../assets/voice.png">
              </button>
              <button @click="cleartheinput1" class="voice-send-button">
                <img class="clear" alt="voice" src="../assets/clear.png">
              </button>
            </div>
          </div>
        </div>
        <span class="right-little-text">Please provide your preferred bite ordering for this plate</span>
        <div class="otheroption1">
          <div class="otheroptionbox1">
              <textarea
                v-model="transcriptDes"
                placeholder="Typing..."
                ref="textarea2"
                class="custom-input-box1"
              ></textarea>
            <div class="voicebuttongroup1">
              <button @click="startSpeechRecognition2"
                      class="voice-start-button"
                      :class="{'recognizing': isRecognizing2}"
                      :disabled="isRecognizing2">
                <img class="voice" alt="voice" src="../assets/voice.png">
              </button>
              <button @click="cleartheinput2" class="voice-send-button">
                <img class="clear" alt="voice" src="../assets/clear.png">
              </button>
            </div>
          </div>
        </div>
        <div class="buttonpart">
          <div class="button2" @click="confirmSelection">
            <span class="confirm-button-text">Confirm</span>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      isRecognizing1: false, // 第一个输入框的语音识别状态
      isRecognizing2: false, // 第二个输入框的语音识别状态
      recognition1: null, // 第一个语音识别对象
      recognition2: null, // 第二个语音识别对象
      username: USER,
      selectedOption: 0,
      showSettings: false,
      speed: 'moderate',
      foodItems: [],
      videoFrame: null, // 用于存储视频帧数据
      subscribeTopic: '/ServerComm',
      publishTopic: '/WebAppComm',
      listener: null,
      publisher: null,
      optionTexts: [
        'Waiting for content',
        'Waiting for content',
        'Waiting for content'
      ],
      recognition: null,
      transcript: '',
      previousSelectedOption: null,
      isRecognizing: false,
      transcriptDes: '',
    }
  },
  watch: {
    transcript(newValue) {
      if (newValue !== '') {
        // 当 transcript 有值时，存储之前的 selectedOption，并设置为 0
        if (this.selectedOption !== 0) {
          this.previousSelectedOption = this.selectedOption;
        }
        this.selectedOption = 0;
      } else {
        // 当 transcript 为空时，恢复之前的 selectedOption
        if (this.previousSelectedOption !== null) {
          this.selectedOption = this.previousSelectedOption;
          this.previousSelectedOption = null; // 清空 previousSelectedOption
        }
      }
    }
  },
  mounted () {
    this.initSubscriber()
    this.initPublisher()
    this.publishMessageOnLoad()
    this.initVideoSubscriber()
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
    cleartheinput1() {
      this.transcript = ''; // 清空第一个输入框
    },

    cleartheinput2() {
      this.transcriptDes = ''; // 清空第二个输入框
    },

    startSpeechRecognition1() {
      if (this.isRecognizing1) {
        return; // If recognition is already running, do nothing
      }
      if (!this.recognition1) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition1 = new SpeechRecognition();
        this.recognition1.lang = 'en-US';
        this.recognition1.continuous = false;

        this.recognition1.onresult = (event) => {
          this.transcript += event.results[0][0].transcript; // 更新第一个输入框内容
          this.isRecognizing1 = false;
        };

        this.recognition1.onerror = () => {
          this.isRecognizing1 = false;
        };

        this.recognition1.onend = () => {
          this.isRecognizing1 = false;
        };
      }

      if (this.isRecognizing1) {
        this.recognition1.stop();
      }

      this.isRecognizing1 = true;
      this.recognition1.start();
    },

    startSpeechRecognition2() {
      if (this.isRecognizing2) {
        return; // If recognition is already running, do nothing
      }
      if (!this.recognition2) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition2 = new SpeechRecognition();
        this.recognition2.lang = 'en-US';
        this.recognition2.continuous = false;

        this.recognition2.onresult = (event) => {
          this.transcriptDes += event.results[0][0].transcript; // 更新第二个输入框内容
          this.isRecognizing2 = false;
        };

        this.recognition2.onerror = () => {
          this.isRecognizing2 = false;
        };

        this.recognition2.onend = () => {
          this.isRecognizing2 = false;
        };
      }

      if (this.isRecognizing2) {
        this.recognition2.stop();
      }

      this.isRecognizing2 = true;
      this.recognition2.start();
    },
    cleartheinput() {
      this.transcript = '';
    },

    startSpeechRecognition() {
      // 如果识别过程中再次点击按钮，重新开始语音识别
      if (!this.recognition) {
        // Initialize the Web Speech API
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition = new SpeechRecognition();
        this.recognition.lang = 'en-US';
        this.recognition.continuous = false; // Set to false for single result

        // Event when speech recognition returns a result
        this.recognition.onresult = (event) => {
          // Append the recognized speech to the existing content
          this.transcript += event.results[0][0].transcript;
          this.isRecognizing = false; // 识别结束，按钮重新可点击
        };

        // Event when an error occurs in recognition
        this.recognition.onerror = (event) => {
          console.error("Speech recognition error", event);
          this.isRecognizing = false; // 如果识别出错，按钮也要重新可点击
          this.focusTextarea(); // 错误后保持输入框激活状态
        };

        // Event when recognition ends
        this.recognition.onend = () => {
          this.isRecognizing = false; // 语音识别结束，按钮重新可点击
          this.focusTextarea(); // 语音识别结束后保持输入框激活状态
        };
      }

      // 停止当前的识别，如果正在识别的话
      if (this.isRecognizing) {
        this.recognition.stop();
      }

      // 再次开始语音识别
      this.isRecognizing = true;
      this.recognition.start();

      // 确保输入框始终保持激活状态
      this.$nextTick(() => {
        this.focusTextarea(); // 确保在识别过程中，输入框保持激活
      });
    },
    focusTextarea() {
      // Automatically focus the textarea
      this.$refs.textarea.focus();
    },
    publishMessageforNopre(){
      const message = new ROSLIB.Message({
        data: JSON.stringify({ // 将消息内容转换为JSON字符串
          state: 'voice',
          status: this.transcript // 使用输入框的内容作为status字段的值
        })
      })
      this.publisher.publish(message)
      this.transcript = '';
    },
    initVideoSubscriber() {
      const ros = new ROSLIB.Ros({
        url:  ROS_URL
      });

      const listener = new ROSLIB.Topic({
        ros: ros,
        name: '/camera/image/compressed',
        messageType: 'sensor_msgs/CompressedImage'
      });

      listener.subscribe((message) => {
        if (typeof message.data === 'string') {
          const base64Image = `data:image/jpeg;base64,${message.data}`;
          this.videoFrame = base64Image;
          this.extractSubImagesAndUpdateItems();
        }
      });
    },

    extractSubImagesAndUpdateItems() {
      this.subImages = [];
      if (this.videoFrame && this.$refs.videoImage && this.foodItems.length > 0) {
        const image = this.$refs.videoImage;
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');

        const img = new Image();
        img.src = this.videoFrame;

        img.onload = () => {
          this.foodItems.forEach((item, index) => {
            // 设置 canvas 大小为目标框的宽高
            const maxBoxWidth = window.innerWidth * 0.1;
            const maxBoxHeight = window.innerHeight * 0.1;
            // const maxBoxWidth = 120; // 框的宽度，可以根据需要调整
            // const maxBoxHeight = 120; // 框的高度

            // 计算缩放比例
            const widthRatio = maxBoxWidth / item.w;
            const heightRatio = maxBoxHeight / item.h;
            const scaleRatio = Math.min(widthRatio, heightRatio);

            // 缩放后的宽高
            const scaledWidth = item.w * scaleRatio;
            const scaledHeight = item.h * scaleRatio;

            // 设置 canvas 大小为缩放后的大小
            canvas.width = scaledWidth;
            canvas.height = scaledHeight;

            // 截取并绘制缩放后的图片
            ctx.drawImage(
              img,
              item.x, item.y, item.w, item.h,  // 原图的位置和大小
              0, 0, scaledWidth, scaledHeight  // 缩放后的位置和大小
            );

            const subImage = canvas.toDataURL('image/jpeg');
            this.subImages.push(subImage);

            if (index < this.foodItems.length) {
              this.foodItems[index].image = subImage; // 更新图片为处理后的缩放图像
            }
          });
        };
      } else {
        console.error('Sub-images not extracted. Check video frame and food items.');
      }
    },

    // extractSubImagesAndUpdateItems() {
    //   this.subImages = [];
    //   if (this.videoFrame && this.$refs.videoImage && this.foodItems.length > 0) {
    //     const image = this.$refs.videoImage;
    //     const canvas = document.createElement('canvas');
    //     const ctx = canvas.getContext('2d');
    //
    //     const img = new Image();
    //     img.src = this.videoFrame;
    //
    //     img.onload = () => {
    //       this.foodItems.forEach((item, index) => {
    //         canvas.width = item.w;
    //         canvas.height = item.h;
    //
    //         ctx.drawImage(
    //           img,
    //           item.x, item.y, item.w, item.h,
    //           0, 0, item.w, item.h
    //         );
    //
    //         const subImage = canvas.toDataURL('image/jpeg');
    //         this.subImages.push(subImage);
    //
    //         if (index < this.foodItems.length) {
    //           this.foodItems[index].image = subImage;
    //         }
    //       });
    //     };
    //   } else {
    //     console.error('Sub-images not extracted. Check video frame and food items.');
    //   }
    // },


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

        if (parsedMessage && Array.isArray(parsedMessage.data)) {
          // 更新现有的 foodItems，而不是覆盖
          const updatedFoodItems = parsedMessage.data.map(item => {
            const itemName = Object.keys(item)[0];
            const itemCoordinates = item[itemName][0];

            return {
              name: itemName,
              image: this.getImageForItem(itemName),
              x: itemCoordinates[0],
              y: itemCoordinates[1],
              w: itemCoordinates[2],
              h: itemCoordinates[3]
            };
          });

          // 如果存在 current_bite，合并到 foodItems 中
          if (parsedMessage.current_bite) {
            const currentBiteName = Object.keys(parsedMessage.current_bite)[0];
            const currentBiteCoordinates = parsedMessage.current_bite[currentBiteName][0];

            updatedFoodItems.push({
              name: currentBiteName,
              image: this.getImageForItem(currentBiteName),
              x: currentBiteCoordinates[0],
              y: currentBiteCoordinates[1],
              w: currentBiteCoordinates[2],
              h: currentBiteCoordinates[3],
              isCurrentBite: true
            });
          }

          // 合并更新
          this.foodItems = [...updatedFoodItems];
        }
      } catch (error) {
        console.error('Failed to parse ROS message:', error);
      }
    },







    // handleRosMessage(message) {
    //   try {
    //     const parsedMessage = JSON.parse(message.data);
    //
    //     // 更新 n_food_types，实际数量加 1
    //     const nFoodTypes = parsedMessage.n_food_types + 1;
    //
    //     // 动态更新 foodItems，包括 current_bite
    //     if (parsedMessage && Array.isArray(parsedMessage.data)) {
    //       this.foodItems = parsedMessage.data.map(item => {
    //         const itemName = Object.keys(item)[0];
    //         const itemCoordinates = item[itemName][0]; // 获取第一个坐标数据
    //
    //         return {
    //           name: itemName,
    //           image: this.getImageForItem(itemName),
    //           x: itemCoordinates[0],
    //           y: itemCoordinates[1],
    //           w: itemCoordinates[2],
    //           h: itemCoordinates[3]
    //         };
    //       });
    //
    //       // 添加 current_bite 到 foodItems
    //       if (parsedMessage.current_bite) {
    //         const currentBiteName = Object.keys(parsedMessage.current_bite)[0];
    //         const currentBiteCoordinates = parsedMessage.current_bite[currentBiteName][0]; // 获取第一个坐标数据
    //
    //         this.foodItems.push({
    //           name: currentBiteName,
    //           image: this.getImageForItem(currentBiteName),
    //           x: currentBiteCoordinates[0],
    //           y: currentBiteCoordinates[1],
    //           w: currentBiteCoordinates[2],
    //           h: currentBiteCoordinates[3],
    //           isCurrentBite: true // 标记这是 current bite
    //         });
    //       }
    //     }
    //   } catch (error) {
    //     console.error('Failed to parse ROS message:', error);
    //   }
    // },
    confirmSelection() {
      // 判断 selectedOption 是否为 0 或 null
      if (this.selectedOption !== null && this.selectedOption !== 0) {
        const selectedText = this.optionTexts[this.selectedOption - 1];

        // 发送 order_selection 消息
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'order_selection',
            status: selectedText
          })
        });
        this.publisher.publish(message);
        console.log('Order selection sent:', message);
      } else {
        console.log('No valid option selected.');
      }

      // 检查是否有输入框内容需要发送
      if (this.transcript !== '' && this.transcriptDes !== '') {
        const voiceMessage = new ROSLIB.Message({
          data: JSON.stringify({
            state: this.transcript,
            status: this.transcriptDes // 使用输入框的内容作为 status 字段的值
          })
        });
        this.publisher.publish(voiceMessage);
        console.log('Voice message sent:', voiceMessage);
      }

      // 只有当 selectedOption 不为 0 或者 transcript 不为空时才跳转
      this.transcript = '';
      this.transcriptDes = '';
      this.$router.push('/preparepickup2');
    },
    publishMessageOnLoad() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'order_selection',
          status: 'ready_for_initial_data'
        })
      })

      if (this.publisher) {
        this.publisher.publish(message)
      } else {
        console.error('Publisher is not initialized yet.')
      }
    },
    // handleKeyDown(event) {
    //   if (event.key === 'e' || event.key === 'E') {
    //     this.$router.push({ name: 'physical' })
    //   }
    // },
    selectOption(option) {
      this.selectedOption = this.selectedOption === option ? 0 : option;
      this.transcript = '';
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
    redirectToChangeItem() {
      this.$router.push('/acquirebite')
    },
    redirectToChangeItemF() {
      this.$router.push('/notify')
    },
    initSubscriber() {
      const ros = new ROSLIB.Ros({
        url:  ROS_URL
      })

      this.listener = new ROSLIB.Topic({
        ros: ros,
        name: this.subscribeTopic,
        messageType: 'std_msgs/String'
      })

      this.listener.subscribe((msg) => {
        try {
          console.log('Received ROS message:', msg.data);
          const data = JSON.parse(msg.data);

          // 处理 foodItems 更新
          if (data && data.n_food_types) {
            // 直接嵌入 handleRosMessage 的功能
            try {
              const parsedMessage = data;

              // 动态更新 foodItems，包括 current_bite
              if (parsedMessage && Array.isArray(parsedMessage.data)) {
                // 更新 foodItems
                this.foodItems = parsedMessage.data.map(item => {
                  const itemName = Object.keys(item)[0];
                  const itemCoordinates = item[itemName][0]; // 获取第一个坐标数据

                  return {
                    name: itemName,
                    image: this.getImageForItem(itemName),
                    x: itemCoordinates[0],
                    y: itemCoordinates[1],
                    w: itemCoordinates[2],
                    h: itemCoordinates[3]
                  };
                });

                // 如果存在 current_bite，添加到 foodItems
                if (parsedMessage.current_bite) {
                  const currentBiteName = Object.keys(parsedMessage.current_bite)[0];
                  const currentBiteCoordinates = parsedMessage.current_bite[currentBiteName][0]; // 获取第一个坐标数据

                  this.foodItems.push({
                    name: currentBiteName,
                    image: this.getImageForItem(currentBiteName),
                    x: currentBiteCoordinates[0],
                    y: currentBiteCoordinates[1],
                    w: currentBiteCoordinates[2],
                    h: currentBiteCoordinates[3],
                    isCurrentBite: true // 标记为当前咬
                  });
                }
              }
            } catch (error) {
              console.error('Failed to parse ROS message:', error);
            }
          }

          // 处理 optionTexts 更新
          if (data && data.n_ordering) {
            const updatedOptionTexts = data.data.map((option) => `${option}`);
            this.optionTexts = [...updatedOptionTexts];
          }
        } catch (error) {
          console.error('Failed to process ROS message:', error);
        }
      });
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
    getImageForItem(name) {
      if (!name) {
        console.error('Item name is undefined:', name);
        return ''; // 或者返回一个默认图像路径
      }
    }
  }
}
</script>

<style scoped>
.voice-start-button.recognizing {
  background-color: #6e7e8e; /* 语音识别中时按钮变为绿色 */
}
.otheroptionbox1{
  background-color: transparent;
  margin-right: 20px;
  height: 30vh;
  width: 50vw;
  top: 645px;
  left: 708px;
  gap: 0px;
  border-radius: 20px 20px 20px 20px;
  opacity: 0px;
  //background: #D9D9D9;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 5px;
  //margin-top: 10px;
  flex-flow: column;
}
.custom-input-box1 {
  width: 90%; /* 文本框宽度占页面宽度的 90% */
  height: 100%; /* 文本框高度占容器的 100% */
  font-size: 1.7vw;
  border: 1px solid #ccc;
  border-radius: 10px;
  padding: 10px;
  resize: none; /* 禁止用户手动调整大小 */
  box-sizing: border-box;
}
.voicebuttongroup1{
  display: flex;
  justify-content: center;
  align-items: center;
  margin-top:2vh;
}
.otheroption1{
  display: flex;
  //align-items: flex-start;
  justify-content: space-between;
  flex-flow: column;
  padding: 0px;
  height: 32vh;
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

.option-container {
  max-height: 36vh; /* 设置选项框的最大高度 */
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
    justify-content: space-around;
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
  .icon {
    margin-right: 8px;
  }
  cursor: pointer
}

</style>
