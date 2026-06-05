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
        <span class = "finish-button-text" @click="redirectToChangeItemF">Finish Feeding</span>
      </button>
    </div>
  </div>
  <div class="container">
    <div class="modal" v-if="showModal">
      <button class="close-button" @click="closeModal">×</button>
      <div v-if="currentStep === 1">
        <div class="modal-header">
          Trouble picking up your choice? Let’s try a different approach.
        </div>
        <div class="modal-subheader">
          Select a skill and continue enjoying your meal.
        </div>
        <div class="skills">
          <div
            v-for="(skill, index) in skills"
            :key="index"
            :class="{ 'skillschoosing': true, active: activeIndex === index }"
            @click="setActive(index)"
          >
            <img :src="skill.img" :alt="skill.name" />
            <span>{{ skill.name }}</span>
          </div>
        </div>
        <div class="confirm-button-container">
          <button class="confirm-button" @click="goToStep2">Confirm</button>
        </div>
      </div>
      <div v-else-if="currentStep === 2" class="horizontal-layout">
        <div class="image-marker-container" @click="addMarker" ref="imageMarkerContainer" :style="{ backgroundImage: 'url(' + imageSrc + ')' }">
          <!--          <img ref="imageRef" :src="imageSrc" alt="Food Image" @load="getImageDimensions2" class="cimg"/>-->
          <div
            v-for="(marker, index) in markers"
            :key="index"
            class="marker"
            :style="{
        top: (marker.y * 100) + '%',
        left: (marker.x * 100) + '%'
      }"
          ></div>
          <svg v-if="markers.length === 2" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;">
            <line
              :x1="markers[0].x * imageWidth2"
              :y1="markers[0].y * imageHeight2"
              :x2="markers[1].x * imageWidth2"
              :y2="markers[1].y * imageHeight2"
              stroke="#a3d5ff" stroke-width="7" stroke-dasharray="5,5"/>
          </svg>
        </div>
        <div class="right-section">
          <div class="instruction">
            Click on one/two points on the image below:
          </div>
          <div class="button-container">
            <button class="reset-button" @click="resetMarkers">Reset Parameter</button>
            <button class="confirm-button" @click="redirectForConfirButton">Confirm</button>
          </div>
        </div>
      </div>
    </div>
    <div class="content">
      <div class="content-body">
        <div class = "left">
          <span class = "left-first-title">Choose your bite</span>

          <div class="left-side-image">
            <div class="image-wrapper" @click="stopCountdown">
              <img :src="imageSrc" :style="imageStyle" @load="getImageDimensions" class="responsive-image"/>
              <!--              <img src="../assets/food.png" alt="Left side image" class="responsive-image" />-->
              <!-- Display Boxes for Current Item -->
              <div
                v-for="(box, index) in currentItem.boxes"
                :key="index"
                :ref="`box-${index}`"
                class="box1"
                :style="{
                  position: 'absolute',
                  top: `${box.BoxTRatio * imageHeight}px`,
                  left: `${box.BoxLRatio * imageWidth}px`,
                  width: `${box.BoxWRatio * imageWidth}px`,
                  height: `${box.BoxHRatio * imageHeight}px`,
                  borderColor: selectedBox === index ? '#FFE699' : '#B4B4B4AD',
                  borderWidth: selectedBox === index ? '4px' : '2px',
                  backgroundColor: selectedBox === index ? 'rgba(128, 128, 128, 0.5)' : 'transparent',
                  zIndex: selectedBox === index ? '10' : '100' // change zIndex
                }"
                @click="handleBoxClick(index)"
              >
                <span
                  class="box-number"
                  :style="{
                    backgroundColor: selectedBox === index ? 'yellow' : 'red',
                    color: selectedBox === index ? 'black' : 'white'
                  }"
                >
                  {{ index + 1 }}
                </span>
              </div>
            </div>
          </div>


          <div class="button-container">
            <button class="custom-button" @click="redirectToChangeItem">Pickup Bite</button>
          </div>
          <div class="button-text">{{ countdownText }}</div>

        </div>
        <div class = "right">
          <span class = "right_text">Current Bite:</span>
          <div class="info-card" @click="stopCountdown">
            <img :src="currentItem.image" alt="current bite image" class="food-image" />
            <div class="info-content">
              <h3 class="food-name">{{ currentItem.name }}</h3>
              <p v-if="nextItem" class="food-detail">Next Bite: {{ nextItem.name }}</p>
<!--              <p class="food-timer">Executing In 00:10 Seconds</p>-->
            </div>
          </div>
          <span class="right_text">Change Food Items:</span>
          <div class="ingredient-list-wrapper">
            <div class="ingredient-list" @click="stopCountdown">
              <div v-if="nFoodTypes === 1" class="info-card">
                <p class="ingredient-name">Sorry, only one option available</p>
              </div>
              <div v-else v-for="(item, index) in items.slice(0, nFoodTypes-1)" :key="index" class="ingredient-item" @click="swapItems(index)">
                <img :src="item.image" alt="food item image" class="ingredient-image" />
                <p class="ingredient-name">{{ item.name }}</p>
              </div>
            </div>
          </div>
          <div class="buttonpart">
            <div>
            <span class="left_text">Choose your dip:</span>
            <div class="option-container" @click="stopCountdown">
              <div class="option" v-for="(option, index) in optionTexts" :key="index">
                <div class="optionboxfordips" :class="{ selected: selectedOption === index + 1 }" @click="selectOption(index + 1)">
                  <span class="option-box-text">{{ option }}</span>
                  <img class="metaballs" alt="option" src="../assets/optionbutton.png">
                </div>
              </div>
            </div>
          </div>

            <div v-if="!showModal" class="button2" @click="stopCountdown">
              <button class="button2" @click="showModal = true; currentStep = 1">
                <img class="button-setting" alt="Vue logo" src="../assets/drink (2).png">
                Execute Food Pickup <br> Skill Manually
              </button>
            </div>
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

// const imageRef = ref(null);

const allowedMessageType = 'std_msgs/String'

export default {
  name: 'TopicSubscriber',
  data () {
    return {
      optionTexts: [
        'No dip',
        'dip1',
        'dip2',
        'dip3',
        'dip4',
      ],
      username: USER,
      countdownInterval: null,
      countdownText: "Auto Executing in 00:15 seconds",
      countdown: 15,
      Pwidth: 0,
      Pheight: 0,
      BoxWRatio: 0,
      BoxHRatio: 0,
      BoxTRatio: 0,
      BoxLRatio: 0,
      imageStyle: {},
      sizeCheckInterval: null,
      markerSize: 30,
      nFoodTypes: 0,
      imageSrc: '',
      showModal: false,
      currentStep: 1,
      markerVisible: false,
      markerPosition: { x: 0, y: 0 },
      selectedOption: 1,
      selectedBox: 0,
      selectedPosition: { x: 0, y: 0 },
      showSettings: false,
      speed: 'moderate',
      items: [
        {
          name: 'Waiting',
          image: '', // Replace
          boxes: [
            { top: 50, left: 45 }, // Box 1
            { top: 55, left: 55 }, // Box 2
            { top: 18, left: 30 } // Box 3
          ]
        },
        {
          name: 'Waiting',
          image: '', // Replace
          boxes: [
            { top: 18, left: 30 }, // Box 1
            { top: 40, left: 18 }, // Box 2
            { top: 30, left: 20 } // Box 3
          ]
        },
        {
          name: 'Waiting',
          image: '',
          boxes: [
            { top: 30, left: 40 }, // Box 1
            { top: 40, left: 30 }, // Box 2
            { top: 30, left: 15 } // Box 3
          ]
        }
      ],
      currentItem: {
        name: 'Waiting for content',
        image: '',
        boxes: [
          { top: 25, left: 15 }, // Box 1
          { top: 40, left: 15 }, // Box 2
          { top: 15, left: 40 } // Box 3
        ]
      },
      nextItem: null,
      item2: {
        name: 'Meatballs',
        image: require('../assets/to.jpg'), // Replace with the correct image path
        nextBite: 'Spaghetti',
        timer: '00:10'
      },
      markers: [], // 用于存储标注点
      markerSrc: require('../assets/mouselogo.png'),
      maxMarkers: 1,

      receivedMessage: '', // Receive ROS messages
      inputMessage: '', // the input box used to send the ROS message
      subscribeTopic: '/ServerComm', // Subscribed ROS topics
      publishTopic: '/WebAppComm', // pubilsh
      listener: null, // listener
      publisher: null, // publish
      imageReceived: false,
      skills: [
        { name: 'Skewering', img: require('../assets/fork.png') },
        { name: 'Scooping', img: require('../assets/bowl.png') },
        { name: 'Dipping', img: require('../assets/dig.png') },
        { name: 'Twirling', img: require('../assets/twir.png') },
        { name: 'Grouping', img: require('../assets/move.png') },
        { name: 'Pushing', img: require('../assets/push.png') }
      ],
      imageWidth: 1,
      imageHeight: 1,
      imageWidth2: 1,
      imageHeight2: 1,
      activeIndex: null,
      previousSelectedOption: null,
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
    this.initPublisher()
    this.sizeCheckInterval = setInterval(this.checkSizes, 500);
    this.initSubscriber()
    this.startCountdown();
    this.publishMessageOnLoad()
    window.addEventListener('keydown', this.handleKeyDown)
    window.addEventListener('resize', this.getImageDimensions)
    this.activeIndex = 0
  },
  beforeUnmount () {
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval);
      this.countdownInterval = null;
      console.log("Countdown stopped before leaving the page.");
    }
    // clearInterval(this.sizeCheckInterval);
    window.removeEventListener('resize', this.getImageDimensions)
    window.removeEventListener('keydown', this.handleKeyDown)
  },
  beforeRouteLeave (to, from, next) {
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval);
      this.countdownInterval = null;
      console.log("Countdown stopped before leaving the page.");
    }
    if (this.listener) {
      console.log('Unsubscribing from listener...');
      this.listener.unsubscribe();
      this.listener = null;
    }

    if (this.publisher) {
      console.log('Unadvertising publisher...');
      this.publisher.unadvertise();
      this.publisher = null;
    }

    if (this.publisher && this.publisher.ros) {
      console.log('Closing ROS connection...');
      this.publisher.ros.close();
      this.publisher.ros = null;

      setTimeout(() => {
        console.log('ROS connection should be fully closed now.');
      }, 1000);
    }
    next();
  },
  methods: {
    selectOption(option) {
      this.selectedOption = this.selectedOption === option ? 0 : option;
      this.transcript = '';
    },
    stopCountdown() {
      if (this.countdownInterval) {
        clearInterval(this.countdownInterval);
        this.countdownInterval = null;
        console.log("Countdown stopped");
        this.countdownText = "";
      }
    },
    startCountdown() {
      this.countdownInterval = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown -= 1;
          this.updateCountdownText();
        } else {
          clearInterval(this.countdownInterval);
          console.log('Countdown completed');
          this.redirectToChangeItem();
        }
      }, 1000);
    },
    updateCountdownText() {
      this.countdownText = `Auto Executing in 00:${this.countdown.toString().padStart(2, '0')} seconds`;
    },

    checkSizes() {
      const container = this.$refs.imageMarkerContainer;
      const img = this.$refs.imageRef;

      if (container) {
        this.imageWidth2 = container.clientWidth;
        this.imageHeight2 = container.clientHeight;
      }

      if (img) {
        this.imageWidth = img.clientWidth;
        this.imageHeight = img.clientHeight;
      }
    },
    getImageDimensions() {
      setTimeout(() => {
        const container = this.$refs.imageMarkerContainer;
        if (container) {
          const containerRect = container.getBoundingClientRect();
          this.imageWidth2 = containerRect.width;
          this.imageHeight2 = containerRect.height;

          console.log('Container loaded');
          console.log('Container Width:', this.imageWidth2);
          console.log('Container Height:', this.imageHeight2);
        } else {
          console.log('Container reference is not available');
        }
        const img = document.querySelector('.responsive-image');
        if (img) {
          this.imageWidth = img.clientWidth;
          this.imageHeight = img.clientHeight;
          console.log('Image loaded');
          console.log('Image Width:', this.imageWidth);
          console.log('Image Height:', this.imageHeight);
        } else {
          console.log('Image reference is not available');
        }
      }, 500);
    },
    getImageDimensions2() {
      setTimeout(() => {
        const container = this.$refs.imageMarkerContainer;
        if (container) {
          const containerRect = container.getBoundingClientRect();
          this.imageWidth2 = containerRect.width;
          this.imageHeight2 = containerRect.height;

          console.log('Container loaded');
          console.log('Container Width:', this.imageWidth2);
          console.log('Container Height:', this.imageHeight2);
        } else {
          console.log('Container reference is not available');
        }
      }, 500);
    },
    publishMessageOnLoad() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_selection',
          status: 'ready_for_initial_data'
        })
      })

      if (this.publisher) {
        this.publisher.publish(message)
      } else {
        console.error('Publisher is not initialized yet.')
      }
    },
    handleRosMessage(message) {
      console.log('Raw message data:', message.data);
      console.log('Type of message.data:', typeof message.data);
      try {
        const parsedMessage = JSON.parse(message.data);
        if (parsedMessage.state === 'auto_time' && parsedMessage.status) {
          console.log('Matched auto_time message. Status:', parsedMessage.status);
          if (this.countdownInterval) {
            clearInterval(this.countdownInterval);
            this.countdownInterval = null;
            console.log("Countdown stopped");
          }
          // clearInterval(this.countdownInterval);
          // this.countdown = parseInt(parsedMessage.status, 10);
          // this.updateCountdownText();
          // console.log('Countdown reset to:', this.countdown);
          // this.startCountdown();
          this.countdown = parseInt(parsedMessage.status, 10);
          this.updateCountdownText();
          console.log('Countdown reset to:', this.countdown);
          //
          // // 启动新的倒计时
          this.startCountdown();
        } else {
          console.log('Received non-auto_time message or missing status field');
        }
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          if (typeof route === 'string') {
            this.$router.push(route);
          } else if (typeof route === 'object') {
            this.$router.push(route);
          }
        }
      } catch (error) {
        console.error('Failed to parse ROS message:', error);
      }
    },
    toggleSettings() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({ // JSON format
          state: 'task_selection',
          status: 'jump' // set status
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
    async updateItemsAndCurrentItem(data) {
      try {
        const parsedData = JSON.parse(data);
        if (parsedData.n_food_types) {
          this.nFoodTypes = parsedData.n_food_types;
        }
        // updates items
        if (Array.isArray(parsedData.data)) {
          for (let index = 0; index < parsedData.data.length; index++) {
            const itemData = parsedData.data[index];
            const itemName = Object.keys(itemData)[0];
            const itemBoxes = Array.isArray(itemData[itemName]) ? itemData[itemName].map(box => ({
              top: box[1],
              left: box[0],
              width: box[2],
              height: box[3]
            })) : [];

            const croppedImage = await this.cropImage(this.imageSrc, itemBoxes[0]);

            const image = new Image();
            image.src = this.imageSrc;
            await new Promise((resolve) => { image.onload = resolve; });

            const imageWidth = image.width;
            const imageHeight = image.height;

            const updatedBoxes = itemBoxes.map(box => this.calculateBoxRatios(box, imageWidth, imageHeight));


            this.items.splice(index, 1, {
              name: itemName,
              image: croppedImage,
              boxes: updatedBoxes
            });
          }
        }

        if (parsedData.current_bite) {
          const currentBiteName = Object.keys(parsedData.current_bite)[0];
          const currentBiteBoxes = Array.isArray(parsedData.current_bite[currentBiteName]) ? parsedData.current_bite[currentBiteName].map(box => ({
            top: box[1],
            left: box[0],
            width: box[2],
            height: box[3]
          })) : [];

          const croppedCurrentImage = await this.cropImage(this.imageSrc, currentBiteBoxes[0]);

          const image = new Image();
          image.src = this.imageSrc;
          await new Promise((resolve) => { image.onload = resolve; });

          const imageWidth = image.width;
          const imageHeight = image.height;

          const updatedCurrentBiteBoxes = currentBiteBoxes.map(box => this.calculateBoxRatios(box, imageWidth, imageHeight));


          this.currentItem = {
            name: currentBiteName,
            image: croppedCurrentImage,
            boxes: updatedCurrentBiteBoxes
          };
        }
      } catch (error) {
        console.error('Failed to parse JSON:', error);
        console.error('Received data:', data);
      }
    },
    calculateBoxRatios(box, imageWidth, imageHeight) {
      box.Pwidth = imageWidth;
      box.Pheight = imageHeight;
      box.BoxWRatio = box.width / imageWidth;
      box.BoxHRatio = box.height / imageHeight;
      box.BoxTRatio = box.top / imageHeight;
      box.BoxLRatio = box.left / imageWidth;
      return box;
    },
    cropImage(imageSrc, box, maxBoxWidth = 100, maxBoxHeight = 100) {
      return new Promise((resolve) => {
        const image = new Image();
        image.src = imageSrc;
        image.onload = () => {
          const canvas = document.createElement('canvas');
          const context = canvas.getContext('2d');

          const widthRatio = maxBoxWidth / box.width;
          const heightRatio = maxBoxHeight / box.height;
          const scaleRatio = Math.min(widthRatio, heightRatio);

          const scaledWidth = box.width * scaleRatio;
          const scaledHeight = box.height * scaleRatio;

          canvas.width = scaledWidth;
          canvas.height = scaledHeight;

          context.drawImage(
            image,
            box.left, box.top,
            box.width, box.height,
            0, 0,
            scaledWidth, scaledHeight
          );

          const croppedImageSrc = canvas.toDataURL('image/jpeg');
          resolve(croppedImageSrc);
        };

        image.onerror = () => {
          console.error('Failed to load image for cropping.');
          resolve('');
        };
      });
    },
    swapItems(index) {
      if (this.nFoodTypes === 1) {
        console.log('Item swapping is disabled because n_food_types is 1.');
        return;
      }
      const tempItem = {
        ...this.currentItem,
        selectedBox: this.selectedBox
      };
      this.currentItem = {
        ...this.items[index]
      };
      this.selectedBox = this.currentItem.selectedBox || 0;
      this.items[index] = {
        ...tempItem
      };
    },
    handleButtonClick() {
      this.publishMessageDrink();
      this.$router.push('/swithtodrink');
    },
    handleButtonClickMouth() {
      this.publishMessagePhysical();
      this.$router.push('/wiping');
    },
    handleBoxClick(index) {
      // this.publishMessageBox(index);
      this.selectedBox = index;
      console.log('s', this.selectedBox);
      const selectedBox = this.currentItem.boxes[index];
      this.cropImage(this.imageSrc, selectedBox)
        .then((croppedImage) => {
          this.currentItem.image = croppedImage;
        })
        .catch((error) => {
          console.error('Failed to crop image:', error);
        });
    },
    setActive (index) {
      this.activeIndex = index;
      this.maxMarkers = (index === 1) ? 2 : 1;
    },
    handleKeyDown (event) { // notify caregiver
      if (event.key === 'e' || event.key === 'E') {
        this.$router.push({ name: 'physical' })
      }
    },
    resetMarkers(){
      this.markers = [];
    },
    closeModal () {
      this.showModal = false;
      this.markers = [];
      this.currentStep = 1;
    },
    goToStep2 () {
      this.currentStep = 2
    },
    goToStep1 () {
      this.currentStep = 1
    },
    addMarker(event) {
      this.$nextTick(() => { // Make sure the DOM update is complete
        if (!this.$refs.imageMarkerContainer) {
          console.log('Container reference is null.');
          return;
        }
        if (this.markers.length < this.maxMarkers) {
          const containerRect = this.$refs.imageMarkerContainer.getBoundingClientRect();
          const x = (event.clientX - containerRect.left) / containerRect.width;
          const y = (event.clientY - containerRect.top) / containerRect.height;
          if (x >= 0 && x <= 1 && y >= 0 && y <= 1) {
            // 计算标记的尺寸
            const markerWidth = 60;
            const markerHeight = 60;

            // 将标记添加到 markers 数组
            this.markers.push({ x, y, width: markerWidth, height: markerHeight });

            console.log(`Marker added at: (${x}, ${y}), Width: ${markerWidth}, Height: ${markerHeight}`); // 输出坐标和尺寸到控制台
          }
        } else {
          console.log('Maximum number of markers reached.'); // 当达到最大标记数量时，输出日志
        }
      });
    },
    updateMarkers() {
      if (!this.$refs.container) return; // 容器不存在则直接返回
      // 由于位置是按百分比存储的，因此不需要重新计算
    },
    resetMarker () {
      this.markerVisible = false // 重置标记
    },
    redirectToChangeItem () {
      // this.publishMessageFood();
      this.publishAcquireFood();
      this.$router.push('/pickingup')
    },
    redirectForConfirButton() {
      // 检查标记点数是否满足 maxMarkers 的要求
      if (this.markers.length === this.maxMarkers) {
        // 如果标记点数满足要求，继续执行
        this.publishMessageFoodPosition(this.activeIndex, this.markers); // 传递 markers 数组
        this.$router.push('/pickingup');
      } else {
        // 如果标记点数不满足要求，显示错误消息或其他提示
        alert(`Please select ${this.maxMarkers} marker(s) before confirming.`);
      }
    },
    redirectToChangeItemdrink () {
      this.$router.push('/swithtodrink')
    },
    redirectToChangeItemF () {
      this.$router.push('/notify')
    },
    initPublisher() {
      const ros = new ROSLIB.Ros({
        url:  ROS_URL // changed
      })

      this.publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/WebAppComm', // 发布到 /talker 话题
        messageType: 'std_msgs/String' // 发布 std_msgs/String 类型的消息
      })
    },
    publishMessageDrink() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_selection',
          status: 'drink_pickup'
        }) // publish json
      })

      this.publisher.publish(message);
    },
    publishMessagePhysical() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_selection',
          status: 'mouth_wiping'
        })
      })

      this.publisher.publish(message);
    },
    publishMessageFood() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_selection',
          status: 'food_pickup'
        }) // publish json
      })

      this.publisher.publish(message);
    },
    publishAcquireFood() {
      if (!this.publisher) {
        console.log('not initialized.');
        return;
      }
      if (this.selectedBox !== null && this.currentItem) {
        const selectedBoxPosition = this.currentItem.boxes[this.selectedBox];
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'bite_selection',
            status: 'aquire_food',
            data: [this.currentItem.name, this.selectedBox + 1] // 发送食物名称和选中的框的索引
          })
        });

        this.publisher.publish(message);
        console.log('Published message:', message);
      } else {
        console.error('No box selected or currentItem is not set.');
      }
      if (this.selectedOption !== null && this.selectedOption !== 0) {
        const selectedText = this.optionTexts[this.selectedOption - 1];

        // 发送 order_selection 消息
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'dip_selection',
            status: selectedText
          })
        });
        this.publisher.publish(message);
        console.log('Order selection sent:', message);
      } else {
        console.log('No valid option selected.');
      }
    },
    publishMessageFoodPosition(index, positions) {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_skill_selection',
          status: index,
          positions: positions.map((position, positionIndex) => ({
            index: positionIndex + 1, // 标记第几对坐标
            x: position.x,
            y: position.y
          }))
        }) // publish json，包括所有坐标
      });
      this.publisher.publish(message);
    },
    initSubscriber () {
      const ros = new ROSLIB.Ros({
        url:  ROS_URL // changedURL
      });
      ros.on('connection', function() {
        console.log('Connected to WebSocket server');
      });

      ros.on('error', function(error) {
        console.error('WebSocket error:', error);
      });

      ros.on('close', function() {
        console.log('Connection closed');
      });

      const imageListener = new ROSLIB.Topic({
        ros: ros,
        name: '/camera/image/compressed', // 您的ROS图像话题名称
        messageType: 'sensor_msgs/CompressedImage'
      });

      if (!this.imageReceived) {
        imageListener.subscribe((message) => {
          // console.log('Received Compressed JPEG image message:', message);

          try {
            // 将图像数据转换为Base64编码的JPEG格式
            const base64Image = 'data:image/jpeg;base64,' + message.data;
            // console.log('Base64 Encoded JPEG Image:', base64Image);

            // 创建一个临时的图像对象来加载和验证图像
            const img = new Image();
            img.src = base64Image;
            // img.src = "";

            img.onload = () => {
              // 图像加载完成后更新 imageSrc
              this.imageSrc = base64Image;

              const Owidth = img.width;
              const Oheight = img.height;

              // 计算宽高比
              const aspectRatio = Owidth / Oheight;
              console.log(`Image Width: ${Owidth}, Height: ${Oheight}, Aspect Ratio: ${aspectRatio}`);

              this.imageStyle = {
                width: '43vw',
                height: `${43 / aspectRatio}vw`, // 高度按比例动态计算
                objectFit: 'cover',
                display: 'block'
              };


              // 标记已接收到图像
              this.imageReceived = true;
              this.selectedBox = 0

              // 执行裁剪和更新操作
              this.updateItemsAndCurrentItem(this.receivedMessage);

              // 停止订阅
              // imageListener.unsubscribe();
              console.log('Image subscription stopped.');
            };

            img.onerror = (error) => {
              console.error('Error loading image for cropping:', error);
            };

          } catch (error) {
            console.error('Error processing compressed JPEG image:', error);
          }
        });
      }
      this.listener = new ROSLIB.Topic({
        ros: ros,
        name: this.subscribeTopic,
        messageType: 'std_msgs/String'
      });

      this.listener.subscribe((message) => {
        console.log('Received message on ' + this.subscribeTopic + ': ' + message.data);
        this.receivedMessage = message.data;
        this.handleRosMessage(message)
        if (this.imageReceived) {
          this.updateItemsAndCurrentItem(this.receivedMessage);
        }
      });

      this.listener.subscribe((message) => {
        console.log('Received message on ' + this.subscribeTopic + ': ' + message.data);
        const data = JSON.parse(message.data);
        if (data && data.n_ordering) {
          const updatedOptionTexts = data.data.map((option) => `${option}`);
          this.optionTexts = [...updatedOptionTexts];
        }
      });
    },
    convertBGRAToBase64PNG (message) {
      const width = message.width;
      const height = message.height;
      const canvas = document.createElement('canvas');
      canvas.width = width;
      canvas.height = height;
      const context = canvas.getContext('2d');

      // 创建一个ImageData对象
      const imageData = context.createImageData(width, height);

      // 将ROS消息的数据（BGRA格式）拷贝到ImageData中
      const data = new Uint8Array(message.data);
      for (let i = 0; i < imageData.data.length; i += 4) {
        imageData.data[i] = data[i + 2];     // R
        imageData.data[i + 1] = data[i + 1]; // G
        imageData.data[i + 2] = data[i];     // B
        imageData.data[i + 3] = data[i + 3]; // A
      }

      // 将ImageData绘制到Canvas上
      context.putImageData(imageData, 0, 0);

      // 将Canvas内容转换为Base64编码的PNG字符串
      const base64Image = canvas.toDataURL('image/png');
      // console.log('Base64 Encoded PNG Image:', base64Image);

      // 更新imageSrc绑定到<img>标签
      this.imageSrc = base64Image;
    },

    rosImageToBase64 (message) {
      const base64String = btoa(
        new Uint8Array(message.data).reduce(
          (data, byte) => data + String.fromCharCode(byte),
          ''
        )
      );
      return base64String;
    }
  }
}
</script>

<style scoped>
.voice-start-button.recognizing {
  background-color: #6e7e8e; /* 语音识别中时按钮变为绿色 */
}
.metaballs{
  display: flex;
  flex-flow: column;
  align-items: center;
  object-fit: contain;
  width: 3vw;
  height: 6vh;
  margin: 0.5vh;
  //object-fit: cover;
  //height: 13vh;
  //top: 210px;
  //left: 716px;
  gap: 0px;
  opacity: 0px;
  border-radius: 20px;
  padding: 0vh;
  align-self: center;
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
.option{
  display: flex;
  //align-items: flex-start;
  justify-content: space-between;
  flex-flow: column;
  padding: 0px;
}
.optionboxfordips{
  width: 25vw;
  height: 8vh;
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
.option-container {
  max-height: 29vh; /* 设置选项框的最大高度 */
  overflow-y: auto;  /* 垂直滚动 */
  padding-right: 10px; /* 为滚动条留出空间 */
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
.selected-box {
  border-color: yellow !important;
  border-width: 20px !important;
  background-color: rgba(255, 255, 0, 0.3) !important;
}
.left-side-image {
  position: relative; /* 相对定位以包含绝对定位的框 */
  margin-right: 20px;
}

.image-wrapper {
  position: relative;
  width: 100%; /* 图片占满父容器宽度 */
}
.responsive-image1 {
  //width: 100%; /* 图片自适应宽度 */
  width: 43vw;
  //height: auto; /* 高度根据宽度自动调整 */
  object-fit: cover;
  display: block;
}
.box1 {
  position: absolute;
  width: 60px;
  height: 65px;
  box-sizing: border-box;
  cursor: pointer;
  border: 5px solid #FFE699;
}
.box-number {
  position: absolute;
  top: 0;
  left: 0;
  font-weight: bold;
  color: white;
  background-color: red;
  border-radius: 50%;
  width: 20px;
  height: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
}
.current-bite {
  //border: 1px solid #ddd;
  //padding: 10px;
  //margin-bottom: 20px;
  //display: flex;
  //align-items: center;
  //gap: 10px;
  //width: 300px; /* Current Bite Container Width */
  //height: 150px; /* Current Bite Container Height */
  display: flex;
  align-items: center;
  padding: 10px;
  border-radius: 10px;
  box-shadow: 0px 2px 5px rgba(0, 0, 0, 0.1);
  background-color: #f5f5f5;
  border: 1px solid #e0e0e0;
  height: 24vh;
  width: 45vw;
}

.current-bite-image {
  width: 100%;  /* Set to 100% to fill the container */
  height: 100%;
  object-fit: contain; /* Preserve the image aspect ratio, fit inside the container */
}

.food-items {
  display: flex;
  justify-content: space-around;
}

.food-item {
  cursor: pointer;
  text-align: center;
  width: 80px; /* Food Item Container Width */
  height: 80px; /* Food Item Container Height */
}

.food-item-image {
  width: 100%;  /* Set to 100% to fill the container */
  height: 100%;
  object-fit: contain; /* Preserve the image aspect ratio, fit inside the container */
}
.info-card {
  display: flex;
  align-items: center;
  padding: 10px;
  border-radius: 10px;
  box-shadow: 0px 2px 5px rgba(0, 0, 0, 0.1);
  background-color: #f5f5f5;
  border: 1px solid #e0e0e0;
  height: 15vh;
  width: 45vw;
}

.food-image {
  //width:15vw;
  //height: 23vh;
  border-radius: 8px;
  object-fit: cover;
  margin-right: 15px;
}

.info-content {
  display: flex;
  flex-direction: column;
}

.food-name {
  //font-size: 18px;
  font-weight: bold;
  margin: 0;
  color: #333;
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
  //line-height: 29.17px;
  line-height: 1.2em;
  text-align: left;
  word-break: break-word;
  overflow-wrap: break-word;
  white-space: normal;

}

.food-detail, .food-timer {
  font-family: Verdana;
  font-size: 18px;
  font-weight: 400;
  line-height: 1px;
  text-align: left;
  line-height: 0.9em;
  word-break: break-word;
  overflow-wrap: break-word;
  white-space: normal;

}
.ingredient-item.active {
  border: 4px solid #fc6423;
  box-shadow: 0 0 15px rgba(252, 100, 35, 0.9);
  background-color: #ffffff;
}

.ingredient-list-wrapper {
  display: flex;
  align-items: center;
  padding: 10px;
  overflow-x: scroll;
}

.ingredient-list {
  display: flex;
  gap: 10px; /* Space between items */
  width: 45vw;
  justify-content: normal;
  align-items: center;
}
.skillschoosing {
  text-align: center;
  transition: all 0.15s ease-in-out;
  cursor: pointer;
  //width: 120px;
  //height: 160px;
  box-sizing: border-box;
  box-shadow: 0 0 5px rgba(0, 0, 0, 0.1);
  display: flex;
  flex-direction: column;
  align-items: center;
  background-color: #f5f5f5;
  padding: 10px;
  border-radius: 8px;
  width: 15.5vw;
  height: 30vh;
  margin: 5px;
  justify-content: space-evenly;
}

.skillschoosing img {
  width: 8vw;
  height: 13.5vh;
  margin: 15px;
}

.skillschoosing span,
.skillschoosing p {
  font-size: 18px;
  color: #000000;
  font-family: Verdana;
  font-weight: 700;
  line-height: 20px;
  text-align: center;

}

.skillschoosing.active {
  border: 4px solid #fc6423;
  box-shadow: 0 0 15px rgba(252, 100, 35, 0.9);
  background-color: #ffffff;
}

.ingredient-item {
  display: flex;
  flex-direction: column;
  justify-content: space-between; /* Center items vertically */
  align-items: center; /* Center items horizontally */
  width: 120px; /* Set a consistent width */
  height: 180px; /* Set a consistent height for the card */
  border-radius: 20px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  background-color: white;
}



.ingredient-image {
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 15px;
  object-fit: cover;
  margin-top: 3vh;
}

.ingredient-name {
  font-family: Verdana;
  font-size: 18px;
  font-weight: 700;
  line-height: 1.2em;
  text-align: center;
  word-break: break-all;
  word-wrap: break-word;
  white-space: normal;

}

.right_text{
  font-family: Verdana;
  font-size: 18px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;

}
.left-first-title{
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 25px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;
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
.buttonpart {
  display: flex;
  align-items: center;
  justify-content: space-evenly;
  height: 30vh;
  width: 45vw;
}

.button2 {
  background-color: #FFE699;
  border-radius: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #FFE699;
  border-radius: 20px;
  width: 16vw;
  height: 25vh;
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
    height:10vh;
    width: 6vw;
  }
  .button-setting{
    height:7vh;
    width: 4vw;
    margin:10px
  }
  border: none;

  font-family: Verdana;
  font-size: 16px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;

}

.button-drink {
  height: 10vh;
  width: 10vw;
}

.show-button-container {
  background-color: #FFE699; /* 匹配其他按钮的背景颜色 */
  border-radius: 20px;
  width: 215px; /* 与其他按钮匹配的宽度 */
  height: 63px; /* 与其他按钮匹配的高度 */
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: 20px;
}

.show-button {
  background-color: transparent; /* 按钮背景透明，继承容器背景色 */
  border: none;
  color: black;
  font-size: 16px;
  cursor: pointer;
}

.box{
  height: 18vh;
  width: 14vw;
  //top: 200px;
  //left: 707px;
  display: flex;
  flex-flow: column;
  align-items: center;
  justify-content: space-between;
  margin:2px;
  padding: 10px;
  gap: 0px;
  border-radius: 9px 9px 9px 9px;
  opacity: 0px;
  background: #E0E0E0;
  .metaballs{
    width: 92px;
    height: 88px;
    top: 210px;
    left: 716px;
    gap: 0px;
    opacity: 0px;
  }
}
.optionboxfordips.selected {
  background-color: #6c7984; /* 选中后背景色 */
  color: white; /* 选中后文字颜色 */
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
  height: 12vh;
  top: 397px;
  left: 707px;
  gap: 0px;
  margin: 3px;
  border-radius: 20px 20px 20px 20px;
  border: 1px 0px 0px 0px;
  opacity: 0px;
  border: 1px solid #000000;
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
  height: 12vh;
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

.box-top{
  height:23vh;
  width:45vw ;
  display: flex;
  justify-content: left;
  background-color: #F2F2F2;
  border: none;
  padding: 15px;
  gap: 0px;
  border-radius: 24px;
  border: 5px ;
  border-color: #333333;
  opacity: 0px;

  .metaballs1{
    height: 18vh;
    width: 13vw;
  }
}
.button-container {
  display: flex;
  justify-content: center;
  align-items: center;
}

.custom-button {
  background-color: #6e7e91; /* Matches the greyish-blue color */
  color: white;
  border: none;
  padding: 10px 30px;
  border-radius: 20px; /* Rounded corners */
  cursor: pointer;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2); /* Optional shadow for depth */
  outline: none;
  height: 11vh; /* Set height to 11vh */
  width: 45vw; /* Set width to 48vw */
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
  line-height: 29.17px;
  text-align: center;
}

.custom-button:hover {
  background-color: #5b6a7b; /* Darker shade on hover */
}
.optionbox.selected {
  background-color: #6c7984; /* 选中后背景色 */
  color: white; /* 选中后文字颜色 */
}
.food{
  width: 45vw;
  height: 68vh;
}
.content{
  display: flex;
  //align-items: center;
  justify-content: center;
  flex-flow: column;
}
.content-body {
  display: flex;
  //align-items: flex-start;
  //align-items: center;
  justify-content: space-evenly;
  //padding: 20px;
  margin-top: 0px;
  .left{
    display: flex;
    justify-content: flex-start;
    align-items: baseline;
    flex-flow: column;
  }
  .right{
    display: flex;
    justify-content: flex-start;
    align-items: baseline;
    flex-flow: column;
  }
}
.threeboxes{
  display: flex;
  //align-items: flex-start;
  align-items: center;
  justify-content: space-between;
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
      z-index: 15;
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

.container {
  position: relative;
}

.modal {
  position: absolute;
  top: 0;
  left: 50%;
  transform: translateX(-50%);
  background-color: #708090; /* 使用指定的背景颜色 */
  border-radius: 12px;
  padding: 20px;
  //max-width: 900px; /* 扩大面板宽度 */
  margin: auto;
  text-align: left;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
  color: white; /* 更改字体颜色为白色 */
  z-index: 110; /* 确保面板位于按钮之上 */
  height: 84vh;
  width: 92vw;
}

.close-button {
  position: absolute;
  top: 10px;
  right: 10px;
  background: transparent;
  border: none;
  font-size: 20px;
  color: white;
  cursor: pointer;
}

.modal-header {
  font-size: 16px;
  font-weight: bold;
  margin-bottom: 10px;
  font-family: Verdana;
  font-weight: 400;
  line-height: 30px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;

}

.modal-subheader {
  margin-bottom: 20px;
  color: #ffffff; /* 副标题的字体颜色 */
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 30px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;

}

.skills {
  display: flex;
  justify-content: space-between;
  background-color: #6c7a89;
  padding: 20px;
  border-radius: 15px;
}


.skill img {
  width: 8vw;
  height: 13.5vh;
  margin: 15px;
}

.skill span {
  font-size: 18px;
  color: #000000; /* 调整技能文本的颜色为黑色 */
  font-family: Verdana;
  font-weight: 700;
  line-height: 20px;
  text-align: center;

}
.button-container {
  display: flex;
  justify-content: center;
  margin-top: 10px;
  flex-flow: column;
}
.confirm-button-container{
  display: flex;
  justify-content: center;
  margin-top: 20px;
}

.confirm-button, .reset-button, .back-button {
  background-color: #fce69e;
  border: none;
  border-radius: 8px;
  padding: 10px 20px;
  cursor: pointer;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
  color: black; /* 更改按钮文本颜色为黑色 */
  margin: 5px;
  width: 33vw;
  height: 12vh;
  font-family: Verdana;
  font-size: 32px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.22500000894069672px;
  text-align: center;

}

.horizontal-layout {
  display: flex;
  justify-content: space-between;
}
.image-marker-container {
  overflow: hidden;
  width: 45vw;  /* 视口宽度的50% */
  height: 45vh; /* 视口高度的50% */
  background-image: imageSrc; /* 使用指定路径的图片作为背景 */
  background-size: 100% 100%; /* 图片完全覆盖容器 */
  background-repeat: no-repeat; /* 防止图片重复 */
  position: relative;
  border: 1px solid #000;
  margin: 0 auto; /* 水平居中 */
  display: flex;
  align-items: center; /* 垂直居中 */
  justify-content: center; /* 水平居中 */
}
.image-container {
  position: relative;
  flex: 2; /* 更改 flex 值以分配更多空间给图片 */
  display: flex;
  justify-content: center;
  margin-right: 20px;
  width: 65vw;
  height: 60vh;
}

.right-section {
  flex: 1;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  align-items: flex-start; /* 左对齐文本 */
  margin-left: 6vh;
}

.instruction {
  margin-bottom: 20px;
  max-width: 100%; /* 限制最大宽度 */
  word-wrap: break-word; /* 确保文本换行 */
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 30px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;

}
.marker{
  width: 30px; /* 图标的宽度 */
  height: 30px; /* 图标的高度 */
  background-image: url('../assets/mouselogo.png'); /* 将红点替换为图标 */
  background-size: cover; /* 使图标完全覆盖容器 */
  position: absolute;
  transform: translate(-50%, -50%);
}

.marker1 {
  position: absolute;
  width: 60px;
  height: 60px;
  //background-image: url('../assets/mouselogo.png'); /* 使用本地标记图标路径 */
  background-size: contain;
  background-repeat: no-repeat;
  pointer-events: none; /* 确保标记不会阻止点击事件 */
}

.show-button-container {
  position: absolute;
  top: 100%;
  left: 50%;
  transform: translateX(-50%);
  z-index: 0; /* 确保按钮在面板之下 */
}

.show-button {
  background-color: #6e7e8e;
  border: none;
  border-radius: 8px;
  color: white;
  padding: 10px 20px;
  cursor: pointer;
  font-size: 16px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
  background-color: transparent; /* 按钮背景透明，继承容器背景色 */
  border: none;
  color: black;
  font-size: 16px;
  cursor: pointer;
}

.image-container {
  //position: relative;
  //display: inline-block;
  display: flex;
  justify-content: flex-start;
  align-items: flex-start;
  .cimg{
    width: 80%;
    height: 100%;
  }
}

.image-container clickimg {
  width: 80%;
  height: 100%;
  //object-fit: contain; /* 保持图片比例，防止拉伸 */
}
.image-container cimg {
  width: 80%;
  height: 100%;
  //object-fit: contain; /* 保持图片比例，防止拉伸 */
}
</style>





