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

const allowedMessageType = 'std_msgs/String'

export default {
  name: 'TopicSubscriber',
  data () {
    return {
      ros: null,
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
      countdown: 1000,
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
      items: [
        {
          name: 'Waiting',
          image: '', 
          boxes: [
            { top: 50, left: 45 }, 
            { top: 55, left: 55 }, 
            { top: 18, left: 30 } 
          ]
        },
        {
          name: 'Waiting',
          image: '', 
          boxes: [
            { top: 18, left: 30 }, 
            { top: 40, left: 18 }, 
            { top: 30, left: 20 } 
          ]
        },
        {
          name: 'Waiting',
          image: '',
          boxes: [
            { top: 30, left: 40 }, 
            { top: 40, left: 30 }, 
            { top: 30, left: 15 } 
          ]
        }
      ],
      currentItem: {
        name: 'Waiting for content',
        image: '',
        boxes: [
          { top: 25, left: 15 }, 
          { top: 40, left: 15 }, 
          { top: 15, left: 40 } 
        ]
      },
      nextItem: null,
      item2: {
        name: 'Meatballs',
        image: require('../assets/to.jpg'), 
        nextBite: 'Spaghetti',
        timer: '00:10'
      },
      markers: [], 
      markerSrc: require('../assets/mouselogo.png'),
      maxMarkers: 1,

      receivedMessage: '', 
      inputMessage: '',   
      listener: null, 
      publisher: null, 
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
        
        if (this.selectedOption !== 0) {
          this.previousSelectedOption = this.selectedOption;
        }
        this.selectedOption = 0;
      } else {
        
        if (this.previousSelectedOption !== null) {
          this.selectedOption = this.previousSelectedOption;
          this.previousSelectedOption = null; 
        }
      }
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.sizeCheckInterval = setInterval(this.checkSizes, 500);
    this.initSubscriber()
    this.startCountdown();
    this.publishMessageOnLoad()
    window.addEventListener('resize', this.getImageDimensions)
    this.activeIndex = 0
  },
  beforeUnmount () {
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval);
      this.countdownInterval = null;
    }
    
    window.removeEventListener('resize', this.getImageDimensions)
  },
  beforeRouteLeave (to, from, next) {
    if (this.countdownInterval) {
      clearInterval(this.countdownInterval);
      this.countdownInterval = null;
    }
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
    selectOption(option) {
      this.selectedOption = this.selectedOption === option ? 0 : option;
      this.transcript = '';
    },
    stopCountdown() {
      if (this.countdownInterval) {
        clearInterval(this.countdownInterval);
        this.countdownInterval = null;
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

        } else {
        }
        const img = document.querySelector('.responsive-image');
        if (img) {
          this.imageWidth = img.clientWidth;
          this.imageHeight = img.clientHeight;
        } else {
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
      }
    },
    handleRosMessage(message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        if (parsedMessage.state === 'auto_time' && parsedMessage.status) {
          if (this.countdownInterval) {
            clearInterval(this.countdownInterval);
            this.countdownInterval = null;
          }

          this.countdown = parseInt(parsedMessage.status, 10);
          this.updateCountdownText();

          this.startCountdown();
        } else {
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
      }
      this.$router.push('/task_selection')
    },
    async updateItemsAndCurrentItem(data) {
      try {
        const parsedData = JSON.parse(data);
        if (parsedData.n_food_types) {
          this.nFoodTypes = parsedData.n_food_types;
        }
        
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
          resolve('');
        };
      });
    },
    swapItems(index) {
      if (this.nFoodTypes === 1) {
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
      this.$router.push('/robot_executing');
    },
    handleButtonClickMouth() {
      this.publishMessagePhysical();
      this.$router.push('/robot_executing');
    },
    handleBoxClick(index) {
      
      this.selectedBox = index;
      const selectedBox = this.currentItem.boxes[index];
      this.cropImage(this.imageSrc, selectedBox)
        .then((croppedImage) => {
          this.currentItem.image = croppedImage;
        })
        .catch((error) => {
        });
    },
    setActive (index) {
      this.activeIndex = index;
      this.maxMarkers = (index === 1) ? 2 : 1;
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
      this.$nextTick(() => { 
        if (!this.$refs.imageMarkerContainer) {
          return;
        }
        if (this.markers.length < this.maxMarkers) {
          const containerRect = this.$refs.imageMarkerContainer.getBoundingClientRect();
          const x = (event.clientX - containerRect.left) / containerRect.width;
          const y = (event.clientY - containerRect.top) / containerRect.height;
          if (x >= 0 && x <= 1 && y >= 0 && y <= 1) {
            
            const markerWidth = 60;
            const markerHeight = 60;

            this.markers.push({ x, y, width: markerWidth, height: markerHeight });

          }
        } else {
        }
      });
    },
    updateMarkers() {
      if (!this.$refs.container) return; 
      
    },
    resetMarker () {
      this.markerVisible = false 
    },
    redirectToChangeItem () {
      
      this.publishAcquireFood();
      this.$router.push('/robot_executing')
    },
    redirectForConfirButton() {
      
      if (this.markers.length === this.maxMarkers) {
        
        this.publishMessageFoodPosition(this.activeIndex, this.markers); 
        this.$router.push('/robot_executing');
      } else {
        
        alert(`Please select ${this.maxMarkers} marker(s) before confirming.`);
      }
    },
    redirectToChangeItemdrink () {
      this.$router.push('/robot_executing')
    },
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot', 
        messageType: 'std_msgs/String' 
      })
    },
    publishMessageDrink() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_selection',
          status: 'drink_pickup'
        }) 
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
        }) 
      })

      this.publisher.publish(message);
    },
    publishAcquireFood() {
      if (!this.publisher) {
        return;
      }
      if (this.selectedBox !== null && this.currentItem) {
        const selectedBoxPosition = this.currentItem.boxes[this.selectedBox];
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'bite_selection',
            status: 'acquire_food',
            data: [this.currentItem.name, this.selectedBox + 1] 
          })
        });

        this.publisher.publish(message);
      } else {
      }
      if (this.selectedOption !== null && this.selectedOption !== 0) {
        const selectedText = this.optionTexts[this.selectedOption - 1];

        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'dip_selection',
            status: selectedText
          })
        });
        this.publisher.publish(message);
      } else {
      }
    },
    publishMessageFoodPosition(index, positions) {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'bite_skill_selection',
          status: index,
          positions: positions.map((position, positionIndex) => ({
            index: positionIndex + 1, 
            x: position.x,
            y: position.y
          }))
        }) 
      });
      this.publisher.publish(message);
    },
    initSubscriber () {
      const imageListener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/camera/image/compressed', 
        messageType: 'sensor_msgs/CompressedImage'
      });

      if (!this.imageReceived) {
        imageListener.subscribe((message) => {

          try {
            
            const base64Image = 'data:image/jpeg;base64,' + message.data;

            const img = new Image();
            img.src = base64Image;

            img.onload = () => {
              
              this.imageSrc = base64Image;

              const Owidth = img.width;
              const Oheight = img.height;

              const aspectRatio = Owidth / Oheight;

              this.imageStyle = {
                width: '43vw',
                height: `${43 / aspectRatio}vw`, 
                objectFit: 'cover',
                display: 'block'
              };

              this.imageReceived = true;
              this.selectedBox = 0

              this.updateItemsAndCurrentItem(this.receivedMessage);

            };

            img.onerror = (error) => {
            };

          } catch (error) {
          }
        });
      }
      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: this.subscribeTopic,
        messageType: 'std_msgs/String'
      });

      this.listener.subscribe((message) => {
        this.receivedMessage = message.data;
        this.handleRosMessage(message)
        if (this.imageReceived) {
          this.updateItemsAndCurrentItem(this.receivedMessage);
        }
      });

      this.listener.subscribe((message) => {
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

      const imageData = context.createImageData(width, height);

      const data = new Uint8Array(message.data);
      for (let i = 0; i < imageData.data.length; i += 4) {
        imageData.data[i] = data[i + 2];     
        imageData.data[i + 1] = data[i + 1]; 
        imageData.data[i + 2] = data[i];     
        imageData.data[i + 3] = data[i + 3]; 
      }

      context.putImageData(imageData, 0, 0);

      const base64Image = canvas.toDataURL('image/png');

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
  background-color: #6e7e8e; 
}
.metaballs{
  display: flex;
  flex-flow: column;
  align-items: center;
  object-fit: contain;
  width: 3vw;
  height: 6vh;
  margin: 0.5vh;
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
  max-height: 29vh; 
  overflow-y: auto;  
  padding-right: 10px; 
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
  position: relative; 
  margin-right: 20px;
}

.image-wrapper {
  position: relative;
  width: 100%; 
}
.responsive-image1 {
  width: 43vw;
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
  width: 100%;  
  height: 100%;
  object-fit: contain; 
}

.food-items {
  display: flex;
  justify-content: space-around;
}

.food-item {
  cursor: pointer;
  text-align: center;
  width: 80px; 
  height: 80px; 
}

.food-item-image {
  width: 100%;  
  height: 100%;
  object-fit: contain; 
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
  border-radius: 8px;
  object-fit: cover;
  margin-right: 15px;
}

.info-content {
  display: flex;
  flex-direction: column;
}

.food-name {
  font-weight: bold;
  margin: 0;
  color: #333;
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
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
  gap: 10px; 
  width: 45vw;
  justify-content: normal;
  align-items: center;
}
.skillschoosing {
  text-align: center;
  transition: all 0.15s ease-in-out;
  cursor: pointer;
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
  justify-content: space-between; 
  align-items: center; 
  width: 120px; 
  height: 180px; 
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
  background-color: #FFE699; 
  border-radius: 20px;
  width: 215px; 
  height: 63px; 
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: 20px;
}

.show-button {
  background-color: transparent; 
  border: none;
  color: black;
  font-size: 16px;
  cursor: pointer;
}

.box{
  height: 18vh;
  width: 14vw;
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
  background-color: #6c7984; 
  color: white; 
}
.option{
  display: flex;
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
  background-color: #6e7e91; 
  color: white;
  border: none;
  padding: 10px 30px;
  border-radius: 20px; 
  cursor: pointer;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2); 
  outline: none;
  height: 11vh; 
  width: 45vw; 
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
  line-height: 29.17px;
  text-align: center;
}

.custom-button:hover {
  background-color: #5b6a7b; 
}
.optionbox.selected {
  background-color: #6c7984; 
  color: white; 
}
.food{
  width: 45vw;
  height: 68vh;
}
.content{
  display: flex;
  justify-content: center;
  flex-flow: column;
}
.content-body {
  display: flex;
  justify-content: space-evenly;
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
    .setting-container {
      position: relative;
    }
    .settings-button {
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
    .settings-button span {
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
  background-color: #708090; 
  border-radius: 12px;
  padding: 20px;
  margin: auto;
  text-align: left;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
  color: white; 
  z-index: 110; 
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
  color: #ffffff; 
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
  color: #000000; 
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
  color: black; 
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
  width: 45vw;  
  height: 45vh; 
  background-image: imageSrc; 
  background-size: 100% 100%; 
  background-repeat: no-repeat; 
  position: relative;
  border: 1px solid #000;
  margin: 0 auto; 
  display: flex;
  align-items: center; 
  justify-content: center; 
}
.image-container {
  position: relative;
  flex: 2; 
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
  align-items: flex-start; 
  margin-left: 6vh;
}

.instruction {
  margin-bottom: 20px;
  max-width: 100%; 
  word-wrap: break-word; 
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 30px;
  letter-spacing: 0.17499999701976776px;
  text-align: left;

}
.marker{
  width: 30px; 
  height: 30px; 
  background-image: url('../assets/mouselogo.png'); 
  background-size: cover; 
  position: absolute;
  transform: translate(-50%, -50%);
}

.marker1 {
  position: absolute;
  width: 60px;
  height: 60px;
  background-size: contain;
  background-repeat: no-repeat;
  pointer-events: none; 
}

.show-button-container {
  position: absolute;
  top: 100%;
  left: 50%;
  transform: translateX(-50%);
  z-index: 0; 
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
  background-color: transparent; 
  border: none;
  color: black;
  font-size: 16px;
  cursor: pointer;
}

.image-container {
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
}
.image-container cimg {
  width: 80%;
  height: 100%;
}
</style>

