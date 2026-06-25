<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Choose your bite</div>
      </div>
    </div>

    <div class="modal-overlay" v-if="showModal">
      <div class="modal-card">
        <button class="modal-close" @click="closeModal">×</button>

        <template v-if="currentStep === 1">
          <p class="skill-modal-hdr">Trouble picking up your choice? Let's try a different approach.</p>
          <p class="skill-modal-sub">Select a skill and continue enjoying your meal.</p>
          <div class="skill-grid">
            <div
              v-for="(skill, index) in skills"
              :key="index"
              class="skill-card"
              :class="{ sel: activeIndex === index }"
              @click="setActive(index)"
            >
              <img class="sk-ico" :src="skill.img" :alt="skill.name" />
              <span class="sk-lbl">{{ skill.name }}</span>
            </div>
          </div>
          <button class="btn md amber w100" style="margin-top:1.5vh" @click="goToStep2">Confirm</button>
        </template>

        <template v-else-if="currentStep === 2">
          <p class="mark-instruction">Click on one or two points on the image below:</p>
          <div
            class="cam mark-cam"
            @click="addMarker"
            ref="imageMarkerContainer"
            :style="{ backgroundImage: 'url(' + imageSrc + ')' }"
          >
            <div
              v-for="(marker, index) in markers"
              :key="index"
              class="mark-dot"
              :style="{ top: (marker.y * 100) + '%', left: (marker.x * 100) + '%' }"
            ></div>
            <svg v-if="markers.length === 2" class="mark-line-svg">
              <line
                :x1="markers[0].x * imageWidth2"
                :y1="markers[0].y * imageHeight2"
                :x2="markers[1].x * imageWidth2"
                :y2="markers[1].y * imageHeight2"
                stroke="#2EC4B6" stroke-width="4" stroke-dasharray="6,6"/>
            </svg>
          </div>
          <div class="mark-actions">
            <button class="btn md ghost" @click="resetMarkers">Reset Parameter</button>
            <button class="btn md amber" @click="redirectForConfirButton">Confirm</button>
          </div>
        </template>
      </div>
    </div>

    <div class="bd det-bd">
      <div class="bite-body">
        <div class="bite-left">
          <div class="cam bite-cam">
            <div class="image-wrapper" @click="stopCountdown">
              <img :src="imageSrc" :style="imageStyle" @load="getImageDimensions" class="responsive-image" alt="Plate"/>

              <div
                v-for="(box, index) in currentItem.boxes"
                :key="index"
                :ref="`box-${index}`"
                class="bite-box"
                :style="{
                  position: 'absolute',
                  top: `${box.BoxTRatio * imageHeight}px`,
                  left: `${box.BoxLRatio * imageWidth}px`,
                  width: `${box.BoxWRatio * imageWidth}px`,
                  height: `${box.BoxHRatio * imageHeight}px`,
                  borderColor: selectedBox === index ? '#F0A500' : '#8BA8C4AD',
                  borderWidth: selectedBox === index ? '4px' : '2px',
                  backgroundColor: selectedBox === index ? 'rgba(240, 165, 0, 0.18)' : 'transparent',
                  zIndex: selectedBox === index ? '10' : '100'
                }"
                @click="handleBoxClick(index)"
              >
                <span
                  class="bite-box-num"
                  :style="{
                    backgroundColor: selectedBox === index ? '#F0A500' : '#243C54',
                    color: selectedBox === index ? '#0D1B2A' : '#F5F0E8'
                  }"
                >
                  {{ index + 1 }}
                </span>
              </div>
            </div>
          </div>

          <button class="btn md amber w100" @click="redirectToChangeItem">Pickup Bite</button>
          <p class="cdown">{{ countdownText }}</p>
        </div>

        <div class="bite-side">
          <span class="field-lbl">Current bite</span>
          <div class="bite-current" @click="stopCountdown">
            <img :src="currentItem.image" alt="current bite" class="bc-img" />
            <div>
              <div class="bc-name">{{ currentItem.name }}</div>
              <div class="bc-next" v-if="nextItem">Next: {{ nextItem.name }}</div>
            </div>
          </div>

          <span class="field-lbl">Swap food item</span>
          <div class="swap-row" @click="stopCountdown">
            <div v-if="nFoodTypes === 1" class="swap-empty">Only one option available</div>
            <div
              v-else
              v-for="(item, index) in items.slice(0, nFoodTypes-1)"
              :key="index"
              class="swap-item"
              @click="swapItems(index)"
            >
              <img :src="item.image" alt="food item" />
              <span class="swap-name">{{ item.name }}</span>
            </div>
          </div>

          <span class="field-lbl">Choose your dip</span>
          <div class="dip-opts" @click="stopCountdown">
            <div
              class="dip-opt"
              v-for="(option, index) in optionTexts"
              :key="index"
              :class="{ sel: selectedOption === index + 1 }"
              @click="selectOption(index + 1)"
            >
              <span>{{ option }}</span>
              <div class="och" style="width:16px;height:16px;font-size:9px" v-if="selectedOption === index + 1">✓</div>
            </div>
          </div>

          <div class="skill-fallback" v-if="!showModal">
            <span class="skill-fallback-lbl">Trouble picking this up?</span>
            <button class="skill-fallback-btn" title="Execute Pickup Skill Manually" @click="showModal = true; currentStep = 1">🛠️</button>
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
      countdownText: "Auto-confirming in 15s",
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
      this.countdownText = `Auto-confirming in ${this.countdown}s`;
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

              this.imageStyle = {
                width: '100%',
                height: '100%',
                objectFit: 'contain',
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
        name: '/robot_to_webapp',
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
.modal-overlay {
  position: fixed;
  inset: 0;
  background: rgba(13, 27, 42, 0.85);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 110;
}

.modal-card {
  background: var(--s1);
  border: 1px solid var(--bd);
  border-radius: var(--rl);
  padding: 3vh 3vw;
  width: 80vw;
  max-width: 900px;
  max-height: 84vh;
  display: flex;
  flex-direction: column;
  position: relative;
  box-shadow: 0 20px 56px rgba(0, 0, 0, .65);
}

.modal-close {
  position: absolute;
  top: 1.5vh;
  right: 1.5vw;
  background: transparent;
  border: none;
  font-size: 2.4vh;
  color: var(--tm);
  cursor: pointer;
}

.skill-modal-hdr {
  font: normal 3vh/1.4 Georgia, serif;
  color: var(--t);
  margin-bottom: .5vh;
  padding-right: 3vw;
}

.skill-modal-sub {
  font-size: 2vh;
  color: var(--tm);
  margin-bottom: 2vh;
}

.skill-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.2vw;
}

.skill-card {
  background: var(--s2);
  border-radius: var(--r);
  border: 2px solid transparent;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: .8vh;
  padding: 2vh 1vw;
  cursor: pointer;
}

.skill-card.sel {
  border-color: var(--a);
  background: rgba(240, 165, 0, .08);
}

.sk-ico {
  width: 5vw;
  height: 9vh;
  object-fit: contain;
  filter: invert(1);
}

.sk-lbl {
  font-size: 2vh;
  font-weight: 700;
  color: var(--t);
}

.mark-instruction {
  font-size: 2.3vh;
  color: var(--t);
  text-align: center;
  margin-bottom: 1.5vh;
}

.mark-cam {
  flex: 1;
  min-height: 36vh;
  background-size: cover;
  background-position: center;
  background-repeat: no-repeat;
  position: relative;
  cursor: crosshair;
}

.mark-dot {
  position: absolute;
  width: 18px;
  height: 18px;
  border-radius: 50%;
  background: var(--a2);
  border: 2px solid var(--t);
  transform: translate(-50%, -50%);
  pointer-events: none;
}

.mark-line-svg {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.mark-actions {
  display: flex;
  gap: 1.2vw;
  margin-top: 1.5vh;
}

.mark-actions .btn {
  flex: 1;
}

.bite-body {
  display: grid;
  grid-template-columns: 1.15fr 1fr;
  gap: 1.5vw;
  flex: 1;
  min-height: 0;
}

.bite-left {
  display: flex;
  flex-direction: column;
  gap: .8vh;
  min-height: 0;
}

.bite-cam {
  flex: 1;
  min-height: 0;
}

.image-wrapper {
  position: relative;
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.bite-box {
  box-sizing: border-box;
  cursor: pointer;
  border-style: solid;
  border-radius: 4px;
}

.bite-box-num {
  position: absolute;
  top: -10px;
  left: -10px;
  font-weight: bold;
  border-radius: 50%;
  width: 22px;
  height: 22px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.3vh;
}

.bite-side {
  display: flex;
  flex-direction: column;
  gap: .8vh;
  min-height: 0;
}

.bite-current {
  background: var(--s2);
  border-radius: 12px;
  padding: 1vh 1vw;
  display: flex;
  gap: 1vw;
  align-items: center;
  cursor: pointer;
}

.bc-img {
  width: 6vh;
  height: 6vh;
  border-radius: 8px;
  object-fit: cover;
  flex-shrink: 0;
}

.bc-name {
  font-size: 2.3vh;
  font-weight: 700;
  color: var(--t);
}

.bc-next {
  font-size: 1.7vh;
  color: var(--tm);
}

.swap-row {
  display: flex;
  gap: .6vw;
  overflow-x: auto;
  min-height: 8vh;
}

.swap-empty {
  font-size: 1.8vh;
  color: var(--tm);
  padding: 1vh;
}

.swap-item {
  flex-shrink: 0;
  width: 9vh;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: .4vh;
  cursor: pointer;
}

.swap-item img {
  width: 6vh;
  height: 6vh;
  border-radius: 10px;
  object-fit: cover;
  background: var(--s2);
}

.swap-name {
  font-size: 1.45vh;
  color: var(--tm);
  text-align: center;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  max-width: 9vh;
}

.dip-opts {
  display: flex;
  flex-direction: column;
  gap: .6vh;
  overflow-y: auto;
  flex: 1;
  min-height: 0;
}

.dip-opt {
  background: var(--s2);
  border-radius: 10px;
  border: 2px solid transparent;
  padding: 1vh 1.2vw;
  display: flex;
  align-items: center;
  justify-content: space-between;
  cursor: pointer;
  font-size: 1.9vh;
  color: var(--t);
  flex-shrink: 0;
}

.dip-opt.sel {
  border-color: var(--a);
  background: rgba(240, 165, 0, .08);
}

.skill-fallback {
  margin-top: auto;
  display: flex;
  align-items: center;
  justify-content: flex-end;
  gap: 1vw;
}

.skill-fallback-lbl {
  font-size: 2.2vh;
  color: var(--tm);
  text-align: right;
  line-height: 1.3;
  max-width: 22vw;
}

.skill-fallback-btn {
  width: 9vh;
  height: 9vh;
  border-radius: 16px;
  flex-shrink: 0;
  background: transparent;
  border: 2px solid var(--s3);
  color: var(--tm);
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 3.6vh;
  cursor: pointer;
}
</style>

