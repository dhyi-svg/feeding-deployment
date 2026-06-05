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
        <div class="image-container" @click="addMarker">
          <img :src="imageSrc" alt="Food Image" ref="imageRef" class="cimg" />
          <img
            v-for="(marker, index) in markers"
            :key="index"
            :src="markerSrc"
            :style="{ top: marker.y + 'px', left: marker.x + 'px' }"
            class="marker"
            alt="Marker"
          />
        </div>
        <div class="right-section">
          <div class="instruction">
            Click on one/two points on the image below:
          </div>
          <div class="button-container">
            <button class="reset-button" @click="resetMarker">Reset Parameter</button>
            <button class="confirm-button" @click="redirectToChangeItem">Confirm</button>
          </div>
        </div>
      </div>
    </div>
    <div class="content">
      <div class="content-body">
        <!-- 左边 -->
        <div class="left">
          <span class="left-first-title">Choose your bite</span>

          <div class="left-side-image">
            <div class="image-wrapper">
              <img src="../assets/food.png" alt="Left side image" class="responsive-image" />
              <!-- Display Boxes for Current Item -->
              <div
                v-for="(box, index) in currentItem.boxes"
                :key="index"
                :ref="`box-${index}`"
                class="box1"
                :style="{
                  top: `calc(${box.top}% - 25px)`,
                  left: `calc(${box.left}% - 25px)`,
                  width: '60px',
                  height: '80px',
                  borderColor: selectedBox === index ? '#FFE699' : '#B4B4B4AD',
                  borderWidth: selectedBox === index ? '4px' : '2px',
                  backgroundColor: selectedBox === index ? 'rgba(128, 128, 128, 0.3)' : 'transparent'
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
            <button class="custom-button" @click="redirectToChangeItem">Acquire Food</button>
          </div>
        </div>
        <div class="right">
          <span class="right_text">Current Bite:</span>
          <div class="info-card">
            <img :src="currentItem.image" alt="current bite image" class="food-image" />
            <div class="info-content">
              <h3 class="food-name">{{ currentItem.name }}</h3>
              <p v-if="nextItem" class="food-detail">Next Bite: {{ nextItem.name }}</p>
              <p class="food-timer">Executing In 00:10 Seconds</p>
            </div>
          </div>
          <span class="right_text">Change Food Items:</span>

          <div class="ingredient-list">
            <div v-for="(item, index) in items" :key="index" class="ingredient-item" @click="swapItems(index)">
              <img :src="item.image" alt="food item image" class="ingredient-image" />
              <p class="ingredient-name">{{ item.name }}</p>
            </div>
          </div>

          <div class="buttonpart">
            <div v-if="!showModal" class="button2">
              <button class="button2" @click="redirectToChangeItemdrink">
                <img class="button-drink" alt="Vue logo" src="../assets/drin.png">
                Switch to Drink
              </button>
            </div>
            <div v-if="!showModal" class="button2">
              <button class="button2" @click="showModal = true">
                <img class="button-setting" alt="Vue logo" src="../assets/drink%20(2).png">
                Execute Pickup Skill Manually
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { ref } from 'vue';

export default {
  name: 'TopicSubscriber',
  data() {
    return {
      showModal: false,
      currentStep: 1,
      selectedBox: null,
      showSettings: false,
      speed: 'moderate',
      items: [
        {
          name: 'Tomatoes',
          image: require('../assets/to.jpg'),
          boxes: [
            { top: 50, left: 45 },
            { top: 55, left: 55 },
            { top: 18, left: 30 },
          ],
        },
        {
          name: 'Spaghetti',
          image: require('../assets/spa.jpg'),
          boxes: [
            { top: 18, left: 30 },
            { top: 40, left: 18 },
            { top: 30, left: 20 },
          ],
        },
        {
          name: 'Coriander',
          image: require('../assets/cori.jpg'),
          boxes: [
            { top: 30, left: 40 },
            { top: 40, left: 30 },
            { top: 30, left: 15 },
          ],
        },
      ],
      currentItem: {
        name: 'Meatballs',
        image: require('../assets/meal.png'),
        boxes: [
          { top: 25, left: 15 },
          { top: 40, left: 15 },
          { top: 15, left: 40 },
        ],
      },
      nextItem: null,
      activeIndex: null,
    };
  },
  methods: {
    swapItems(index) {
      const tempItem = this.currentItem;
      this.currentItem = this.items[index];
      this.items[index] = tempItem;
      this.selectedBox = null; // 取消之前选中的框
    },
    handleBoxClick(index) {
      this.selectedBox = index; // 记录选中的框的索引
    },
    setActive(index) {
      this.activeIndex = index;
    },
    closeModal() {
      this.showModal = false;
    },
    goToStep2() {
      this.currentStep = 2;
    },
    addMarker(event) {
      if (this.markers.length < 30) {
        const rect = event.target.getBoundingClientRect();
        const x = event.pageX - rect.left - window.scrollX;
        const y = event.pageY - rect.top - window.scrollY;
        this.markers.push({ x, y });
      }
    },
    resetMarker() {
      this.markerVisible = false;
    },
    toggleSettings() {
      this.showSettings = !this.showSettings;
    },
    redirectToChangeItem() {
      this.$router.push('/pickingup');
    },
    redirectToChangeItemdrink() {
      this.$router.push('/swithtodrink');
    },
  },
};
</script>

<style scoped>
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
.responsive-image {
  width: 45vw;
  height: 68vh;
  display: block;
}
.box1 {
  position: absolute;
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
  height: 24vh;
  width: 45vw;
}
.food-image {
  width: 15vw;
  height: 23vh;
  border-radius: 8px;
  object-fit: cover;
  margin-right: 15px;
}
.info-content {
  display: flex;
  flex-direction: column;
}
.food-name {
  font-family: Verdana;
  font-size: 24px;
  font-weight: 700;
  line-height: 29.17px;
  text-align: left;
}
.food-detail,
.food-timer {
  font-family: Verdana;
  font-size: 18px;
  font-weight: 400;
  line-height: 1px;
  text-align: left;
}
.ingredient-item.active {
  border: 4px solid #fc6423;
  box-shadow: 0 0 15px rgba(252, 100, 35, 0.9);
  background-color: #ffffff;
}
.ingredient-list {
  display: flex;
  gap: 10px;
  width: 45vw;
  justify-content: space-between;
  align-items: baseline;
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
  width: 14vw;
  text-align: center;
  border-radius: 15px;
  box-shadow: 0px 4px 12px rgba(0, 0, 0, 0.1);
  padding: 5px;
  background-color: white;
}
.ingredient-image {
  width: 11.1vw;
  border-radius: 15px;
  object-fit: cover;
  margin-top: 14px;
  margin-left: 15px;
  margin-right: 15px;
}
.ingredient-name {
  font-family: Verdana;
  font-size: 18px;
  font-weight: 700;
  line-height: 0px;
  text-align: center;
}
.right_text {
  font-family: Verdana;
  font-size: 18px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.175px;
  text-align: left;
}
.left-first-title {
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 25px;
  letter-spacing: 0.175px;
  text-align: left;
}
.usertext {
  align-items: baseline;
  display: flex;
  justify-content: center;
  flex-flow: column;
  margin-left: 5px;
}
.username {
  font-family: Verdana;
  font-size: 20px;
  font-weight: 400;
  line-height: 18px;
  letter-spacing: 0.175px;
  text-align: left;
}
.userslog {
  font-family: Verdana;
  font-size: 16px;
  font-weight: 400;
  line-height: 18px;
  letter-spacing: 0.175px;
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
  background-color: #ffe699;
  border-radius: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  width: 20vw;
  height: 25vh;
  gap: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-flow: column;
  .icon {
    margin-right: 8px;
  }
  .button-drink {
    height: 10vh;
    width: 6vw;
  }
  .button-setting {
    height: 7vh;
    width: 4vw;
    margin: 10px;
  }
  border: none;
  font-family: Verdana;
  font-size: 20px;
  font-weight: 400;
  line-height: 24px;
  letter-spacing: 0.225px;
  text-align: center;
}
.button-drink {
  height: 10vh;
  width: 10vw;
}
.show-button-container {
  background-color: #ffe699;
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
.box {
  height: 18vh;
  width: 14vw;
  display: flex;
  flex-flow: column;
  align-items: center;
  justify-content: space-between;
  margin: 2px;
  padding: 10px;
  gap: 0;
  border-radius: 9px;
  opacity: 0;
  background: #e0e0e0;
  .metaballs {
    width: 92px;
    height: 88px;
    gap: 0;
    opacity: 0;
  }
}
.option {
  display: flex;
  justify-content: space-between;
  flex-flow: column;
  padding: 0;
}
.optionbox {
  width: 50vw;
  height: 12vh;
  gap: 0;
  margin: 3px;
  border-radius: 20px;
  opacity: 0;
  border: 1px solid #000000;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 5px;
}
.otheroption {
  display: flex;
  justify-content: space-between;
  flex-flow: column;
  padding: 0;
}
.otheroptionbox {
  width: 50vw;
  height: 12vh;
  gap: 0;
  border-radius: 20px;
  opacity: 0;
  background: #d9d9d9;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 5px;
  margin: 0;
}
.title {
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
.box-top {
  height: 23vh;
  width: 45vw;
  display: flex;
  justify-content: left;
  background-color: #f2f2f2;
  border: none;
  padding: 15px;
  gap: 0;
  border-radius: 24px;
  border-color: #333333;
  opacity: 0;
  .metaballs1 {
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
.food {
  width: 45vw;
  height: 68vh;
}
.content {
  display: flex;
  justify-content: center;
  flex-flow: column;
}
.content-body {
  display: flex;
  justify-content: space-evenly;
  margin-top: 23px;
  .left {
    display: flex;
    justify-content: flex-start;
    align-items: baseline;
    flex-flow: column;
  }
  .right {
    display: flex;
    justify-content: flex-start;
    align-items: baseline;
    flex-flow: column;
  }
}
.threeboxes {
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
    gap: 0;
    opacity: 0;
  }
  .right {
    display: flex;
    justify-content: center;
    align-items: center;
    .settings-button-text {
      font-family: Verdana;
      font-size: 18px;
      font-weight: 400;
      line-height: 24px;
      letter-spacing: 0.175px;
      text-align: left;
    }
    .finish-button-text {
      font-family: Verdana;
      font-size: 18px;
      font-weight: 400;
      line-height: 24px;
      letter-spacing: 0.175px;
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
    padding: 15px;
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
  z-index: 10;
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
  letter-spacing: 0.175px;
  text-align: left;
}
.modal-subheader {
  margin-bottom: 20px;
  color: #ffffff;
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 30px;
  letter-spacing: 0.175px;
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
  margin-top: 20px;
  flex-flow: column;
}
.confirm-button-container {
  display: flex;
  justify-content: center;
  margin-top: 20px;
}
.confirm-button,
.reset-button,
.back-button {
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
  letter-spacing: 0.225px;
  text-align: center;
}
.horizontal-layout {
  display: flex;
  justify-content: space-between;
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
}
.instruction {
  margin-bottom: 20px;
  max-width: 100%;
  word-wrap: break-word;
  font-family: Verdana;
  font-size: 20px;
  font-weight: 700;
  line-height: 30px;
  letter-spacing: 0.175px;
  text-align: left;
}
.marker {
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
}
.image-container clickimg {
  width: 80%;
  height: 100%;
}
</style>

