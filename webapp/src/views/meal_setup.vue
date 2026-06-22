<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
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
      
      <div class="left">
        <img ref="videoImage" class="food" :src="videoFrame">

      </div>
      <div class="right">
        <div class="right-other-option-text">
          <span class="right-little-text">Please list the food items on the plate</span>

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
      ros: null,
      isRecognizing1: false, 
      isRecognizing2: false, 
      recognition1: null, 
      recognition2: null, 
      username: USER,
      selectedOption: 0,
      foodItems: [],
      videoFrame: null, 
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
    this.initSubscriber()
    this.initPublisher()
    this.publishMessageOnLoad()
    this.initVideoSubscriber()
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
    cleartheinput1() {
      this.transcript = ''; 
    },

    cleartheinput2() {
      this.transcriptDes = ''; 
    },

    startSpeechRecognition1() {
      if (this.isRecognizing1) {
        return; 
      }
      if (!this.recognition1) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition1 = new SpeechRecognition();
        this.recognition1.lang = 'en-US';
        this.recognition1.continuous = false;

        this.recognition1.onresult = (event) => {
          this.transcript += event.results[0][0].transcript; 
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
        return; 
      }
      if (!this.recognition2) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition2 = new SpeechRecognition();
        this.recognition2.lang = 'en-US';
        this.recognition2.continuous = false;

        this.recognition2.onresult = (event) => {
          this.transcriptDes += event.results[0][0].transcript; 
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
      
      if (!this.recognition) {
        
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        this.recognition = new SpeechRecognition();
        this.recognition.lang = 'en-US';
        this.recognition.continuous = false; 

        this.recognition.onresult = (event) => {
          
          this.transcript += event.results[0][0].transcript;
          this.isRecognizing = false; 
        };

        this.recognition.onerror = (event) => {
          this.isRecognizing = false; 
          this.focusTextarea(); 
        };

        this.recognition.onend = () => {
          this.isRecognizing = false; 
          this.focusTextarea(); 
        };
      }

      if (this.isRecognizing) {
        this.recognition.stop();
      }

      this.isRecognizing = true;
      this.recognition.start();

      this.$nextTick(() => {
        this.focusTextarea(); 
      });
    },
    focusTextarea() {
      
      this.$refs.textarea.focus();
    },
    publishMessageforNopre(){
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'voice',
          status: this.transcript
        })
      })
      this.publisher.publish(message)
      this.transcript = '';
    },
    initVideoSubscriber() {

      const listener = new ROSLIB.Topic({
        ros: this.ros,
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
            
            const maxBoxWidth = window.innerWidth * 0.1;
            const maxBoxHeight = window.innerHeight * 0.1;

            const widthRatio = maxBoxWidth / item.w;
            const heightRatio = maxBoxHeight / item.h;
            const scaleRatio = Math.min(widthRatio, heightRatio);

            const scaledWidth = item.w * scaleRatio;
            const scaledHeight = item.h * scaleRatio;

            canvas.width = scaledWidth;
            canvas.height = scaledHeight;

            ctx.drawImage(
              img,
              item.x, item.y, item.w, item.h,  
              0, 0, scaledWidth, scaledHeight  
            );

            const subImage = canvas.toDataURL('image/jpeg');
            this.subImages.push(subImage);

            if (index < this.foodItems.length) {
              this.foodItems[index].image = subImage; 
            }
          });
        };
      } else {
      }
    },

    handleRosMessage(message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          if (typeof route === 'string') {
            this.$router.push(route); 
          } else if (typeof route === 'object') {
            this.$router.push(route); 
          }
        }

        if (parsedMessage && Array.isArray(parsedMessage.data)) {
          
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

          this.foodItems = [...updatedFoodItems];
        }
      } catch (error) {
      }
    },

    confirmSelection() {
      
      if (this.selectedOption !== null && this.selectedOption !== 0) {
        const selectedText = this.optionTexts[this.selectedOption - 1];

        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'meal_setup',
            status: selectedText
          })
        });
        this.publisher.publish(message);
      } else {
      }

      if (this.transcript !== '' && this.transcriptDes !== '') {
        const voiceMessage = new ROSLIB.Message({
          data: JSON.stringify({
            state: this.transcript,
            status: this.transcriptDes 
          })
        });
        this.publisher.publish(voiceMessage);
      }

      this.transcript = '';
      this.transcriptDes = '';
      this.$router.push('/robot_executing');
    },
    publishMessageOnLoad() {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'meal_setup',
          status: 'ready_for_initial_data'
        })
      })

      if (this.publisher) {
        this.publisher.publish(message)
      } else {
      }
    },

    selectOption(option) {
      this.selectedOption = this.selectedOption === option ? 0 : option;
      this.transcript = '';
      this.$router.push('/task_selection')
    },
    redirectToChangeItem() {
      this.$router.push('/bite_selection')
    },
    redirectToChangeItemF() {
      this.$router.push('/notify_caregiver')
    },
    initSubscriber() {

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: this.subscribeTopic,
        messageType: 'std_msgs/String'
      })

      this.listener.subscribe((msg) => {
        try {
          const data = JSON.parse(msg.data);

          if (data && data.n_food_types) {
            
            try {
              const parsedMessage = data;

              if (parsedMessage && Array.isArray(parsedMessage.data)) {
                
                this.foodItems = parsedMessage.data.map(item => {
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

                if (parsedMessage.current_bite) {
                  const currentBiteName = Object.keys(parsedMessage.current_bite)[0];
                  const currentBiteCoordinates = parsedMessage.current_bite[currentBiteName][0]; 

                  this.foodItems.push({
                    name: currentBiteName,
                    image: this.getImageForItem(currentBiteName),
                    x: currentBiteCoordinates[0],
                    y: currentBiteCoordinates[1],
                    w: currentBiteCoordinates[2],
                    h: currentBiteCoordinates[3],
                    isCurrentBite: true 
                  });
                }
              }
            } catch (error) {
            }
          }

          if (data && data.n_ordering) {
            const updatedOptionTexts = data.data.map((option) => `${option}`);
            this.optionTexts = [...updatedOptionTexts];
          }
        } catch (error) {
        }
      });
    },
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: this.publishTopic,
        messageType: 'std_msgs/String'
      })
    },
    getImageForItem(name) {
      if (!name) {
        return ''; 
      }
    }
  }
}
</script>

<style scoped>
.voice-start-button.recognizing {
  background-color: #6e7e8e; 
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
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 5px;
  flex-flow: column;
}
.custom-input-box1 {
  width: 90%; 
  height: 100%; 
  font-size: 1.7vw;
  border: 1px solid #ccc;
  border-radius: 10px;
  padding: 10px;
  resize: none; 
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
  justify-content: space-between;
  flex-flow: column;
  padding: 0px;
  height: 32vh;
}
.voiceinputbox{
  font-family: Verdana;
  font-size: 18px;
  width: 70%; 
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
  max-height: 36vh; 
  overflow-y: auto;  
  padding-right: 10px; 
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
  background-color: #6c7984; 
  color: white; 
}
.food{
  width: 43vw;
  height: 58vh;
}
.content{
  display: flex;
  justify-content: center;
  flex-flow: column;
}
.content-body {
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
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
    letter-spacing: 0.17499999701976776px;
    text-align: left;
    word-wrap: break-word; 
    white-space: normal;
    line-height: 1.2em;
    overflow-wrap: break-word;
  }
  .right-first-title{
    font-family: Verdana;
    font-size: clamp(12px, 2.5vh, 20px);
    margin-top: auto;
    font-weight: 700;
    max-width: 12vw;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
    word-wrap: break-word; 
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
    display: flex;
    align-items: center;
    justify-content: space-around;
    padding: 10px;
  }
}
.threeboxes{
  width: 43vw;
  height: 18vh;
  display: flex;
  align-items: start;
  justify-content: normal;
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
  min-height:15vh;
  min-width:12vw;
  max-width:15vw;
  margin: 1vh;
  display: flex;
  flex-flow: column;
  align-items: center;
  justify-content: space-between;
  padding: 1px;
  gap: 0px;
  border-radius: 9px 9px 9px 9px;
  opacity: 0px;
  background: #F2F2F2;
  .metaballs{
    display: flex;
    flex-flow: column;
    align-items: center;
    margin: 0.5vh;
    object-fit: cover;
    gap: 0px;
    opacity: 0px;
    border-radius: 20px;
    padding: 0vh;
    align-self: center;
  }
}
.option{
  display: flex;
  justify-content: space-between;
  flex-flow: column;
  padding: 0px;

}
.optionbox{
  width: 50vw;
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
