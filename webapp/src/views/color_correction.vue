<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class="usertext">
        <div class="username">{{ username }}</div>
        <div class="userslog">Adjust color detection.</div>
      </div>
    </div>
  </div>

  <div class="content">
    
    <div class="image-and-side">
      <div class="canvas-container">
        <canvas
          ref="canvas"
          class="main-canvas"
          :style="{ cursor: showingResult || waitingForResult ? 'default' : 'crosshair' }"
          @click="onCanvasClick"
        />
        <div v-if="!imageReady" class="canvas-overlay-text">Waiting for camera image…</div>
        <div v-else-if="waitingForResult" class="canvas-overlay-text">Running detection…</div>
      </div>

      <div class="side-panel" :style="sideHeight ? { height: sideHeight + 'px' } : {}">
        <div class="color-swatch" :style="{ backgroundColor: selectedColorCss }"></div>
        <div class="slider-wrapper">
          <input
            type="range"
            class="vertical-slider"
            v-model.number="colorRange"
            :style="{ width: sliderHeight + 'px' }"
            min="0" max="1" step="0.01"
          />
        </div>
        <div class="range-value">{{ colorRange.toFixed(2) }}</div>
      </div>
    </div>

    <div v-if="detectionStatus" :class="['status-badge', detectionStatus]">
      {{ detectionStatusText }}
    </div>

    <div class="buttons">
      <button class="reset-button"   :disabled="!showingResult || waitingForResult" @click="resetImage">Reset</button>
      <button class="rerun-button"   :disabled="!selectedColor || waitingForResult || showingResult" @click="rerunDetection">Rerun</button>
      <button class="confirm-button" :disabled="!showingResult || waitingForResult" @click="confirmColor">Confirm</button>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'

export default {
  name: 'ColorCorrection',
  data () {
    return {
      ros: null,
      username: USER,
      colorRange: 0.1,
      selectedColor: null,
      imageReady: false,
      showingResult: false,
      waitingForResult: false,
      receivingResultNext: false,
      detectionStatus: null,
      pickImageElement: null,
      sideHeight: null,
      publisher: null,
      listener: null,
      imageListener: null,
    }
  },
  computed: {
    selectedColorCss () {
      if (!this.selectedColor) return '#cccccc'
      const { r, g, b } = this.selectedColor
      return `rgb(${r},${g},${b})`
    },
    detectionStatusText () {
      if (this.detectionStatus === 'success') return '✓ Detection successful'
      if (this.detectionStatus === 'failed') return '✗ No detection — adjust color or range and retry'
      return ''
    },
    sliderHeight () {
      return Math.max(60, (this.sideHeight || 250) - 130)
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initSubscriber()
    window.addEventListener('resize', this.updateSideHeight)
  },
  beforeUnmount () {
    if (this.listener)      { this.listener.unsubscribe();      this.listener = null }
    if (this.imageListener) { this.imageListener.unsubscribe(); this.imageListener = null }
    if (this.publisher)     { this.publisher.unadvertise();     this.publisher = null }
    window.removeEventListener('resize', this.updateSideHeight)
  },
  methods: {
    initPublisher () {
      this.publisher = new ROSLIB.Topic({ ros, name: '/webapp_to_robot', messageType: 'std_msgs/String' })
    },
    initSubscriber () {

      this.imageListener = new ROSLIB.Topic({
        ros, name: '/camera/image/compressed', messageType: 'sensor_msgs/CompressedImage'
      })
      this.imageListener.subscribe((msg) => {
        const src = 'data:image/jpeg;base64,' + msg.data
        if (this.receivingResultNext) {
          this.receivingResultNext = false
          const img = new Image()
          img.onload = () => {
            const canvas = this.$refs.canvas
            if (!canvas) return
            canvas.width  = img.naturalWidth
            canvas.height = img.naturalHeight
            canvas.getContext('2d').drawImage(img, 0, 0)
            this.showingResult    = true
            this.waitingForResult = false
            this.updateSideHeight()
          }
          img.src = src
        } else {
          this.loadPickImage(src)
        }
      })

      this.listener = new ROSLIB.Topic({
        ros, name: '/robot_to_webapp', messageType: 'std_msgs/String'
      })
      this.listener.subscribe((msg) => {
        try {
          const data = JSON.parse(msg.data)
          if (data.state === 'color_correction') {
            if (data.status === 'info' && data.initial_color_range !== undefined) {
              this.colorRange = data.initial_color_range
            } else if (data.status === 'detection_success') {
              this.receivingResultNext = true
              this.detectionStatus    = 'success'
            } else if (data.status === 'detection_failed') {
              this.receivingResultNext = false
              this.waitingForResult    = false
              this.detectionStatus     = 'failed'
            }
          }
          const route = routeMap[data.state]?.[data.status]
          if (route) this.$router.push(route)
        } catch (e) {
        }
      })
    },

    loadPickImage (src) {
      const img = new Image()
      img.onload = () => {
        this.pickImageElement = img
        const canvas = this.$refs.canvas
        if (!canvas) return
        canvas.width  = img.naturalWidth
        canvas.height = img.naturalHeight
        canvas.getContext('2d').drawImage(img, 0, 0)
        this.imageReady      = true
        this.showingResult   = false
        this.selectedColor   = null
        this.detectionStatus = null
        this.updateSideHeight()
      }
      img.src = src
    },

    updateSideHeight () {
      this.$nextTick(() => {
        requestAnimationFrame(() => {
          const canvas = this.$refs.canvas
          if (!canvas) return
          const h = canvas.getBoundingClientRect().height
          if (h > 0) this.sideHeight = h
        })
      })
    },

    drawPickWithCrosshairs (x, y) {
      const canvas = this.$refs.canvas
      if (!canvas || !this.pickImageElement) return
      const ctx = canvas.getContext('2d')
      ctx.drawImage(this.pickImageElement, 0, 0)
      const lw = Math.max(2, Math.round(canvas.width / 300))
      const r  = Math.max(20, Math.round(canvas.width / 40))
      ctx.save()
      ctx.strokeStyle = '#FF3300'
      ctx.lineWidth   = lw * 2
      ctx.beginPath()
      ctx.arc(x, y, r, 0, 2 * Math.PI)
      ctx.stroke()
      ctx.lineWidth = lw
      ctx.beginPath()
      ctx.moveTo(x - r, y); ctx.lineTo(x + r, y)
      ctx.moveTo(x, y - r); ctx.lineTo(x, y + r)
      ctx.stroke()
      ctx.restore()
    },

    onCanvasClick (event) {
      if (!this.imageReady || this.showingResult || this.waitingForResult) return
      const canvas = this.$refs.canvas
      const rect   = canvas.getBoundingClientRect()
      const scaleX = canvas.width  / rect.width
      const scaleY = canvas.height / rect.height
      const x = Math.max(0, Math.min(canvas.width  - 1, Math.floor((event.clientX - rect.left) * scaleX)))
      const y = Math.max(0, Math.min(canvas.height - 1, Math.floor((event.clientY - rect.top)  * scaleY)))
      const pixel = canvas.getContext('2d').getImageData(x, y, 1, 1).data
      this.selectedColor = { r: pixel[0], g: pixel[1], b: pixel[2] }
      this.drawPickWithCrosshairs(x, y)
    },

    resetImage () {
      if (!this.pickImageElement || !this.$refs.canvas) return
      const canvas = this.$refs.canvas
      canvas.width  = this.pickImageElement.naturalWidth
      canvas.height = this.pickImageElement.naturalHeight
      canvas.getContext('2d').drawImage(this.pickImageElement, 0, 0)
      this.selectedColor   = null
      this.showingResult   = false
      this.detectionStatus = null
    },

    publish (obj) {
      this.publisher.publish(new ROSLIB.Message({ data: JSON.stringify(obj) }))
    },

    rerunDetection () {
      if (!this.selectedColor || this.waitingForResult) return
      this.waitingForResult = true
      this.detectionStatus  = null
      this.publish({
        state: 'color_correction', status: 'rerun',
        r: this.selectedColor.r, g: this.selectedColor.g, b: this.selectedColor.b,
        color_range: this.colorRange
      })
    },

    confirmColor () {
      if (!this.selectedColor || this.waitingForResult) return
      this.waitingForResult = true
      this.detectionStatus  = null
      this.publish({
        state: 'color_correction', status: 'confirm',
        r: this.selectedColor.r, g: this.selectedColor.g, b: this.selectedColor.b,
        color_range: this.colorRange
      })
    },

  }
}
</script>

<style scoped>
.top {
  height: 9vh;
  background: #eee;
  display: flex;
  align-items: center;
  padding: 5px 15px;
  margin-bottom: 5px;
}
.left { display: flex; align-items: center; }
.usertext { display: flex; flex-flow: column; align-items: baseline; margin-left: 5px; }
.username { font-family: Verdana; font-size: 20px; font-weight: 400; text-align: left; line-height: 18px; }
.userslog { font-family: Verdana; font-size: 16px; font-weight: 400; text-align: left; line-height: 18px; }

.content {
  display: flex;
  flex-direction: column;
  align-items: center;
  height: 85vh;
  padding: 8px 16px;
  box-sizing: border-box;
  gap: 8px;
}

.image-and-side {
  display: flex;
  flex-direction: row;
  gap: 10px;
  flex: 1;
  min-height: 0;
  width: 100%;
  align-items: stretch;
}

.canvas-container {
  position: relative;
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  min-width: 0;
  min-height: 0;
}

.main-canvas {
  max-width: 100%;
  max-height: 100%;
  border: 2px solid #aaa;
  border-radius: 6px;
  display: block;
}

.canvas-overlay-text {
  position: absolute;
  font-family: Verdana;
  font-size: 18px;
  color: #666;
  background: rgba(255,255,255,0.8);
  padding: 8px 16px;
  border-radius: 6px;
  pointer-events: none;
}

.side-panel {
  width: 72px;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 6px;
  flex-shrink: 0;
  align-self: center;
}

.color-swatch {
  width: 68px;
  height: 68px;
  border-radius: 10px;
  border: 2px solid #777;
  flex-shrink: 0;
  box-shadow: 0 2px 6px rgba(0,0,0,0.2);
}

.slider-wrapper {
  flex: 1;
  min-height: 0;
  width: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  overflow: hidden;
}

.vertical-slider {
  -webkit-appearance: none;
  appearance: none;
  transform: rotate(-90deg);
  flex-shrink: 0;
  height: 68px;
  background: transparent;
  cursor: pointer;
  outline: none;
}

.vertical-slider::-webkit-slider-runnable-track {
  background: #ccc;
  border-radius: 5px;
  height: 10px;
}
.vertical-slider::-moz-range-track {
  background: #ccc;
  border-radius: 5px;
  height: 10px;
}

.vertical-slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  appearance: none;
  width: 54px;
  height: 54px;
  background: #444;
  border-radius: 10px;
  cursor: grab;
  box-shadow: 0 3px 8px rgba(0,0,0,0.35);
  border: 3px solid #fff;
  margin-top: -22px;
}
.vertical-slider::-moz-range-thumb {
  width: 54px;
  height: 54px;
  background: #444;
  border-radius: 10px;
  cursor: grab;
  box-shadow: 0 3px 8px rgba(0,0,0,0.35);
  border: 3px solid #fff;
}
.vertical-slider::-webkit-slider-thumb:active { background: #222; cursor: grabbing; }
.vertical-slider::-moz-range-thumb:active     { background: #222; cursor: grabbing; }

.range-value {
  font-family: Verdana;
  font-size: 38px;
  font-weight: 700;
  color: #333;
  flex-shrink: 0;
}

.status-badge {
  padding: 5px 16px;
  border-radius: 8px;
  font-family: Verdana;
  font-size: 15px;
  font-weight: 700;
  width: 100%;
  text-align: center;
  box-sizing: border-box;
}
.status-badge.success { background: #d4edda; color: #155724; }
.status-badge.failed  { background: #f8d7da; color: #721c24; }

.buttons { display: flex; gap: 12px; }
.reset-button, .rerun-button, .confirm-button {
  border: none;
  color: black;
  cursor: pointer;
  border-radius: 20px;
  width: 15vw;
  height: 9vh;
  display: flex;
  align-items: center;
  justify-content: center;
  font-family: Verdana;
  font-size: 22px;
  font-weight: 400;
  text-align: center;
  padding: 8px;
}
.reset-button   { background-color: #B3D9FF; }
.rerun-button   { background-color: #FFE699; }
.confirm-button { background-color: #90EE90; }
.reset-button:disabled,
.rerun-button:disabled, .confirm-button:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}
</style>
