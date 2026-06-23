<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Adjust color detection.</div>
      </div>
    </div>

    <div class="bd det-bd">
      <div class="cc-row">
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

        <div class="cc-side" :style="sideHeight ? { height: sideHeight + 'px' } : {}">
          <div class="cc-swatch" :style="{ backgroundColor: selectedColorCss }"></div>
          <div class="slider-wrapper">
            <input
              type="range"
              class="cc-slider"
              v-model.number="colorRange"
              :style="{ width: sliderHeight + 'px' }"
              min="0" max="1" step="0.01"
            />
          </div>
          <div class="cc-val">{{ colorRange.toFixed(2) }}</div>
        </div>
      </div>

      <div v-if="detectionStatus" :class="['status-badge', detectionStatus]">
        {{ detectionStatusText }}
      </div>

      <div class="det-actions">
        <button class="btn md ghost" :disabled="!showingResult || waitingForResult" @click="resetImage">Reset</button>
        <button class="btn md teal" :disabled="!selectedColor || waitingForResult || showingResult" @click="rerunDetection">Rerun</button>
        <button class="btn md amber" :disabled="!showingResult || waitingForResult" @click="confirmColor">Confirm</button>
      </div>
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
      this.publisher = new ROSLIB.Topic({ ros: this.ros, name: '/webapp_to_robot', messageType: 'std_msgs/String' })
    },
    initSubscriber () {

      this.imageListener = new ROSLIB.Topic({
        ros: this.ros, name: '/camera/image/compressed', messageType: 'sensor_msgs/CompressedImage'
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
        ros: this.ros, name: '/robot_to_webapp', messageType: 'std_msgs/String'
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
.cc-row {
  display: flex;
  flex-direction: row;
  gap: 1vw;
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
  border: 2px solid var(--s3);
  border-radius: 8px;
  display: block;
}

.canvas-overlay-text {
  position: absolute;
  font-family: Verdana;
  font-size: 1.8vh;
  color: var(--tm);
  background: var(--s1);
  padding: 1vh 1.5vw;
  border-radius: 8px;
  pointer-events: none;
}

.cc-side {
  width: 7vw;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1vh;
  flex-shrink: 0;
  align-self: center;
}

.cc-swatch {
  width: 6vw;
  height: 6vw;
  max-width: 68px;
  max-height: 68px;
  border-radius: 10px;
  border: 2px solid var(--s3);
  flex-shrink: 0;
  box-shadow: 0 2px 6px rgba(0, 0, 0, .4);
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

.cc-slider {
  -webkit-appearance: none;
  appearance: none;
  transform: rotate(-90deg);
  flex-shrink: 0;
  height: 68px;
  background: transparent;
  cursor: pointer;
  outline: none;
}

.cc-slider::-webkit-slider-runnable-track {
  background: var(--s3);
  border-radius: 5px;
  height: 10px;
}
.cc-slider::-moz-range-track {
  background: var(--s3);
  border-radius: 5px;
  height: 10px;
}

.cc-slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  appearance: none;
  width: 54px;
  height: 54px;
  background: var(--a);
  border-radius: 10px;
  cursor: grab;
  box-shadow: 0 3px 8px rgba(0, 0, 0, .5);
  border: 3px solid var(--g);
  margin-top: -22px;
}
.cc-slider::-moz-range-thumb {
  width: 54px;
  height: 54px;
  background: var(--a);
  border-radius: 10px;
  cursor: grab;
  box-shadow: 0 3px 8px rgba(0, 0, 0, .5);
  border: 3px solid var(--g);
}
.cc-slider::-webkit-slider-thumb:active { background: #c87800; cursor: grabbing; }
.cc-slider::-moz-range-thumb:active     { background: #c87800; cursor: grabbing; }

.cc-val {
  font-family: Verdana;
  font-size: 2.2vh;
  font-weight: 700;
  color: var(--t);
  flex-shrink: 0;
}

.status-badge {
  padding: .8vh 1.5vw;
  border-radius: 8px;
  font-family: Verdana;
  font-size: 1.6vh;
  font-weight: 700;
  width: 100%;
  text-align: center;
  box-sizing: border-box;
  flex-shrink: 0;
}
.status-badge.success { background: rgba(46, 196, 182, .15); color: var(--a2); }
.status-badge.failed  { background: rgba(220, 60, 60, .12); color: #e88; }
</style>
