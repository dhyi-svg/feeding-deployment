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
          <div class="cc-controls">
            <div class="slider-wrapper">
              <input
                type="range"
                class="cc-slider"
                v-model.number="colorRange"
                :style="{ width: sliderHeight + 'px' }"
                min="0" max="1" step="0.01"
              />
            </div>
            <div
              class="cc-gradient"
              :style="{ height: sliderHeight + 'px', background: gradientCss }"
            >
              <div v-if="selectedColor" class="cc-picked-box" :style="pickedBoxStyle"></div>
            </div>
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
      // Fraction of the gradient bar height occupied by the flat picked-color
      // band (centered). Shared by gradientCss and the boxed overlay so they
      // line up. Constant — never mutated.
      PICKED_BAND_FRAC: 0.2,
      selectedColor: null,
      pickPoint: null,
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
    // Lower/upper detection-band colors, reproducing the backend HSV tolerance
    // math in attachment_perception.detect_attachment_color: h_tol = range*90,
    // s_tol = v_tol = range*255, on the OpenCV HSV scale (H 0-179, S/V 0-255).
    limitColorsCss () {
      if (!this.selectedColor) return { lower: '#cccccc', upper: '#cccccc' }
      const { r, g, b } = this.selectedColor
      const { h, s, v } = this.rgbToHsvCV(r, g, b)
      const cr = this.colorRange
      const hTol = cr * 90
      const sTol = cr * 255
      const vTol = cr * 255
      const wrapHue = (x) => ((x % 180) + 180) % 180
      const clip = (x) => Math.max(0, Math.min(255, x))
      const toCss = (hh, ss, vv) => {
        const c = this.hsvCVToRgb(wrapHue(hh), clip(ss), clip(vv))
        return `rgb(${c.r},${c.g},${c.b})`
      }
      return {
        lower: toCss(h - hTol, s - sTol, v - vTol),
        upper: toCss(h + hTol, s + sTol, v + vTol)
      }
    },
    lowerLimitCss () { return this.limitColorsCss.lower },
    upperLimitCss () { return this.limitColorsCss.upper },
    // Continuous gradient (bottom=lower -> flat picked band -> top=upper). The
    // duplicated picked stops flatten the center; PICKED_BAND_FRAC sets its size
    // and is shared with the boxed overlay so the two stay aligned.
    gradientCss () {
      const half = (this.PICKED_BAND_FRAC * 100) / 2
      const lo = (50 - half).toFixed(1)
      const hi = (50 + half).toFixed(1)
      return `linear-gradient(to top, ${this.lowerLimitCss} 0%, ${this.selectedColorCss} ${lo}%, ${this.selectedColorCss} ${hi}%, ${this.upperLimitCss} 100%)`
    },
    pickedBoxStyle () {
      const half = (this.PICKED_BAND_FRAC * 100) / 2
      return { top: (50 - half) + '%', height: (this.PICKED_BAND_FRAC * 100) + '%' }
    },
    detectionStatusText () {
      if (this.detectionStatus === 'success') return '✓ Detection successful'
      if (this.detectionStatus === 'failed') return '✗ No detection — adjust color or range and retry'
      return ''
    },
    sliderHeight () {
      return Math.max(60, (this.sideHeight || 250) - 40)
    }
  },
  watch: {
    // Repaint the in-range highlight live as the slider moves the range.
    colorRange () { this.updatePreview() }
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
    // RGB (0-255) -> HSV on the OpenCV scale: H in [0,179], S/V in [0,255].
    rgbToHsvCV (r, g, b) {
      const rn = r / 255, gn = g / 255, bn = b / 255
      const max = Math.max(rn, gn, bn), min = Math.min(rn, gn, bn)
      const d = max - min
      let h = 0
      if (d !== 0) {
        if (max === rn)      h = ((gn - bn) / d) % 6
        else if (max === gn) h = (bn - rn) / d + 2
        else                 h = (rn - gn) / d + 4
        h *= 60
        if (h < 0) h += 360
      }
      const s = max === 0 ? 0 : d / max
      const v = max
      return { h: Math.round(h / 2), s: Math.round(s * 255), v: Math.round(v * 255) }
    },
    // Inverse of rgbToHsvCV: OpenCV-scale HSV -> RGB (0-255).
    hsvCVToRgb (h, s, v) {
      const hh = (h * 2) / 60
      const sn = s / 255
      const vn = v / 255
      const c = vn * sn
      const x = c * (1 - Math.abs((hh % 2) - 1))
      const m = vn - c
      let rn = 0, gn = 0, bn = 0
      if      (hh < 1) { rn = c; gn = x }
      else if (hh < 2) { rn = x; gn = c }
      else if (hh < 3) { gn = c; bn = x }
      else if (hh < 4) { gn = x; bn = c }
      else if (hh < 5) { rn = x; bn = c }
      else             { rn = c; bn = x }
      return {
        r: Math.round((rn + m) * 255),
        g: Math.round((gn + m) * 255),
        b: Math.round((bn + m) * 255)
      }
    },
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
        this.pickPoint       = null
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

    // Redraw the clean pick image + the in-range red highlight + crosshairs.
    // Called on pick and on every slider change so the highlighted region tracks
    // the current permissible color range. No-op once a result is being shown.
    updatePreview () {
      if (!this.imageReady || this.showingResult || this.waitingForResult) return
      const canvas = this.$refs.canvas
      if (!canvas || !this.pickImageElement) return
      const ctx = canvas.getContext('2d')
      ctx.drawImage(this.pickImageElement, 0, 0)
      if (this.selectedColor) this.paintInRange(ctx, canvas)
      if (this.pickPoint) this.drawCrosshairs(ctx, canvas, this.pickPoint.x, this.pickPoint.y)
    },

    // Tint every pixel within the current permissible color range red (alpha).
    // Mirrors attachment_perception.detect_attachment_color exactly: OpenCV HSV,
    // h_tol = range*90, s_tol = v_tol = range*255, with circular-hue handling.
    paintInRange (ctx, canvas) {
      const { r, g, b } = this.selectedColor
      const { h: H, s: S, v: V } = this.rgbToHsvCV(r, g, b)
      const cr = this.colorRange
      const hTol = Math.round(cr * 90)
      const sTol = Math.round(cr * 255), vTol = Math.round(cr * 255)
      const sLo = Math.max(0, S - sTol), sHi = Math.min(255, S + sTol)
      const vLo = Math.max(0, V - vTol), vHi = Math.min(255, V + vTol)
      const fullHue = hTol >= 90
      const loH = ((H - hTol) % 180 + 180) % 180
      const hiH = ((H + hTol) % 180 + 180) % 180
      const A = 0.5
      const im = ctx.getImageData(0, 0, canvas.width, canvas.height)
      const d = im.data
      for (let i = 0; i < d.length; i += 4) {
        const R = d[i], G = d[i + 1], B = d[i + 2]
        const mx = R > G ? (R > B ? R : B) : (G > B ? G : B)
        if (mx < vLo || mx > vHi) continue
        const mn = R < G ? (R < B ? R : B) : (G < B ? G : B)
        const df = mx - mn
        const Sv = mx === 0 ? 0 : Math.round((df * 255) / mx)
        if (Sv < sLo || Sv > sHi) continue
        let hh = 0
        if (df !== 0) {
          if (mx === R) hh = ((G - B) / df) % 6
          else if (mx === G) hh = (B - R) / df + 2
          else hh = (R - G) / df + 4
          hh *= 60
          if (hh < 0) hh += 360
        }
        hh = Math.round(hh / 2)
        const hueOk = fullHue || (loH <= hiH ? (hh >= loH && hh <= hiH) : (hh >= loH || hh <= hiH))
        if (!hueOk) continue
        d[i]     = R * (1 - A) + 255 * A
        d[i + 1] = G * (1 - A)
        d[i + 2] = B * (1 - A)
      }
      ctx.putImageData(im, 0, 0)
    },

    drawCrosshairs (ctx, canvas, x, y) {
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
      // Sample from the clean image, not the live canvas: a previous selection
      // may have drawn crosshairs, and clicking on those would otherwise read
      // the crosshair color (red) instead of the underlying pixel.
      const ctx = canvas.getContext('2d')
      ctx.drawImage(this.pickImageElement, 0, 0)
      const pixel = ctx.getImageData(x, y, 1, 1).data
      this.selectedColor = { r: pixel[0], g: pixel[1], b: pixel[2] }
      this.pickPoint = { x, y }
      this.updatePreview()
    },

    resetImage () {
      if (!this.pickImageElement || !this.$refs.canvas) return
      const canvas = this.$refs.canvas
      canvas.width  = this.pickImageElement.naturalWidth
      canvas.height = this.pickImageElement.naturalHeight
      canvas.getContext('2d').drawImage(this.pickImageElement, 0, 0)
      this.selectedColor   = null
      this.pickPoint       = null
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
  font-size: 2.1vh;
  color: var(--tm);
  background: var(--s1);
  padding: 1vh 1.5vw;
  border-radius: 8px;
  pointer-events: none;
}

.cc-side {
  width: 9vw;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1vh;
  flex-shrink: 0;
  align-self: center;
}

.cc-controls {
  flex: 1;
  min-height: 0;
  width: 100%;
  display: flex;
  flex-direction: row;
  align-items: center;
  justify-content: center;
  gap: 0.5vw;
  overflow: hidden;
}

.cc-gradient {
  width: 34px;
  flex-shrink: 0;
  position: relative;
  border-radius: 8px;
  border: 2px solid var(--s3);
  box-shadow: 0 2px 6px rgba(0, 0, 0, .4);
}

.cc-picked-box {
  position: absolute;
  left: 0;
  right: 0;
  box-sizing: border-box;
  border: 2px solid #fff;
  outline: 1px solid rgba(0, 0, 0, .65);
  border-radius: 3px;
  pointer-events: none;
}

.slider-wrapper {
  height: 100%;
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
  font-size: 2.6vh;
  font-weight: 700;
  color: var(--t);
  flex-shrink: 0;
}

.status-badge {
  padding: .8vh 1.5vw;
  border-radius: 8px;
  font-family: Verdana;
  font-size: 1.9vh;
  font-weight: 700;
  width: 100%;
  text-align: center;
  box-sizing: border-box;
  flex-shrink: 0;
}
.status-badge.success { background: rgba(46, 196, 182, .15); color: var(--a2); }
.status-badge.failed  { background: rgba(220, 60, 60, .12); color: #e88; }
</style>
