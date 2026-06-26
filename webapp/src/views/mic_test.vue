<template>
  <div class="page">
    <div class="tb">
      <div class="av">⚙</div>
      <div>
        <div class="tb-n">Mic Test</div>
        <div class="tb-s">Calibrate the switch-button mic — not part of the caregiver flow</div>
      </div>
    </div>

    <div class="bd">
      <div class="mic-wrap">
        <p class="mic-hint">
          Calibrates the physical button adapter used by the real app. Keep the
          browser or system speech mic on the built-in microphone; save the button
          adapter here so the app can open it by device id.
        </p>

        <div class="mic-row">
          <span>Button adapter</span>
          <select v-model="selectedDeviceId" @change="saveSelectedDevice">
            <option value="">Browser default microphone</option>
            <option v-for="device in audioInputs" :key="device.deviceId" :value="device.deviceId">
              {{ device.label || 'Microphone (permission needed for name)' }}
            </option>
          </select>
        </div>
        <div class="mic-row"><span>Saved button adapter</span><b>{{ savedDeviceLabel }}</b></div>

        <div class="mic-enable-row">
          <button class="btn md ghost" style="flex:1" @click="refreshAudioDevices">Refresh devices</button>
          <button class="btn md amber" style="flex:1" @click="enableMic">Enable button mic</button>
          <button class="btn md ghost" style="flex:1" @click="releaseMic">Release mic</button>
          <span class="mic-status">{{ status }}</span>
        </div>

        <div class="mic-row"><span>Active input device</span><b>{{ device }}</b></div>
        <div class="mic-row">
          <span>Live peak (0–1)</span>
          <b>{{ peak.toFixed(3) }} &nbsp;·&nbsp; max {{ maxSeen.toFixed(3) }}</b>
        </div>

        <div class="mic-meter"><div class="mic-meter-fill" :style="{ width: meterWidth + '%' }"></div></div>

        <div class="threshold-row">
          <span class="threshold-lbl">Threshold</span>
          <input type="range" min="0" max="0.3" step="0.001" v-model.number="threshold" />
          <b>{{ threshold.toFixed(3) }}</b>
        </div>

        <div class="mic-row">
          <span>Press
            <span class="press-state" :class="{ hit: pressActive }">{{ pressActive ? 'PRESS!' : '— idle —' }}</span>
          </span>
          <b>{{ count }} presses</b>
        </div>

        <div class="mic-enable-row">
          <button class="btn sm ghost" style="flex:1" @click="maxSeen = 0">Reset max</button>
          <button class="btn sm ghost" style="flex:1" @click="count = 0">Reset count</button>
          <button class="btn sm ghost" style="flex:1" @click="logs = []">Clear log</button>
        </div>

        <div class="mic-log">
          <div v-for="(line, i) in logs" :key="i" :class="line.type">{{ line.msg }}</div>
        </div>

        <p class="mic-hint">
          Tip: tap near the iPad's built-in mic vs. press the button to see which the meter
          responds to — that tells you whether the adapter is the active input. Then set the
          threshold so idle stays quiet but every press fires, and note that value.
        </p>
      </div>
    </div>
  </div>
</template>

<script>
import { PressDetector } from '@/utils/pressClassifier'

export default {
  name: 'MicTest',
  data () {
    return {
      status: '— not started —',
      device: '—',
      audioInputs: [],
      selectedDeviceId: this.loadStoredValue('takeoverMicDeviceId'),
      savedDeviceLabel: this.loadStoredValue('takeoverMicDeviceLabel') || '— not set —',
      peak: 0,
      maxSeen: 0,
      threshold: 0.1,
      count: 0,
      pressActive: false,
      logs: [],
      _analyser: null,
      _data: null,
      _prevAbove: false,
      _press: null,
      _raf: null,
      _stream: null,
      _audioCtx: null
    }
  },
  computed: {
    meterWidth () { return Math.min(100, this.peak * 100 / 0.3) }
  },
  beforeUnmount () {
    this.releaseMic(false)
  },
  mounted () {
    this.refreshAudioDevices()
  },
  methods: {
    loadStoredValue (key) {
      try {
        return window.localStorage.getItem(key) || ''
      } catch (e) {
        return ''
      }
    },
    async refreshAudioDevices () {
      if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
        this.status = 'device list unavailable'
        return
      }

      try {
        const devices = await navigator.mediaDevices.enumerateDevices()
        this.audioInputs = devices.filter((device) => device.kind === 'audioinput')
        if (this.selectedDeviceId && !this.audioInputs.some((device) => device.deviceId === this.selectedDeviceId)) {
          this.selectedDeviceId = ''
        }
        this.addLog('peak', 'found ' + this.audioInputs.length + ' audio input device(s)')
      } catch (e) {
        this.status = 'device refresh error: ' + e.name
      }
    },
    selectedDeviceLabel () {
      const selected = this.audioInputs.find((device) => device.deviceId === this.selectedDeviceId)
      return selected && selected.label ? selected.label : (this.selectedDeviceId ? 'Selected microphone' : 'Browser default microphone')
    },
    saveSelectedDevice () {
      const label = this.selectedDeviceLabel()
      this.savedDeviceLabel = label
      try {
        window.localStorage.setItem('takeoverMicDeviceId', this.selectedDeviceId || '')
        window.localStorage.setItem('takeoverMicDeviceLabel', label)
      } catch (e) {
        this.addLog('peak', 'could not save selected mic: ' + e.message)
      }
      this.addLog('press', 'saved button adapter: ' + label)
    },
    audioConstraints () {
      const audio = {
        echoCancellation: false,
        noiseSuppression: false,
        autoGainControl: false
      }
      if (this.selectedDeviceId) audio.deviceId = { exact: this.selectedDeviceId }
      return { audio }
    },
    async enableMic () {
      try {
        await this.releaseMic(false)
        this.saveSelectedDevice()

        const stream = await navigator.mediaDevices.getUserMedia(this.audioConstraints())
        this._stream = stream
        this.status = 'mic granted'

        try {
          const track = stream.getAudioTracks()[0]
          let label = track && track.label
          if (!label) {
            const devs = await navigator.mediaDevices.enumerateDevices()
            const inp = devs.find(d => d.kind === 'audioinput')
            label = inp && inp.label
          }
          this.device = label || 'unknown (label hidden until permission)'
        } catch (e) { this.device = 'enumerate failed' }

        const ctx = new (window.AudioContext || window.webkitAudioContext)()
        this._audioCtx = ctx
        await ctx.resume()
        const src = ctx.createMediaStreamSource(stream)
        this._analyser = ctx.createAnalyser()
        this._analyser.fftSize = 2048
        this._data = new Float32Array(this._analyser.fftSize)
        src.connect(this._analyser)
        this._press = new PressDetector({
          onPress: () => this.onPress()
        })
        this.loop()
        await this.refreshAudioDevices()
      } catch (e) {
        this.status = 'ERROR: ' + e.name + ' — ' + e.message +
          '  (NotAllowed/undefined usually means you are NOT on HTTPS)'
      }
    },
    async releaseMic (logRelease = true) {
      if (this._raf) {
        cancelAnimationFrame(this._raf)
        this._raf = null
      }
      if (this._press) {
        this._press.reset()
        this._press = null
      }
      if (this._stream) {
        this._stream.getTracks().forEach((track) => track.stop())
        this._stream = null
      }
      if (this._audioCtx) {
        try { await this._audioCtx.close() } catch (e) { /* already closed */ }
        this._audioCtx = null
      }
      this._analyser = null
      this._data = null
      this._prevAbove = false
      this.peak = 0
      if (logRelease) {
        this.status = 'mic released'
        this.addLog('peak', 'mic released')
      }
    },
    addLog (type, msg) {
      this.logs.unshift({ type, msg })
      if (this.logs.length > 80) this.logs.pop()
    },
    loop () {
      if (!this._analyser || !this._data) return
      this._analyser.getFloatTimeDomainData(this._data)
      let p = 0
      for (let i = 0; i < this._data.length; i++) {
        const a = Math.abs(this._data[i])
        if (a > p) p = a
      }
      this.peak = p
      if (p > this.maxSeen) this.maxSeen = p

      if (p > 0.01) this.addLog('peak', 'peak: ' + p.toFixed(4) + ' | threshold: ' + this.threshold.toFixed(4))

      const above = p > this.threshold
      if (above && !this._prevAbove) this._press.edge(Date.now())
      this._prevAbove = above
      this._raf = requestAnimationFrame(this.loop)
    },
    onPress () {
      this.count++
      this.pressActive = true
      this.addLog('press', 'PRESS #' + this.count)
      setTimeout(() => { this.pressActive = false }, 200)
    }
  }
}
</script>

<style scoped>
.mic-wrap {
  max-width: 760px;
  width: 100%;
  margin: 0 auto;
  flex: 1;
  min-height: 0;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 1.6vh;
}
.mic-hint {
  font-size: 2.3vh;
  color: var(--tm);
  line-height: 1.5;
  text-align: center;
}
.mic-enable-row {
  display: flex;
  align-items: center;
  gap: 1.5vw;
}
.mic-status {
  font-size: 2.4vh;
  color: var(--tm);
}
.mic-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 1vw;
  font-size: 2.4vh;
  color: var(--tm);
  padding: 1.2vh 0;
  border-bottom: 1px solid var(--bd);
}
.mic-row b { color: var(--t); }
.mic-meter {
  height: 2.6vh;
  background: var(--s1);
  border: 1px solid var(--s3);
  border-radius: 8px;
  overflow: hidden;
}
.mic-meter-fill { height: 100%; background: var(--a2); }
.threshold-row {
  display: flex;
  align-items: center;
  gap: 1.5vw;
  font-size: 2.4vh;
  color: var(--tm);
}
.threshold-lbl { flex-shrink: 0; }
.threshold-row input[type=range] {
  flex: 1;
  accent-color: var(--a);
  height: 2.6vh;
}
.threshold-row b { color: var(--t); flex-shrink: 0; min-width: 4ch; text-align: right; }
.press-state { font-weight: 700; color: var(--tm); margin-left: 6px; }
.press-state.hit { color: var(--a); }
.mic-log {
  height: 20vh;
  flex-shrink: 0;
  overflow-y: auto;
  background: var(--g);
  border: 1px solid var(--bd);
  border-radius: 10px;
  padding: 1.5vh;
  font-family: monospace;
  font-size: 1.75vh;
  color: var(--tm);
}
.mic-log .press { color: var(--a2); font-weight: 700; }
.mic-log .peak { color: var(--tm); }
</style>
