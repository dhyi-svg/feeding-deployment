<template>
  <div class="mic-test">
    <h2>iPad button mic test</h2>
    <p class="hint">
      Confirms (1) the mic works over HTTPS on this iPad, (2) your button's
      audio adapter is the active input, and (3) the detection threshold to use
      in the real takeover component.
    </p>

    <div class="row">
      <button @click="enableMic">Enable mic</button>
      <span class="status">{{ status }}</span>
    </div>

    <div class="row">Active input device: <b>{{ device }}</b></div>
    <div class="row">
      Live peak (0–1): <b>{{ peak.toFixed(3) }}</b>
      &nbsp;&nbsp; max seen: <b>{{ maxSeen.toFixed(3) }}</b>
    </div>
    <div class="meter-wrap"><div class="meter" :style="{ width: meterWidth + '%' }"></div></div>

    <div class="row">
      Threshold:
      <input type="range" min="0" max="0.3" step="0.001" v-model.number="threshold" />
      <b>{{ threshold.toFixed(3) }}</b>
    </div>

    <div class="row">
      Detector:
      <span class="press" :class="{ hit: pressActive }">{{ pressActive ? 'PRESS!' : '— idle —' }}</span>
      &nbsp;&nbsp; presses detected: <b>{{ count }}</b>
    </div>

    <div class="row">
      <button @click="maxSeen = 0">reset max</button>
      &nbsp;<button @click="logs = []">clear log</button>
    </div>

    <div class="log-panel">
      <div v-for="(line, i) in logs" :key="i" :class="line.type">{{ line.msg }}</div>
    </div>

    <p class="hint">
      Tip: tap near the iPad's built-in mic vs. press the button to see which the
      meter responds to — that tells you whether the adapter is the active input.
      Then set the threshold so idle stays quiet but every press fires, and note
      that value.
    </p>
  </div>
</template>

<script>

export default {
  name: 'MicTest',
  data () {
    return {
      status: '— not started —',
      device: '—',
      peak: 0,
      maxSeen: 0,
      threshold: 0.02,
      count: 0,
      pressActive: false,
      logs: [],
      _analyser: null,
      _data: null,
      _prevAbove: false,
      _lastHit: 0,
      _raf: null
    }
  },
  computed: {
    meterWidth () { return Math.min(100, this.peak * 100 / 0.3) }
  },
  beforeUnmount () {
    if (this._raf) cancelAnimationFrame(this._raf)
  },
  methods: {
    async enableMic () {
      try {
        
        const stream = await navigator.mediaDevices.getUserMedia({
          audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false }
        })
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
        await ctx.resume() 
        const src = ctx.createMediaStreamSource(stream)
        this._analyser = ctx.createAnalyser()
        this._analyser.fftSize = 2048
        this._data = new Float32Array(this._analyser.fftSize)
        src.connect(this._analyser)
        this.loop()
      } catch (e) {
        this.status = 'ERROR: ' + e.name + ' — ' + e.message +
          '  (NotAllowed/undefined usually means you are NOT on HTTPS)'
      }
    },
    addLog (type, msg) {
      this.logs.unshift({ type, msg })
      if (this.logs.length > 80) this.logs.pop()
    },
    loop () {
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
      const now = Date.now()
      if (above && !this._prevAbove && (now - this._lastHit) > 1500) { 
        this._lastHit = now
        this.count++
        this.pressActive = true
        this.addLog('press', 'PRESS #' + this.count + ' — peak: ' + p.toFixed(4) + ' | threshold: ' + this.threshold.toFixed(4))
        setTimeout(() => { this.pressActive = false }, 500)
      }
      this._prevAbove = above
      this._raf = requestAnimationFrame(this.loop)
    }
  }
}
</script>

<style scoped>
.mic-test { font-family: -apple-system, system-ui, sans-serif; margin: 1.2rem; line-height: 1.4; background: var(--g); color: var(--t); min-height: 100vh; padding: 1.2rem; }
.hint { color: var(--tm); max-width: 40rem; }
.row { margin: .9rem 0; }
.status { margin-left: .5rem; }
.meter-wrap { width: 100%; max-width: 40rem; height: 44px; background: var(--s1); border-radius: 8px; overflow: hidden; }
.meter { height: 100%; background: var(--a2); }
.press { font-size: 1.6rem; font-weight: 700; color: var(--tm); }
.press.hit { color: #e88; }
button { font-size: 1.1rem; padding: .6rem 1rem; border-radius: 8px; border: 2px solid var(--s3); background: var(--s2); color: var(--t); cursor: pointer; }
.log-panel { margin-top: .8rem; max-width: 40rem; height: 220px; overflow-y: auto; background: var(--g); border: 1px solid var(--bd); color: var(--tm); font-family: monospace; font-size: .8rem; padding: .5rem; border-radius: 8px; }
.log-panel .press { color: #e88; font-weight: 700; }
.log-panel .peak  { color: var(--tm); }
</style>
