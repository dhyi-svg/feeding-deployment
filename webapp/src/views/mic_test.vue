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
          Confirms the mic works over HTTPS, your button's audio adapter is the active
          input, and the detection threshold to use in the real takeover component.
        </p>

        <div class="mic-enable-row">
          <button class="btn md amber" style="min-width:34%" @click="enableMic">Enable mic</button>
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
          <span>Raw click
            <span class="press-state" :class="{ hit: pressActive }">{{ pressActive ? 'CLICK!' : '— idle —' }}</span>
          </span>
          <b>{{ count }} clicks</b>
        </div>

        <div class="event-banner"
             :class="{ single: lastEvent === 'SINGLE', double: lastEvent === 'DOUBLE' }">
          {{ lastEvent ? lastEvent + ' PRESS' : 'tap once = single · tap twice quickly = double' }}
        </div>

        <div class="mic-row">
          <span>Single presses</span><b>{{ singleCount }}</b>
        </div>
        <div class="mic-row">
          <span>Double presses</span><b>{{ doubleCount }}</b>
        </div>

        <div class="threshold-row">
          <span class="threshold-lbl">Double window</span>
          <input type="range" min="200" max="1500" step="10" v-model.number="doubleWindowMs" />
          <b>{{ doubleWindowMs }}ms</b>
        </div>

        <div class="mic-enable-row">
          <button class="btn sm ghost" style="flex:1" @click="maxSeen = 0">Reset max</button>
          <button class="btn sm ghost" style="flex:1" @click="singleCount = 0; doubleCount = 0; count = 0">Reset counts</button>
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
import { PressClassifier, DOUBLE_WINDOW } from '@/utils/pressClassifier'

export default {
  name: 'MicTest',
  data () {
    return {
      status: '— not started —',
      device: '—',
      peak: 0,
      maxSeen: 0,
      threshold: 0.1,
      count: 0,
      singleCount: 0,
      doubleCount: 0,
      lastEvent: '',
      doubleWindowMs: DOUBLE_WINDOW,
      pressActive: false,
      logs: [],
      _analyser: null,
      _data: null,
      _prevAbove: false,
      _press: null,
      _eventFlash: null,
      _raf: null
    }
  },
  computed: {
    meterWidth () { return Math.min(100, this.peak * 100 / 0.3) }
  },
  beforeUnmount () {
    if (this._raf) cancelAnimationFrame(this._raf)
    if (this._press) this._press.reset()
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
        this._press = new PressClassifier({
          onSingle: () => this.onSingle(),
          onDouble: () => this.onDouble()
        })
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

      // keep the classifier window in sync with the on-screen slider
      this._press.doubleWindow = this.doubleWindowMs

      const above = p > this.threshold
      if (above && !this._prevAbove) {
        const fresh = this._press.edge(Date.now())
        if (fresh) {
          this.count++
          this.pressActive = true
          this.addLog('click', 'click #' + this.count + ' — peak: ' + p.toFixed(4))
          setTimeout(() => { this.pressActive = false }, 200)
        }
      }
      this._prevAbove = above
      this._raf = requestAnimationFrame(this.loop)
    },
    flashEvent (kind) {
      this.lastEvent = kind
      if (this._eventFlash) clearTimeout(this._eventFlash)
      this._eventFlash = setTimeout(() => { this.lastEvent = '' }, 900)
    },
    onSingle () {
      this.singleCount++
      this.flashEvent('SINGLE')
      this.addLog('single', 'SINGLE PRESS #' + this.singleCount)
    },
    onDouble () {
      this.doubleCount++
      this.flashEvent('DOUBLE')
      this.addLog('double', 'DOUBLE PRESS #' + this.doubleCount)
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
.mic-log .single { color: var(--a2); font-weight: 700; }
.mic-log .double { color: var(--a); font-weight: 700; }
.mic-log .click { color: var(--t); }
.mic-log .peak { color: var(--tm); }
.event-banner {
  text-align: center;
  font-family: Verdana, sans-serif;
  font-weight: 700;
  font-size: 2.8vh;
  padding: 1.6vh 0;
  border-radius: 10px;
  background: var(--s1);
  border: 1px solid var(--s3);
  color: var(--tm);
  transition: background .1s, color .1s, border-color .1s;
}
.event-banner.single { background: rgba(46, 196, 182, .18); border-color: var(--a2); color: var(--a2); }
.event-banner.double { background: rgba(240, 165, 0, .18); border-color: var(--a); color: var(--a); }
</style>
