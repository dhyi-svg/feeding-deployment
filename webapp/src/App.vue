<template>
  <div id="app">
    <div v-if="showTakeoverEnable" class="enable-takeover-wrap">
      <button class="enable-takeover-btn" @click="enableTakeoverMic">
        Enable voice button
      </button>
    </div>
    <div v-if="showMicSetupModal" class="mic-setup-backdrop">
      <div class="mic-setup-modal">
        <button class="mic-setup-close" @click="closeMicSetup" title="Close">×</button>
        <div class="mic-setup-title">Voice Button Channels</div>
        <div class="mic-setup-copy">
          Select the physical button adapter for button presses. Keep the voice
          channel as the built-in microphone used by Chrome or the system.
        </div>

        <div class="mic-channel-grid">
          <label class="mic-channel-field">
            <span>Button channel</span>
            <select v-model="setupButtonDeviceId" @change="saveMicSetupSelection">
              <option value="">Browser default microphone</option>
              <option v-for="device in audioInputs" :key="'button-' + device.deviceId" :value="device.deviceId">
                {{ device.label || 'Microphone (permission needed for name)' }}
              </option>
            </select>
          </label>
          <label class="mic-channel-field">
            <span>Voice channel</span>
            <select v-model="setupVoiceDeviceId" @change="saveMicSetupSelection">
              <option value="">Browser/system default microphone</option>
              <option v-for="device in audioInputs" :key="'voice-' + device.deviceId" :value="device.deviceId">
                {{ device.label || 'Microphone (permission needed for name)' }}
              </option>
            </select>
          </label>
        </div>

        <div class="mic-setup-status">{{ micSetupStatus }}</div>
        <div class="mic-setup-actions">
          <button class="mic-setup-secondary" @click="refreshAudioDevices">Refresh devices</button>
          <button class="mic-setup-primary" @click="saveMicSetupAndEnable">Save & Enable</button>
        </div>
      </div>
    </div>
    <div v-if="!audioEnabled" class="enable-audio-wrap">
      <button class="enable-audio-btn" @click="enableAudio">
        🔊 Enable voice
      </button>
    </div>
    <div v-if="showTakeOver" class="global-controls">
      <button
        v-if="showBaseControl"
        class="global-btn base"
        @click="controlBase()"
      >Robot Base Control</button>
      <button class="global-btn arm" @click="controlArm()">Robot Arm Control</button>
    </div>
    <router-view></router-view>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import { ROS_URL } from '@/config/parameterConfig'
import { categoryOf } from '@/config/skillCategories'
import { PressDetector } from '@/utils/pressClassifier'

export default {
  name: 'app',
  data () {
    return {
      takeoverMicEnabled: false,
      takeoverThreshold: 0.1,
      skillPlan: [],
      skillCurrent: -1,
      _analyser: null,
      _audioBuf: null,
      _prevAbove: false,
      _press: null,
      _raf: null,
      _ros: null,
      _destroyed: false,
      _reconnectTimer: null,
      audioEnabled: false,
      _micActive: false,
      _takeoverMicDeviceId: '',
      showMicSetupModal: false,
      audioInputs: [],
      setupButtonDeviceId: '',
      setupVoiceDeviceId: '',
      micSetupStatus: 'Choose the button and voice channels.',
      // Routes where the button-detection mic is released so the iPad's mic is
      // free for speech-to-text (the transparency / adaptability Q&A pages and
      // the gesture_setup form). Add others here (e.g. '/meal_setup') as needed.
      micFreeRoutes: ['/transparency', '/adaptability', '/gesture_setup']
    }
  },
  computed: {
    showTakeOver () {
      const p = this.$route.path
      const excluded = [
        '/manipulation_teleop', '/manipulation_done', '/navigation_teleop', '/mictest',
        '/transparency', '/adaptability', '/personalization',
        '/gesture_menu', '/gesture_setup', '/gesture_test',
        '/gesture_record_positive', '/gesture_record_negative'
      ]
      return !excluded.includes(p)
    },
    onTaskSelection () {
      return this.$route.path === '/task_selection'
    },
    showBaseControl () {
      // Offer base teleop on the task menu, or whenever a *navigation* skill is the
      // one currently executing (same skill-plan signal the physical takeover
      // button uses to route base vs arm). Previously driven by a separate
      // base_control enabled/disabled message; now it tracks the skill plan.
      if (this.onTaskSelection) return true
      const skill = this.skillCurrent >= 0 ? this.skillPlan[this.skillCurrent] : null
      return !!skill && categoryOf(skill) === 'navigation'
    },
    showTakeoverEnable () {
      // The takeover-mic enable is only meaningful during autonomous operation.
      // Hide it on pages where you've already taken over / it's irrelevant (and
      // where their longer headers would collide with the centered pill).
      if (this.takeoverMicEnabled) return false
      const excluded = ['/manipulation_teleop', '/manipulation_done', '/navigation_teleop', '/idle_takeover', '/mictest']
      const p = this.$route.path
      // Don't offer to enable the button mic on voice pages — it would hold the
      // mic the speech recognizer needs.
      return !excluded.includes(p) && !this.micFreeRoutes.includes(p)
    },
  },
  watch: {
    // When navigating onto a voice page (e.g. transparency), release the
    // button-detection mic so the iPad's speech recognizer gets exclusive
    // access; re-acquire it when leaving. Only relevant once the user has
    // enabled the takeover mic.
    '$route.path': {
      immediate: true,
      handler (path) {
        this._syncMicForRoute(path)
      }
    }
  },
  mounted () {
    this.connectRos()
    // Every press -> bite-transfer confirm. There is no single/double
    // distinction: the physical user button no longer triggers teleoperation
    // (use the on-screen Robot Arm/Base Control buttons instead).
    this._press = new PressDetector({
      onPress: () => this.onPress()
    })
    window.addEventListener('release-takeover-mic', this.releaseTakeoverMic)
  },
  beforeUnmount () {
    this._destroyed = true
    window.removeEventListener('release-takeover-mic', this.releaseTakeoverMic)
    if (this._reconnectTimer) { clearTimeout(this._reconnectTimer); this._reconnectTimer = null }
    this._stopMic()
    if (this._press) this._press.reset()
  },
  methods: {
    connectRos () {
      // App.vue is the persistent root: unlike the views (which build a fresh
      // connection on every mount), this socket would otherwise live and die
      // once. If it drops with no reconnect, the /skill_plan listener and spoken
      // prompts silently go dead. Reconnect on close so they heal.
      const ros = new ROSLIB.Ros({ url: ROS_URL })
      this._ros = ros
      ros.on('error', () => { /* a 'close' follows; reconnect is handled there */ })
      ros.on('close', () => {
        if (this._destroyed) return
        if (this._reconnectTimer) return
        this._reconnectTimer = setTimeout(() => {
          this._reconnectTimer = null
          this.connectRos()
        }, 1000)
      })
      this.skillPlanListener = new ROSLIB.Topic({
        ros,
        name: '/skill_plan',
        messageType: 'std_msgs/String'
      })
      // /skill_plan is latched on the backend, so re-subscribing after a
      // reconnect immediately re-delivers the current plan (no state lost).
      this.skillPlanListener.subscribe((msg) => {
        try {
          const parsed = JSON.parse(msg.data)
          this.skillPlan = parsed.plan || []
          this.skillCurrent = (typeof parsed.current === 'number') ? parsed.current : -1
        } catch (e) { /* ignore non-JSON */ }
      })
      // The robot publishes spoken prompts (bite-transfer cues, etc.) on /speak.
      // We voice them on this device (the iPad) via the Web Speech API instead of
      // a speaker wired to the compute machine. Re-created on reconnect, like above.
      this.speakListener = new ROSLIB.Topic({
        ros,
        name: '/speak',
        messageType: 'std_msgs/String'
      })
      this.speakListener.subscribe((msg) => this.speakText(msg.data))
    },
    enableAudio () {
      // iPad Safari blocks speechSynthesis until speak() is first called from an
      // explicit user gesture with a real utterance (an empty/silent one doesn't
      // unlock it). This button is that gesture; "Voice enabled" both unlocks
      // speech for the rest of the session and audibly confirms output works.
      this.audioEnabled = true
      if (!('speechSynthesis' in window)) {
        alert('This browser has no speech synthesis support.')
        return
      }
      try {
        window.speechSynthesis.cancel()
        const u = new SpeechSynthesisUtterance('Voice enabled')
        u.lang = 'en-US'
        window.speechSynthesis.speak(u)
      } catch (e) { /* ignore */ }
    },
    speakText (text) {
      if (!text || !('speechSynthesis' in window)) return
      // Drop any queued/stale utterance so prompts don't pile up if they arrive
      // faster than they're spoken.
      window.speechSynthesis.cancel()
      const utterance = new SpeechSynthesisUtterance(text)
      utterance.lang = 'en-US'
      utterance.rate = 0.95
      window.speechSynthesis.speak(utterance)
    },
    controlArm () {
      this.$router.push('/manipulation_teleop')
    },
    controlBase () {
      // Pass the running skill so navigation_teleop shows Resume/Done only when a
      // navigation skill is actually executing; otherwise it enters aux mode
      // (manual driving + Return).
      const skill = this.skillCurrent >= 0 ? this.skillPlan[this.skillCurrent] : null
      const query = (skill && categoryOf(skill) === 'navigation') ? { hla: skill } : {}
      this.$router.push({ path: '/navigation_teleop', query })
    },

    async enableTakeoverMic () {
      try {
        await this.requestMicPermission()
        await this.refreshAudioDevices()
        this.setupButtonDeviceId = this._getStoredValue('takeoverMicDeviceId')
        this.setupVoiceDeviceId = this._getStoredValue('voiceMicDeviceId')
        this.showMicSetupModal = true
        this.micSetupStatus = 'Permission granted. Select channels, then save.'
      } catch (e) {
        alert('Could not start the voice button setup: ' + e.name + ' — ' + e.message +
          '\n(The webapp must be served over HTTPS for the iPad to allow the mic.)')
      }
    },
    async requestMicPermission () {
      const stream = await navigator.mediaDevices.getUserMedia({
        audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false }
      })
      stream.getTracks().forEach((track) => track.stop())
    },
    async refreshAudioDevices () {
      if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
        this.micSetupStatus = 'This browser cannot list audio devices.'
        return
      }
      const devices = await navigator.mediaDevices.enumerateDevices()
      this.audioInputs = devices.filter((device) => device.kind === 'audioinput')
      if (this.setupButtonDeviceId && !this.audioInputs.some((device) => device.deviceId === this.setupButtonDeviceId)) {
        this.setupButtonDeviceId = ''
      }
      if (this.setupVoiceDeviceId && !this.audioInputs.some((device) => device.deviceId === this.setupVoiceDeviceId)) {
        this.setupVoiceDeviceId = ''
      }
      this.micSetupStatus = `Found ${this.audioInputs.length} audio input device(s).`
    },
    closeMicSetup () {
      this.showMicSetupModal = false
    },
    setupDeviceLabel (deviceId, fallback) {
      const device = this.audioInputs.find((item) => item.deviceId === deviceId)
      return device && device.label ? device.label : fallback
    },
    saveMicSetupSelection () {
      this._setStoredValue('takeoverMicDeviceId', this.setupButtonDeviceId)
      this._setStoredValue('takeoverMicDeviceLabel', this.setupDeviceLabel(this.setupButtonDeviceId, 'Browser default microphone'))
      this._setStoredValue('voiceMicDeviceId', this.setupVoiceDeviceId)
      this._setStoredValue('voiceMicDeviceLabel', this.setupDeviceLabel(this.setupVoiceDeviceId, 'Browser/system default microphone'))
      this.micSetupStatus = 'Channel selection saved.'
    },
    async saveMicSetupAndEnable () {
      try {
        this.saveMicSetupSelection()
        await this._stopMic()
        if (!this.micFreeRoutes.includes(this.$route.path)) {
          await this._startMic()
        }
        this.takeoverMicEnabled = true
        this.showMicSetupModal = false
      } catch (e) {
        this.micSetupStatus = 'Could not enable button channel: ' + e.name + ' — ' + e.message
      }
    },
    async _syncMicForRoute (path = this.$route.path) {
      if (!this.takeoverMicEnabled) return
      const freeMic = this.micFreeRoutes.includes(path)
      if (freeMic) {
        await this._stopMic()
      } else if (!this._micActive) {
        try {
          await this._startMic()
        } catch (e) {
          // Re-acquire failed (some iOS versions need a fresh user gesture);
          // fall back to the enable pill so the user can re-tap.
          this.takeoverMicEnabled = false
        }
      }
    },
    async releaseTakeoverMic (event) {
      const done = event && event.detail && event.detail.done
      try {
        await this._stopMic()
        if (done) done()
      } catch (e) {
        if (done) done(e)
      }
    },
    // Acquire the mic and start the button-detection loop. Split out from
    // enableTakeoverMic so the route watcher can release/re-acquire the mic
    // (to free it for speech-to-text on voice pages) without re-prompting.
    async _startMic () {
      if (this._micActive) return
      const stream = await navigator.mediaDevices.getUserMedia(this._takeoverMicConstraints())
      const ctx = new (window.AudioContext || window.webkitAudioContext)()
      await ctx.resume()
      const src = ctx.createMediaStreamSource(stream)
      this._analyser = ctx.createAnalyser()
      this._analyser.fftSize = 2048
      this._audioBuf = new Float32Array(this._analyser.fftSize)
      src.connect(this._analyser)
      // Kept as plain (non-reactive) instance props on purpose: wrapping native
      // MediaStream/AudioContext in Vue's reactive proxy can break their methods.
      this._micStream = stream
      this._audioCtx = ctx
      this._micActive = true
      this._prevAbove = false
      if (this._press) this._press.reset()
      this.audioLoop()
    },
    // Stop the loop and fully release the mic (stop the MediaStream tracks and
    // close the AudioContext) so the iPad frees it for the speech recognizer.
    async _stopMic () {
      if (this._raf) { cancelAnimationFrame(this._raf); this._raf = null }
      if (this._micStream) {
        this._micStream.getTracks().forEach((t) => t.stop())
        this._micStream = null
      }
      if (this._audioCtx) {
        try { await this._audioCtx.close() } catch (e) { /* already closed */ }
        this._audioCtx = null
      }
      this._analyser = null
      this._micActive = false
      if (this._press) this._press.reset()
    },
    audioLoop () {
      if (!this._micActive || !this._analyser) return
      this._analyser.getFloatTimeDomainData(this._audioBuf)
      let peak = 0
      for (let i = 0; i < this._audioBuf.length; i++) {
        const a = Math.abs(this._audioBuf[i])
        if (a > peak) peak = a
      }
      const above = peak > this.takeoverThreshold
      if (above && !this._prevAbove) this._press.edge(Date.now())
      this._prevAbove = above
      this._raf = requestAnimationFrame(this.audioLoop)
    },
    onPress () {
      // Decoupled from the page: any view that wants the physical button (e.g.
      // bite_confirm_transfer) listens for this event and runs its own action.
      window.dispatchEvent(new CustomEvent('takeover-press'))
    },
    _getStoredValue (key) {
      try {
        return window.localStorage.getItem(key) || ''
      } catch (e) {
        return ''
      }
    },
    _setStoredValue (key, value) {
      try {
        window.localStorage.setItem(key, value || '')
      } catch (e) { /* storage unavailable */ }
    },
    _getTakeoverMicDeviceId () {
      return this._getStoredValue('takeoverMicDeviceId')
    },
    _takeoverMicConstraints () {
      const audio = {
        echoCancellation: false,
        noiseSuppression: false,
        autoGainControl: false
      }
      const deviceId = this._getTakeoverMicDeviceId()
      this._takeoverMicDeviceId = deviceId
      if (deviceId) audio.deviceId = { exact: deviceId }
      return { audio }
    }
  }
}
</script>

<style>
html, body {
  margin: 0;
  padding: 0;
  background: #0D1B2A;
}

#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 0px !important;
}

nav {
  padding: 30px;
}

nav a {
  font-weight: bold;
  color: #2c3e50;
}

nav a.router-link-exact-active {
  color: #42b983;
}

.global-controls {
  position: fixed;
  top: 0;
  right: 2.5vw;
  height: 10vh;
  z-index: 1000;
  display: flex;
  align-items: center;
  gap: 16px;
}

.global-btn {
  height: 8vh;
  font-family: Verdana, sans-serif;
  font-size: 2.8vh;
  font-weight: 700;
  padding: 0 2.6vw;
  border-radius: 14px;
  color: #F5F0E8;
  cursor: pointer;
  background: #1E3347;
  border: 2px solid #243C54;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.3);
  white-space: nowrap;
}
.global-btn.arm { color: #F0A500; border-color: rgba(240, 165, 0, .4); }
.global-btn.arm:active { background: #243C54; }
.global-btn.base { color: #2EC4B6; border-color: rgba(46, 196, 182, .4); }
.global-btn.base:active { background: #243C54; }
.enable-takeover-wrap {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  height: 10vh;
  z-index: 1100;
  display: flex;
  align-items: center;
  justify-content: center;
  pointer-events: none;
}
.enable-takeover-btn {
  pointer-events: auto;
  font-family: Verdana, sans-serif;
  font-size: 3vh;
  font-weight: 800;
  padding: 2vh 4vw;
  border-radius: 999px;
  background: #2EC4B6;
  color: #0D1B2A;
  border: 3px solid #2EC4B6;
  box-shadow: 0 4px 14px rgba(46, 196, 182, .45);
  cursor: pointer;
}
.mic-setup-backdrop {
  position: fixed;
  inset: 0;
  z-index: 3000;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 4vh 4vw;
  background: rgba(5, 10, 18, .62);
}
.mic-setup-modal {
  position: relative;
  width: min(760px, 92vw);
  background: #172033;
  color: #F5F0E8;
  border: 2px solid #31405f;
  border-radius: 8px;
  padding: 3vh 2.5vw 2.5vh;
  box-shadow: 0 18px 48px rgba(0, 0, 0, .45);
  text-align: left;
}
.mic-setup-close {
  position: absolute;
  top: 1vh;
  right: 1vw;
  width: 44px;
  height: 44px;
  min-height: 44px;
  border-radius: 50%;
  border: 1px solid #31405f;
  background: #202b42;
  color: #F5F0E8;
  font-size: 3vh;
  line-height: 1;
  cursor: pointer;
}
.mic-setup-title {
  font-family: Georgia, serif;
  font-size: 3.6vh;
  color: #F5F0E8;
  margin-right: 52px;
}
.mic-setup-copy {
  margin-top: 1vh;
  color: #aab6ca;
  font-size: 2.2vh;
  line-height: 1.45;
}
.mic-channel-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1.5vw;
  margin-top: 2.2vh;
}
.mic-channel-field {
  display: flex;
  flex-direction: column;
  gap: .8vh;
  color: #aab6ca;
  font-size: 2vh;
  font-weight: 700;
}
.mic-channel-field select {
  width: 100%;
  min-height: 6vh;
  border-radius: 8px;
  border: 1px solid #31405f;
  background: #0b1220;
  color: #F5F0E8;
  font-family: Verdana, sans-serif;
  font-size: 2vh;
  padding: 0 1vw;
}
.mic-setup-status {
  margin-top: 1.6vh;
  min-height: 3vh;
  color: #2EC4B6;
  font-size: 2vh;
  font-weight: 700;
}
.mic-setup-actions {
  display: flex;
  gap: 1vw;
  justify-content: flex-end;
  margin-top: 1.8vh;
}
.mic-setup-actions button {
  min-height: 6vh;
  border-radius: 8px;
  font-family: Verdana, sans-serif;
  font-size: 2.1vh;
  font-weight: 800;
  padding: 0 2vw;
  cursor: pointer;
}
.mic-setup-primary {
  background: #F0A500;
  border: 2px solid #F0A500;
  color: #0D1B2A;
}
.mic-setup-secondary {
  background: #202b42;
  border: 2px solid #31405f;
  color: #F5F0E8;
}
.enable-audio-wrap {
  position: fixed;
  left: 0;
  right: 0;
  bottom: 2vh;
  z-index: 1100;
  display: flex;
  justify-content: center;
  pointer-events: none;
}
.enable-audio-btn {
  pointer-events: auto;
  font-family: Verdana, sans-serif;
  font-size: 2.6vh;
  font-weight: 800;
  padding: 1.6vh 4vw;
  border-radius: 999px;
  background: #F0A500;
  color: #0D1B2A;
  border: 3px solid #F0A500;
  box-shadow: 0 4px 14px rgba(240, 165, 0, .45);
  cursor: pointer;
}
@media (max-width: 720px) {
  .mic-channel-grid {
    grid-template-columns: 1fr;
  }
  .mic-setup-actions {
    flex-direction: column;
  }
}
</style>
