<template>
  <div id="app">
    <div v-if="showTakeoverEnable" class="enable-takeover-wrap">
      <button class="enable-takeover-btn" @click="enableTakeoverMic">
        🎙 Enable takeover button
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
import { PressClassifier } from '@/utils/pressClassifier'

export default {
  name: 'app',
  data () {
    return {
      takeoverPublisher: null,
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
      _micActive: false,
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
    '$route.path' (path) {
      if (!this.takeoverMicEnabled) return
      const freeMic = this.micFreeRoutes.includes(path)
      if (freeMic && this._micActive) {
        this._stopMic()
      } else if (!freeMic && !this._micActive) {
        this._startMic().catch(() => {
          // Re-acquire failed (some iOS versions need a fresh user gesture);
          // fall back to the enable pill so the user can re-tap.
          this.takeoverMicEnabled = false
        })
      }
    }
  },
  mounted () {
    this.connectRos()
    // Single press -> bite-transfer confirm (latency-tolerant). Double press is
    // intentionally inert: the physical user button must no longer trigger
    // teleoperation (use the on-screen Robot Arm/Base Control buttons instead).
    // We still classify the double so a fast two-click doesn't fire a spurious
    // single-press confirm.
    this._press = new PressClassifier({
      onSingle: () => this.onSinglePress(),
      onDouble: () => this.onDoublePress()
    })
  },
  beforeUnmount () {
    this._destroyed = true
    if (this._reconnectTimer) { clearTimeout(this._reconnectTimer); this._reconnectTimer = null }
    this._stopMic()
    if (this._press) this._press.reset()
  },
  methods: {
    connectRos () {
      // App.vue is the persistent root: unlike the views (which build a fresh
      // connection on every mount), this socket would otherwise live and die
      // once. If it drops with no reconnect, takeoverPublisher and the
      // /skill_plan listener silently go dead -> the global Take Over buttons and
      // base-control visibility stop working. Reconnect on close so they heal.
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
      this.takeoverPublisher = new ROSLIB.Topic({
        ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
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
    },
    controlArm () {
      if (this.takeoverPublisher) {
        this.takeoverPublisher.publish(new ROSLIB.Message({
          data: JSON.stringify({ state: 'teleop', status: 'takeover' })
        }))
      }
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
        await this._startMic()
        this.takeoverMicEnabled = true
      } catch (e) {
        alert('Could not start the takeover button mic: ' + e.name + ' — ' + e.message +
          '\n(The webapp must be served over HTTPS for the iPad to allow the mic.)')
      }
    },
    // Acquire the mic and start the button-detection loop. Split out from
    // enableTakeoverMic so the route watcher can release/re-acquire the mic
    // (to free it for speech-to-text on voice pages) without re-prompting.
    async _startMic () {
      if (this._micActive) return
      const stream = await navigator.mediaDevices.getUserMedia({
        audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false }
      })
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
    onSinglePress () {
      // Decoupled from the page: any view that wants the physical button (e.g.
      // bite_confirm_transfer) listens for this event and runs its own action.
      window.dispatchEvent(new CustomEvent('takeover-single-press'))
    },
    onDoublePress () {
      // Disabled: the physical button no longer triggers teleoperation. Left as
      // a no-op (rather than deleting onTakeoverButton) so it can be re-enabled
      // by restoring `this.onTakeoverButton()` here if needed.
    },
    onTakeoverButton () {
      if (!this.showTakeOver || this.$route.path === '/idle_takeover') return
      if (this.skillCurrent < 0) {
        if (this.$route.path !== '/idle_takeover') this.$router.push('/idle_takeover')
        return
      }
      const skill = this.skillPlan[this.skillCurrent]
      if (categoryOf(skill) === 'navigation') this.controlBase()
      else this.controlArm()
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
</style>
