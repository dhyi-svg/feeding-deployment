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
      _raf: null
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
      return !excluded.includes(this.$route.path)
    },
  },
  mounted () {
    const ros = new ROSLIB.Ros({ url: ROS_URL })
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
    this.skillPlanListener.subscribe((msg) => {
      try {
        const parsed = JSON.parse(msg.data)
        this.skillPlan = parsed.plan || []
        this.skillCurrent = (typeof parsed.current === 'number') ? parsed.current : -1
      } catch (e) { /* ignore non-JSON */ }
    })
    // Single press -> bite-transfer confirm (latency-tolerant); double press ->
    // stop autonomous & take over (fires fast, on the 2nd click).
    this._press = new PressClassifier({
      onSingle: () => this.onSinglePress(),
      onDouble: () => this.onDoublePress()
    })
  },
  beforeUnmount () {
    if (this._raf) cancelAnimationFrame(this._raf)
    if (this._press) this._press.reset()
  },
  methods: {
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
        this.takeoverMicEnabled = true
        this.audioLoop()
      } catch (e) {
        alert('Could not start the takeover button mic: ' + e.name + ' — ' + e.message +
          '\n(The webapp must be served over HTTPS for the iPad to allow the mic.)')
      }
    },
    audioLoop () {
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
      this.onTakeoverButton()
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
  gap: 12px;
}

.global-btn {
  height: 6vh;
  font-family: Verdana, sans-serif;
  font-size: 1.9vh;
  font-weight: 700;
  padding: 0 1.6vw;
  border-radius: 10px;
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
  font-size: 1.8vh;
  font-weight: 700;
  padding: 1vh 1.6vw;
  border-radius: 999px;
  background: #1E3347;
  color: #2EC4B6;
  border: 2px solid rgba(46, 196, 182, .4);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.3);
  cursor: pointer;
}
</style>
