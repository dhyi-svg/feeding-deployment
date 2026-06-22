<template>
  <div class="navteleop">
    
    <div class="header">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class="header-text">
        <div class="header-title">Base Navigation Control</div>
        <div class="header-sub">Push to drive · release to stop · press Done when parked</div>
      </div>
      <button class="done-btn" @click="finish()">Done</button>
    </div>

    <div class="banner" :class="{ bad: !connected }">{{ bannerText }}</div>

    <div class="hla-banner" v-if="currentHla">
      Robot is currently: <b>{{ skillLabel(currentHla) }}</b>
    </div>

    <div class="joystick-area">
      <div
        class="pad"
        ref="pad"
        @pointerdown="onDown"
        @pointermove="onMove"
        @pointerup="center"
        @pointercancel="center"
        @lostpointercapture="center"
      >
        <div class="axis ax-v"></div>
        <div class="axis ax-h"></div>
        <div class="knob" ref="knob" :class="{ dragging }">&#9209;</div>
      </div>

      <div class="readout">
        <span>drive&nbsp; <b>{{ lin.toFixed(2) }}</b> m/s</span>
        <span>turn&nbsp; <b>{{ ang.toFixed(2) }}</b> rad/s</span>
      </div>
      <div class="hint">push to drive · release to stop</div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL } from '@/config/parameterConfig'
import { skillLabel } from '@/config/skillLabels'

const MAX_LIN = 0.30   
const MAX_ANG = 0.60   
const SEND_HZ = 10     
const R = 96           
const KCENTER = 96     
const DEADZONE = 0.05  

export default {
  name: 'NavigationTeleop',
  data () {
    return {
      lin: 0,
      ang: 0,
      dragging: false,
      connected: true,
      activePid: null,
      sendTimer: null,
      ros: null,
      cmdVelPub: null,
      takeoverPub: null,
      donePub: null,
      listener: null,
      skillPlanListener: null,

      currentHla: null
    }
  },
  computed: {
    bannerText () {
      return this.connected ? 'connected' : 'connection lost — base stopping'
    }
  },
  mounted () {

    if (this.$route.query.hla) {
      this.currentHla = this.$route.query.hla
    }
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.ros.on('connection', () => { this.connected = true })
    this.ros.on('close', () => this.setLinkHealthy(false))
    this.ros.on('error', () => this.setLinkHealthy(false))

    this.cmdVelPub = new ROSLIB.Topic({ ros: this.ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' })

    this.takeoverPub = new ROSLIB.Topic({ ros: this.ros, name: '/shared_autonomy/takeover', messageType: 'std_msgs/Empty' })
    this.donePub = new ROSLIB.Topic({ ros: this.ros, name: '/shared_autonomy/done', messageType: 'std_msgs/Empty' })
    
    this.listener = new ROSLIB.Topic({ ros: this.ros, name: '/robot_to_webapp', messageType: 'std_msgs/String' })
    this.listener.subscribe((msg) => this.handleRosMessage(msg))

    this.skillPlanListener = new ROSLIB.Topic({ ros: this.ros, name: '/skill_plan', messageType: 'std_msgs/String' })
    this.skillPlanListener.subscribe((msg) => this.handleSkillPlan(msg))

    this.takeoverPub.publish(new ROSLIB.Message({}))

    this.sendTimer = setInterval(() => this.sendVelocity(), 1000 / SEND_HZ)

    window.addEventListener('blur', this.center)
    document.addEventListener('visibilitychange', this.onVisibility)
  },
  beforeUnmount () {
    this.teardown()
  },
  beforeRouteLeave (to, from, next) {
    this.teardown()
    next()
  },
  methods: {
    onDown (e) {
      this.activePid = e.pointerId
      this.$refs.pad.setPointerCapture(this.activePid)
      this.dragging = true
      this.updateFromPoint(e.clientX, e.clientY)
      e.preventDefault()
    },
    onMove (e) {
      if (this.activePid === e.pointerId) {
        this.updateFromPoint(e.clientX, e.clientY)
        e.preventDefault()
      }
    },
    setKnob (dx, dy) {
      const knob = this.$refs.knob
      if (!knob) return
      knob.style.left = (KCENTER + dx) + 'px'
      knob.style.top = (KCENTER + dy) + 'px'
    },
    updateFromPoint (cx, cy) {
      const r = this.$refs.pad.getBoundingClientRect()
      let dx = cx - (r.left + r.width / 2)
      let dy = cy - (r.top + r.height / 2)
      const d = Math.hypot(dx, dy)
      if (d > R) { dx = dx / d * R; dy = dy / d * R }
      this.setKnob(dx, dy)

      let fy = -dy / R   
      let fx = -dx / R   
      if (Math.abs(fy) < DEADZONE) fy = 0
      if (Math.abs(fx) < DEADZONE) fx = 0

      this.lin = fy * MAX_LIN
      this.ang = fx * MAX_ANG
    },
    center () {
      this.activePid = null
      this.dragging = false
      this.setKnob(0, 0)
      this.lin = 0
      this.ang = 0
    },
    onVisibility () {
      if (document.hidden) this.center()
    },
    sendVelocity () {
      if (!this.cmdVelPub) return
      this.cmdVelPub.publish(new ROSLIB.Message({
        linear: { x: this.lin, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: this.ang }
      }))
    },
    setLinkHealthy (ok) {
      this.connected = ok
      if (!ok) this.center()
    },
    handleRosMessage (msg) {
      try {
        const parsed = JSON.parse(msg.data)
        if (parsed.state === 'navigation_teleop') return
        const route = routeMap[parsed.state]?.[parsed.status]
        if (route) this.$router.push(route)
      } catch (e) {
      }
    },
    handleSkillPlan (msg) {
      try {
        const parsed = JSON.parse(msg.data)
        const plan = Array.isArray(parsed.plan) ? parsed.plan : []
        const idx = parsed.current
        this.currentHla = (typeof idx === 'number' && idx >= 0 && idx < plan.length) ? plan[idx] : null
      } catch (e) {
      }
    },
    skillLabel (name) {
      return skillLabel(name)
    },
    finish () {
      
      this.center()
      this.sendVelocity()
      if (this.donePub) this.donePub.publish(new ROSLIB.Message({}))
      this.$router.push('/task_selection')
    },
    teardown () {
      this.center()
      if (this.cmdVelPub) this.sendVelocity()  
      if (this.sendTimer) { clearInterval(this.sendTimer); this.sendTimer = null }
      window.removeEventListener('blur', this.center)
      document.removeEventListener('visibilitychange', this.onVisibility)
      if (this.listener) { this.listener.unsubscribe(); this.listener = null }
      if (this.skillPlanListener) { this.skillPlanListener.unsubscribe(); this.skillPlanListener = null }
    }
  }
}
</script>

<style scoped>
.navteleop {
  max-width: 1140px;
  margin: 0 auto;
  font-family: Verdana, sans-serif;
  background: #fff;
  padding: 8px 20px 10px;
  box-sizing: border-box;
  
  height: 100vh;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.header {
  display: flex; align-items: center; gap: 12px;
  background: #eee; border-radius: 8px;
  padding: 6px 14px; margin-bottom: 8px;
}
.header .user { width: 36px; height: 36px; }
.header-title { font-size: 20px; font-weight: 700; color: #1f2937; }
.header-sub { font-size: 14px; color: #6e7e8e; }
.done-btn {
  margin-left: auto; font-family: Verdana, sans-serif; font-size: 16px;
  font-weight: 700; padding: 0 22px; border: none; border-radius: 8px;
  background: #28a745; color: #fff; cursor: pointer; height: 50px;
}

.banner {
  text-align: center; font-size: 14px; padding: 8px; border-radius: 8px;
  color: #085041; background: #e1f5ee; margin-bottom: 8px;
}
.banner.bad { color: #791f1f; background: #fcebeb; }

.hla-banner {
  text-align: center; font-size: 15px; padding: 8px; border-radius: 8px;
  color: #2c5777; background: #e3edf7; margin-bottom: 8px;
}

.joystick-area {
  display: flex; flex-direction: column; align-items: center;
  justify-content: center; gap: 24px;
  flex: 1; min-height: 0;
}

.pad {
  position: relative; width: 300px; height: 300px; border-radius: 50%;
  background: #f1efe8; border: 1px solid #b4b2a9; touch-action: none;
}
.axis { position: absolute; background: #d3d1c7; }
.ax-v { width: 1px; height: 100%; left: 50%; }
.ax-h { height: 1px; width: 100%; top: 50%; }

.knob {
  position: absolute; width: 108px; height: 108px; border-radius: 50%;
  background: #e6f1fb; border: 2px solid #378add;
  left: 96px; top: 96px;
  display: flex; align-items: center; justify-content: center;
  color: #185fa5; font-size: 26px;
  transition: left 0.08s ease-out, top 0.08s ease-out;
}
.knob.dragging { transition: none; }

.readout { display: flex; gap: 32px; font-size: 16px; color: #5f5e5a; }
.readout b { color: #111; font-weight: 700; }
.hint { font-size: 13px; color: #888780; }
</style>
