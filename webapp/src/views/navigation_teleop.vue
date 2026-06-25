<template>
  <div class="navteleop">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">{{ subtitle }}</div>
      </div>
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

    <div class="bottom" :class="{ 'two-col': navMode === 'skill' }">
      <template v-if="navMode === 'skill'">
        <button class="navbtn resume" @click="resume()">Resume</button>
        <button class="navbtn done" @click="finish()">Done</button>
      </template>
      <button v-else class="navbtn return" @click="goBack()">Return</button>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'
import { skillLabel } from '@/config/skillLabels'
import { categoryOf } from '@/config/skillCategories'

const MAX_LIN = 0.30
const MAX_ANG = 0.60
const SEND_HZ = 10
const R = 96
const KCENTER = 96
const DEADZONE = 0.05
const HEARTBEAT_MS = 3000

export default {
  name: 'NavigationTeleop',
  data () {
    return {
      username: USER,
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
      resumePub: null,
      cancelPub: null,
      armPub: null,
      heartbeatTimer: null,
      listener: null,
      skillPlanListener: null,

      currentHla: null,
      // True when reached as a base-driving detour from the arm-teleop chooser
      // (manipulation_done). The arm session is paused, not concluded, so we must
      // keep its heartbeat alive and follow the shared-autonomy takeover/cancel
      // protocol — Return goes back to the chooser, not the executive.
      detour: false,
      // 'skill' = teleop *during* a running navigation skill (Resume/Done hand-off);
      // 'aux'   = idle / a manipulation skill is current → manual driving + Return.
      navMode: 'aux',
      // Page to return to in aux mode; set from the referrer in beforeRouteEnter.
      referrer: '/robot_executing'
    }
  },
  computed: {
    bannerText () {
      return this.connected ? 'connected' : 'connection lost — base stopping'
    },
    subtitle () {
      return this.navMode === 'skill'
        ? 'Base navigation — push to drive, Resume to let autonomy finish, Done when parked'
        : 'Manual base driving — push to drive, Return when finished'
    }
  },
  beforeRouteEnter (to, from, next) {
    next(vm => {
      vm.referrer = (from && from.fullPath && from.fullPath !== '/' && from.fullPath !== to.fullPath)
        ? from.fullPath
        : '/robot_executing'
    })
  },
  mounted () {

    if (this.$route.query.hla) {
      this.currentHla = this.$route.query.hla
    }
    this.detour = this.$route.query.detour === '1'
    // Freeze the button set at entry: skill mode only when a navigation skill is
    // actually running (callers pass ?hla for that). Everything else is aux
    // (a base-driving detour from arm teleop is aux too — its hla is a
    // manipulation skill, so categoryOf is never 'navigation').
    this.navMode = (this.currentHla && categoryOf(this.currentHla) === 'navigation') ? 'skill' : 'aux'
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.ros.on('connection', () => { this.connected = true })
    this.ros.on('close', () => this.setLinkHealthy(false))
    this.ros.on('error', () => this.setLinkHealthy(false))

    this.cmdVelPub = new ROSLIB.Topic({ ros: this.ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' })

    this.takeoverPub = new ROSLIB.Topic({ ros: this.ros, name: '/shared_autonomy/takeover', messageType: 'std_msgs/Empty' })
    this.donePub = new ROSLIB.Topic({ ros: this.ros, name: '/shared_autonomy/done', messageType: 'std_msgs/Empty' })
    // Resume: hand back to autonomy, which replans from the current pose to the
    // original goal (manager re-sends the goal). Distinct from done/blind-success.
    this.resumePub = new ROSLIB.Topic({ ros: this.ros, name: '/shared_autonomy/resume', messageType: 'std_msgs/Empty' })
    // Cancel: end a takeover WITHOUT reporting goal-reached. Used by the base
    // detour, where there is no navigation goal to "complete" — the manager
    // aborts (never set_succeeded), so no spurious goal-reached is reported.
    this.cancelPub = new ROSLIB.Topic({ ros: this.ros, name: '/shared_autonomy/cancel', messageType: 'std_msgs/Empty' })
    // Heartbeat channel for the paused arm-teleop session (detour only).
    this.armPub = new ROSLIB.Topic({ ros: this.ros, name: '/webapp_to_robot', messageType: 'std_msgs/String' })

    // Follow the executive's page jumps like every other page.
    this.listener = new ROSLIB.Topic({ ros: this.ros, name: '/robot_to_webapp', messageType: 'std_msgs/String' })
    this.listener.subscribe((msg) => this.handleRosMessage(msg))

    this.skillPlanListener = new ROSLIB.Topic({ ros: this.ros, name: '/skill_plan', messageType: 'std_msgs/String' })
    this.skillPlanListener.subscribe((msg) => this.handleSkillPlan(msg))

    // Announce a shared-autonomy takeover both when interrupting a running
    // navigation skill (skill mode) and on a base detour from arm teleop, so the
    // manager always sees the protocol. A plain idle aux drive (e.g. from the
    // task menu) isn't taking anything over, so it stays silent.
    if (this.navMode === 'skill' || this.detour) {
      this.takeoverPub.publish(new ROSLIB.Message({}))
    }

    // On a detour the arm-teleop session is only paused; keep its heartbeat
    // alive so the backend (10s no-heartbeat timeout) doesn't conclude it.
    if (this.detour) {
      this.heartbeatTimer = setInterval(() => {
        if (this.armPub) {
          this.armPub.publish(new ROSLIB.Message({
            data: JSON.stringify({ state: 'teleop', status: 'heartbeat' })
          }))
        }
      }, HEARTBEAT_MS)
    }

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
      // Stop the base and tell the manager the human has parked the robot at the
      // goal (blind success). The executive then continues to the NEXT skill in
      // the plan, so return to the skill-plan (explanation) page -- NOT the task
      // menu -- so the user sees what's next and the executive can page-jump on.
      // Same destination as resume(); only the published intent differs.
      this.center()
      this.sendVelocity()
      if (this.donePub) this.donePub.publish(new ROSLIB.Message({}))
      this.$router.push('/robot_executing')
    },
    resume () {
      // Stop driving, tell the manager to hand back to autonomy (it replans from
      // the current pose to the original goal), and return to the skill-plan
      // (explanation) page -- the same place manipulation teleop returns to.
      // The takeover button is available there, so the user can take over again
      // if the robot gets stuck again on the way to the goal.
      this.center()
      this.sendVelocity()
      if (this.resumePub) this.resumePub.publish(new ROSLIB.Message({}))
      this.$router.push('/robot_executing')
    },
    goBack () {
      // Stop the base and return to whatever page sent us here. On a detour the
      // human is ending the takeover without reaching any goal, so publish
      // /shared_autonomy/cancel (NOT done — that would report a bogus
      // goal-reached) and return to the chooser (the referrer). A plain idle aux
      // drive took nothing over, so it just returns silently.
      this.center()
      this.sendVelocity()
      if (this.detour && this.cancelPub) this.cancelPub.publish(new ROSLIB.Message({}))
      this.$router.push(this.referrer || '/robot_executing')
    },
    teardown () {
      this.center()
      if (this.cmdVelPub) this.sendVelocity()
      if (this.heartbeatTimer) { clearInterval(this.heartbeatTimer); this.heartbeatTimer = null }
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
  width: 100%;
  font-family: Verdana, sans-serif;
  background: var(--g);
  color: var(--t);
  padding: 8px 3vw 10px;
  box-sizing: border-box;
  height: 100vh;
  height: 100dvh;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.banner {
  text-align: center; font-size: 16px; padding: 8px; border-radius: 8px;
  color: var(--a2); background: rgba(46, 196, 182, .12); margin: 8px 0;
}
.banner.bad { color: #e88; background: rgba(220, 60, 60, .12); }

.hla-banner {
  text-align: center; font-size: 17px; padding: 8px; border-radius: 8px;
  color: var(--a2); background: rgba(46, 196, 182, .12); margin-bottom: 8px;
}

.joystick-area {
  display: flex; flex-direction: column; align-items: center;
  justify-content: center; gap: 24px;
  flex: 1; min-height: 0;
}

.pad {
  position: relative; width: 300px; height: 300px; border-radius: 50%;
  background: var(--s2); border: 1px solid var(--s3); touch-action: none;
}
.axis { position: absolute; background: var(--s3); }
.ax-v { width: 1px; height: 100%; left: 50%; }
.ax-h { height: 1px; width: 100%; top: 50%; }

.knob {
  position: absolute; width: 108px; height: 108px; border-radius: 50%;
  background: rgba(46, 196, 182, .15); border: 2px solid var(--a2);
  left: 96px; top: 96px;
  display: flex; align-items: center; justify-content: center;
  color: var(--a2); font-size: 26px;
  transition: left 0.08s ease-out, top 0.08s ease-out;
}
.knob.dragging { transition: none; }

.readout { display: flex; gap: 32px; font-size: 19px; color: var(--tm); }
.readout b { color: var(--t); font-weight: 700; }
.hint { font-size: 15px; color: var(--tm); }

/* Bottom action bar — mirrors the manipulation_teleop layout. */
.bottom {
  display: grid; grid-template-columns: 1fr; gap: 8px;
  border-top: 1px solid var(--bd); padding-top: 10px; margin-top: 10px;
}
.bottom.two-col { grid-template-columns: 1fr 1fr; }
.navbtn {
  font-family: Verdana, sans-serif; font-size: 19px; font-weight: 700;
  padding: 12px 0; border-radius: 8px; cursor: pointer; min-height: 64px; border: none;
}
.navbtn.resume {
  background: rgba(46, 196, 182, .12); color: var(--a2);
  border: 2px solid rgba(46, 196, 182, .22);
}
.navbtn.done { background: var(--a); color: var(--g); }
.navbtn.return { background: var(--s2); color: var(--t); border: 1px solid var(--s3); }
</style>
