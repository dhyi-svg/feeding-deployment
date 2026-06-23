<template>
  <div class="teleop">

    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">Manual Control</div>
        <div class="tb-s">Move the arm, then press Done</div>
      </div>
      <button class="btn sm ghost" style="margin-left:auto;height:6vh;padding:0 1.5vw" @click="backToMenu()">Back to Menu</button>
    </div>

    <div class="hla-banner" v-if="currentHla">
      Robot is currently: <b>{{ skillLabel(currentHla) }}</b>
    </div>

    <div class="tabbar">
      <button
        class="tab"
        :class="{ active: tab === 'task' }"
        @click="setTab('task')"
      >Task space</button>
      <button
        class="tab"
        :class="{ active: tab === 'joint' }"
        @click="setTab('joint')"
      >Joint space</button>
    </div>

    <div class="stepsize">
      <span class="stepsize-label">Step size</span>
      <button
        v-for="s in stepSizes"
        :key="s"
        class="step"
        :class="{ active: stepSize === s }"
        @click="setStepSize(s)"
      >{{ stepLabels[s] }}</button>
    </div>

    <div class="status" :class="statusClass">{{ statusText }}</div>

    <div class="tab-content">
    
    <div v-show="tab === 'task'" class="pads">
      <div class="pad">
        <p class="pad-label">Move</p>
        <div class="grid move-grid">
          <button class="jog up"    :disabled="busy" @click="cmd('move.up')">Up</button>
          <button class="jog left"  :disabled="busy" @click="cmd('move.left')">Left</button>
          <button class="jog away"  :disabled="busy" @click="cmd('move.away')">Reach out</button>
          <button class="jog towards" :disabled="busy" @click="cmd('move.towards')">Pull back</button>
          <button class="jog right" :disabled="busy" @click="cmd('move.right')">Right</button>
          <button class="jog down"  :disabled="busy" @click="cmd('move.down')">Down</button>
        </div>
      </div>

      <div class="pad">
        <p class="pad-label">Rotate</p>
        <div class="grid rotate-grid">
          <button class="jog rtiltup"  :disabled="busy" @click="cmd('rotate.tilt_up')">Tilt up</button>
          <button class="jog rturnleft" :disabled="busy" @click="cmd('rotate.turn_left')">Turn left</button>
          <button class="jog rrollright" :disabled="busy" @click="cmd('rotate.roll_right')">Roll right</button>
          <button class="jog rrollleft" :disabled="busy" @click="cmd('rotate.roll_left')">Roll left</button>
          <button class="jog rturnright" :disabled="busy" @click="cmd('rotate.turn_right')">Turn right</button>
          <button class="jog rtiltdown" :disabled="busy" @click="cmd('rotate.tilt_down')">Tilt down</button>
        </div>
      </div>
    </div>

    <div v-show="tab === 'joint'" class="joint-list">
      <div
        v-for="j in joints"
        :key="j.index"
        class="joint-col"
        :class="{ moving: activeJoint === j.index }"
      >
        <span class="joint-label">{{ j.index }}<br>{{ j.label }}</span>
        <button class="jog" :disabled="busy" @click="cmd('joint.' + j.index + '.neg', j.index)" v-html="j.neg"></button>
        <button class="jog" :disabled="busy" @click="cmd('joint.' + j.index + '.pos', j.index)" v-html="j.pos"></button>
      </div>
    </div>
    </div>

    <div class="gripper">
      <p class="pad-label">Gripper</p>
      <div class="gripper-row">
        <button class="jog" :disabled="busy" @click="cmd('gripper.open')">Open</button>
        <button class="jog" :disabled="busy" @click="cmd('gripper.close')">Close</button>
      </div>
    </div>

    <div class="bottom" :class="{ 'three-col': currentHla }">
      <button
        v-if="!retractRunning"
        id="retract-btn"
        class="retract"
        :disabled="busy"
        @click="runRetract()"
      >Retract</button>
      <button
        v-else
        id="stop-btn"
        class="stop"
        @click="stopRetract()"
      >Stop</button>

      <template v-if="currentHla">
        <button
          class="done redo"
          :disabled="busy"
          @click="finishTeleop('redo')"
        >Done — Redo Skill</button>
        <button
          class="done"
          :disabled="busy"
          @click="finishTeleop('next')"
        >Done — Next Skill</button>
      </template>
      <button
        v-else
        class="done"
        :disabled="busy"
        @click="finishTeleop()"
      >Done</button>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'
import { skillLabel } from '@/config/skillLabels'

const DEG = Math.PI / 180
const STEPS = {
  fine: { translation: 0.005, rotation: 2 * DEG, joint: 2 * DEG },
  medium: { translation: 0.02, rotation: 5 * DEG, joint: 5 * DEG },
  coarse: { translation: 0.05, rotation: 15 * DEG, joint: 15 * DEG }
}

const MOTION_TIMEOUT_MS = 15000

const HEARTBEAT_MS = 3000

export default {
  data () {
    return {
      ros: null,
      username: USER,
      tab: 'task',
      stepSize: 'medium',
      stepSizes: ['fine', 'medium', 'coarse'],
      stepLabels: { fine: 'Fine', medium: 'Medium', coarse: 'Coarse' },
      busy: false,
      retractRunning: false,
      activeJoint: null,
      statusText: 'Ready',
      statusClass: 'idle',
      pendingCmdId: null,
      cmdCounter: 0,
      motionTimer: null,
      heartbeatTimer: null,
      publisher: null,
      logPublisher: null,
      listener: null,
      skillPlanListener: null,

      currentHla: null,

      joints: [
        { index: 1, label: 'Base rotate', neg: '&#8634;', pos: '&#8635;' },
        { index: 2, label: 'Shoulder', neg: '&#8634;', pos: '&#8635;' },
        { index: 3, label: 'Arm twist', neg: '&#8634;', pos: '&#8635;' },
        { index: 4, label: 'Elbow bend', neg: '&#8634;', pos: '&#8635;' },
        { index: 5, label: 'Forearm twist', neg: '&#8634;', pos: '&#8635;' },
        { index: 6, label: 'Wrist bend', neg: '&#8634;', pos: '&#8635;' },
        { index: 7, label: 'Wrist twist', neg: '&#8634;', pos: '&#8635;' }
      ]
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initRos()

    if (this.$route.query.hla) {
      this.currentHla = this.$route.query.hla
    }
    
    this.failureContext = this.$route.query.failure || null
    
    this.heartbeatTimer = setInterval(() => {
      this.publish({ state: 'teleop', status: 'heartbeat' })
    }, HEARTBEAT_MS)
  },
  beforeUnmount () {
    this.clearMotionTimer()
    this.clearHeartbeat()
  },
  beforeRouteLeave (to, from, next) {
    this.clearMotionTimer()
    this.clearHeartbeat()
    if (this.listener) {
      this.listener.unsubscribe()
      this.listener = null
    }
    if (this.skillPlanListener) {
      this.skillPlanListener.unsubscribe()
      this.skillPlanListener = null
    }
    if (this.publisher) {
      this.publisher.unadvertise()
      this.publisher = null
    }
    next()
  },
  methods: {
    initRos () {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })

      this.logPublisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/InterventionLog',
        messageType: 'std_msgs/String'
      })

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })
      this.listener.subscribe((msg) => this.handleRosMessage(msg))

      this.skillPlanListener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/skill_plan',
        messageType: 'std_msgs/String'
      })
      this.skillPlanListener.subscribe((msg) => this.handleSkillPlan(msg))
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

    handleRosMessage (msg) {
      let parsed
      try {
        parsed = JSON.parse(msg.data)
      } catch (e) {
        return
      }
      if (parsed.state === 'teleop') {
        const status = parsed.status
        if (status === 'motion_complete' || status === 'motion_aborted') {
          
          if (parsed.cmd_id != null && parsed.cmd_id !== this.pendingCmdId) {
            return
          }
          this.finishMotion(status, parsed.reason || null)
        }
        return
      }

      const route = routeMap[parsed.state]?.[parsed.status]
      if (route) {
        this.$router.push(route)
      }
    },

    backToMenu () {

      this.logEvent('tap', 'back_to_menu', {})
      this.publish({ state: 'teleop', status: 'done' })
      this.$router.push('/task_selection')
    },

    finishTeleop (postAction = null) {
      this.logEvent('tap', 'done', { post_action: postAction })
      const msg = { state: 'teleop', status: 'done' }
      if (postAction) msg.post_action = postAction
      this.publish(msg)
    },

    setTab (tab) {
      if (this.busy) return
      this.tab = tab
    },

    setStepSize (s) {
      if (this.busy) return
      this.stepSize = s
      this.logEvent('tap', 'step_size', { step_size: s })
    },

    stepValue (control) {
      const s = STEPS[this.stepSize]
      if (control.startsWith('move.')) return s.translation
      if (control.startsWith('rotate.')) return s.rotation
      if (control.startsWith('joint.')) return s.joint
      return null
    },

    cmd (control, jointIndex = null) {

      if (control === 'done') {
        this.finishTeleop()
        return
      }

      if (this.busy) {
        this.logEvent('tap_ignored', control, {})
        return
      }

      const value = this.stepValue(control)
      this.beginMotion(control)
      if (jointIndex != null) this.activeJoint = jointIndex

      this.logEvent('tap', control, { value })
      this.publish({
        state: 'teleop',
        status: 'command',
        control,
        step_size: this.stepSize,
        value,
        cmd_id: this.pendingCmdId
      })
    },

    runRetract () {
      if (this.busy) return
      this.beginMotion('retract')
      this.retractRunning = true
      this.logEvent('tap', 'retract', {})
      this.publish({
        state: 'teleop',
        status: 'command',
        control: 'retract',
        cmd_id: this.pendingCmdId
      })
    },

    stopRetract () {
      this.logEvent('tap', 'retract.stop', {})
      this.publish({ state: 'teleop', status: 'halt', cmd_id: this.pendingCmdId })
      
    },

    beginMotion (control) {
      this.busy = true
      this.pendingCmdId = ++this.cmdCounter
      this.statusText = 'Moving…'
      this.statusClass = 'moving'
      this.startMotionTimer(control)
    },

    finishMotion (status, reason) {
      this.clearMotionTimer()
      this.busy = false
      this.retractRunning = false
      this.activeJoint = null
      this.pendingCmdId = null
      if (status === 'motion_aborted') {
        this.statusText = reason ? ('Stopped: ' + reason) : 'Motion stopped'
        this.statusClass = 'aborted'
      } else {
        this.statusText = 'Ready'
        this.statusClass = 'idle'
      }
      this.logEvent(status, this.lastControl, { reason })
    },

    startMotionTimer (control) {
      this.lastControl = control
      this.clearMotionTimer()
      this.motionTimer = setTimeout(() => {
        
        this.busy = false
        this.retractRunning = false
        this.activeJoint = null
        this.pendingCmdId = null
        this.statusText = 'No response — re-enabled'
        this.statusClass = 'aborted'
        this.logEvent('motion_timeout', control, {})
      }, MOTION_TIMEOUT_MS)
    },

    clearMotionTimer () {
      if (this.motionTimer) {
        clearTimeout(this.motionTimer)
        this.motionTimer = null
      }
    },

    clearHeartbeat () {
      if (this.heartbeatTimer) {
        clearInterval(this.heartbeatTimer)
        this.heartbeatTimer = null
      }
    },

    publish (obj) {
      if (!this.publisher) return
      this.publisher.publish(new ROSLIB.Message({ data: JSON.stringify(obj) }))
    },

    logEvent (event, control, extra) {
      const entry = Object.assign({
        t: new Date().toISOString(),
        screen: 'teleop',
        tab: this.tab,
        event,
        control,
        step_size: this.stepSize,
        session_id: this.sessionId || null,
        failure_context: this.failureContext || null
      }, extra || {})
      if (this.logPublisher) {
        this.logPublisher.publish(new ROSLIB.Message({ data: JSON.stringify(entry) }))
      }
    }
  }
}
</script>

<style scoped>

.teleop {
  max-width: 1140px;
  margin: 0 auto;
  font-family: Verdana, sans-serif;
  background: var(--g);
  color: var(--t);
  padding: 8px 20px 10px;
  box-sizing: border-box;
  height: 100vh;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.tabbar { display: flex; gap: 8px; margin: 8px 0; }
.tab {
  flex: 1; font-family: Verdana, sans-serif; font-size: 16px; padding: 11px 0;
  color: var(--t); background: var(--s2); border: 1px solid var(--s3); border-radius: 8px;
  cursor: pointer;
}
.tab.active {
  font-weight: 700; background: var(--a); color: var(--g); border-color: var(--a);
}

.stepsize { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
.stepsize-label { font-size: 14px; color: var(--tm); min-width: 70px; }
.step {
  flex: 1; font-family: Verdana, sans-serif; font-size: 14px; padding: 11px 0;
  border: 1px solid var(--s3); border-radius: 8px; background: var(--s2); color: var(--t);
  cursor: pointer;
}
.step.active {
  font-weight: 700; background: var(--s3); color: var(--t); border-color: var(--s3);
}

.status {
  font-size: 14px; padding: 7px 10px; border-radius: 8px; margin-bottom: 8px;
  text-align: center;
}
.status.idle { background: var(--s2); color: var(--tm); }
.status.moving { background: rgba(240, 165, 0, .15); color: var(--a); }
.status.aborted { background: rgba(220, 60, 60, .15); color: #e88; }

.hla-banner {
  font-size: 15px; padding: 8px 10px; border-radius: 8px; margin-bottom: 8px;
  text-align: center; background: rgba(46, 196, 182, .12); color: var(--a2);
}

.pad-label { font-size: 14px; font-weight: 700; margin: 0 0 6px; color: var(--tm); }

.tab-content { flex: 1; min-height: 0; }

.pads { display: flex; gap: 16px; height: 100%; }
.pad { flex: 1; display: flex; flex-direction: column; min-height: 0; }

.grid {
  display: grid;
  grid-template-columns: 96px 1fr 96px;
  gap: 8px;
  flex: 1;
  min-height: 0;
}

.move-grid { grid-template-rows: repeat(4, 1fr); }
.rotate-grid { grid-template-rows: repeat(4, 1fr); }

.jog {
  font-family: Verdana, sans-serif; font-size: 17px; font-weight: 700;
  border: 1px solid var(--s3); border-radius: 12px; color: var(--t);
  background: var(--s2); cursor: pointer; min-height: 56px;
}
.jog:active { background: var(--s3); }
.jog:disabled, button:disabled { opacity: 0.4; cursor: default; }

.move-grid .up      { grid-column: 1 / 4; grid-row: 1; }
.move-grid .left    { grid-column: 1; grid-row: 2 / 4; }
.move-grid .away    { grid-column: 2; grid-row: 2; }
.move-grid .towards { grid-column: 2; grid-row: 3; }
.move-grid .right   { grid-column: 3; grid-row: 2 / 4; }
.move-grid .down    { grid-column: 1 / 4; grid-row: 4; }

.rotate-grid .rtiltup    { grid-column: 1 / 4; grid-row: 1; }
.rotate-grid .rturnleft  { grid-column: 1; grid-row: 2 / 4; }
.rotate-grid .rrollright { grid-column: 2; grid-row: 2; }
.rotate-grid .rrollleft  { grid-column: 2; grid-row: 3; }
.rotate-grid .rturnright { grid-column: 3; grid-row: 2 / 4; }
.rotate-grid .rtiltdown  { grid-column: 1 / 4; grid-row: 4; }

.joint-list { display: flex; gap: 8px; height: 100%; }
.joint-col {
  flex: 1;
  display: flex; flex-direction: column; gap: 8px;
  align-items: stretch; border-radius: 8px; padding: 6px;
}
.joint-col.moving { background: rgba(240, 165, 0, .1); }
.joint-label {
  font-size: 14px; color: var(--t); text-align: center; line-height: 1.2;
  min-height: 40px; display: flex; align-items: center; justify-content: center;
}
.joint-col .jog { flex: 1; padding: 0; font-size: 46px; line-height: 1; }

.gripper { margin-top: 10px; }
.gripper-row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
.gripper-row .jog { padding: 12px 0; min-height: 56px; }

.bottom {
  display: grid; grid-template-columns: 1fr 1fr; gap: 8px;
  border-top: 1px solid var(--bd); padding-top: 10px; margin-top: 10px;
}

.bottom.three-col { grid-template-columns: 1fr 1fr 1fr; }

.done.redo { background: var(--s2); color: var(--t); }
.retract {
  font-family: Verdana, sans-serif; font-size: 16px; font-weight: 700;
  padding: 12px 0; border: 2px solid rgba(46, 196, 182, .22); border-radius: 8px;
  background: rgba(46, 196, 182, .12); color: var(--a2); cursor: pointer; min-height: 56px;
}
.stop {
  font-family: Verdana, sans-serif; font-size: 16px; font-weight: 700;
  padding: 12px 0; border: none; border-radius: 8px;
  background: #c0392b; color: #fff; cursor: pointer; min-height: 56px;
}
.done {
  font-family: Verdana, sans-serif; font-size: 16px; font-weight: 700;
  padding: 12px 0; border: none; border-radius: 8px;
  background: var(--a); color: var(--g); cursor: pointer; min-height: 56px;
}
</style>
