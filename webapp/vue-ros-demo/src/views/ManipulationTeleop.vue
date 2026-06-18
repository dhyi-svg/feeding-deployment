<template>
  <div class="teleop">
    <!-- Header (matches the app's top bar) -->
    <div class="header">
      <img class="user" alt="User" src="https://c.animaapp.com/jvBoNEN4/img/user.svg">
      <div class="header-text">
        <div class="header-title">Manual Control</div>
        <div class="header-sub">Move the arm, then press Done</div>
      </div>
      <!-- Client-side escape: leaves the page even if the robot is unresponsive. -->
      <button class="menu-btn" @click="backToMenu()">Back to Menu</button>
    </div>

    <!-- What the robot was doing when control was handed over. -->
    <div class="hla-banner" v-if="currentHla">
      Robot is currently: <b>{{ skillLabel(currentHla) }}</b>
    </div>

    <!-- Tab bar -->
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

    <!-- Step size -->
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

    <!-- Status strip -->
    <div class="status" :class="statusClass">{{ statusText }}</div>

    <!-- Tab content: fixed height so the Gripper / Retract / Done rows below
         stay at the same position when switching between tabs. -->
    <div class="tab-content">
    <!-- Task space tab -->
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

    <!-- Joint space tab -->
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

    <!-- Gripper (applies in both tabs) -->
    <div class="gripper">
      <p class="pad-label">Gripper</p>
      <div class="gripper-row">
        <button class="jog" :disabled="busy" @click="cmd('gripper.open')">Open</button>
        <button class="jog" :disabled="busy" @click="cmd('gripper.close')">Close</button>
      </div>
    </div>

    <!-- Bottom row -->
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

      <!-- Mid-skill takeover: let the user choose what happens after they finish.
           "Redo" re-runs the interrupted skill; "Continue" treats it as done and
           advances to the next skill. -->
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

// Step increments. Translation in metres, rotation/joints in radians.
const DEG = Math.PI / 180
const STEPS = {
  fine: { translation: 0.005, rotation: 2 * DEG, joint: 2 * DEG },
  medium: { translation: 0.02, rotation: 5 * DEG, joint: 5 * DEG },
  coarse: { translation: 0.05, rotation: 15 * DEG, joint: 15 * DEG }
}

// Re-enable the UI if no motion_complete/aborted comes back within this window.
const MOTION_TIMEOUT_MS = 15000

// Liveness ping so the robot can tell "user is connected but idle" from
// "iPad disconnected" and exit the session instead of hanging. Must be well
// under the robot-side SESSION_TIMEOUT (see teleop_recovery.py).
const HEARTBEAT_MS = 3000

export default {
  data () {
    return {
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
      // Snake_case name of the HLA the robot is executing (null if idle), from
      // the latched /SkillPlan topic.
      currentHla: null,
      // Every joint is revolute, so each group uses the same convention:
      // top button = counterclockwise (neg, ↺), bottom button = clockwise (pos, ↻).
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
    this.initRos()
    // Dev/testing hook: inject the current HLA via URL when running without a
    // backend, e.g. /#/manipulation_teleop?hla=acquire_bite
    if (this.$route.query.hla) {
      this.currentHla = this.$route.query.hla
    }
    // failure_context is passed through the route query when the executive opens this screen
    this.failureContext = this.$route.query.failure || null
    // Liveness ping so the robot can detect an iPad disconnect and exit teleop.
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
      const ros = new ROSLIB.Ros({ url: ROS_URL })

      this.publisher = new ROSLIB.Topic({
        ros,
        name: '/WebAppComm',
        messageType: 'std_msgs/String'
      })

      // Intervention log: one JSON line per interaction, recorded robot-side.
      this.logPublisher = new ROSLIB.Topic({
        ros,
        name: '/InterventionLog',
        messageType: 'std_msgs/String'
      })

      this.listener = new ROSLIB.Topic({
        ros,
        name: '/ServerComm',
        messageType: 'std_msgs/String'
      })
      this.listener.subscribe((msg) => this.handleRosMessage(msg))

      // Latched skill plan: tells us which HLA the robot is executing. Latching
      // means we get the current value immediately even though this screen only
      // mounts after the takeover (i.e. after the plan was published).
      this.skillPlanListener = new ROSLIB.Topic({
        ros,
        name: '/SkillPlan',
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
        console.error('Teleop: failed to parse /SkillPlan message:', e)
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
        console.error('Teleop: failed to parse ROS message:', e)
        return
      }
      if (parsed.state === 'teleop') {
        const status = parsed.status
        if (status === 'motion_complete' || status === 'motion_aborted') {
          // Ignore stale completions from a command we are no longer waiting on.
          if (parsed.cmd_id != null && parsed.cmd_id !== this.pendingCmdId) {
            return
          }
          this.finishMotion(status, parsed.reason || null)
        }
        return
      }

      // Any other state: let the executive drive navigation (e.g. it sends a
      // task_selection jump after the user taps Done), like every other page.
      const route = routeMap[parsed.state]?.[parsed.status]
      if (route) {
        this.$router.push(route)
      }
    },

    backToMenu () {
      // Escape hatch: signal the executive to end the session AND route locally,
      // so the user isn't stranded if the robot/rosbridge is unresponsive.
      // Enabled even while busy on purpose (it's the "get me out" control).
      this.logEvent('tap', 'back_to_menu', {})
      this.publish({ state: 'teleop', status: 'done' })
      this.$router.push('/task_selection')
    },

    // End the teleop session. On a mid-skill takeover, postAction ("redo" |
    // "next") tells the executive whether to re-run the interrupted skill or
    // advance to the next one. Routing is driven by the executive, like Done.
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

    // value of the active step in the units relevant to the control
    stepValue (control) {
      const s = STEPS[this.stepSize]
      if (control.startsWith('move.')) return s.translation
      if (control.startsWith('rotate.')) return s.rotation
      if (control.startsWith('joint.')) return s.joint
      return null
    },

    cmd (control, jointIndex = null) {
      // Done is a terminal action, not a bounded motion. We only signal the
      // executive; it drives navigation to the correct next page (like every
      // other page in this app), so we do NOT route locally here.
      if (control === 'done') {
        this.finishTeleop()
        return
      }

      // Gate: a tap while busy is ignored and logged, never queued.
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
      // The controller should respond with motion_aborted; finishMotion clears state.
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
        // No completion came back — recover rather than soft-lock the screen.
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
      console.log('[intervention]', JSON.stringify(entry))
      if (this.logPublisher) {
        this.logPublisher.publish(new ROSLIB.Message({ data: JSON.stringify(entry) }))
      }
    }
  }
}
</script>

<style scoped>
/* Palette mirrors the rest of the app:
   Verdana, slate #6e7e8e for selected toggles, yellow #FFE699 action buttons,
   #eee bars, green #28a745 confirm, red #ff4d4f for stop/error. */
.teleop {
  max-width: 1140px;
  margin: 0 auto;
  font-family: Verdana, sans-serif;
  background: #fff;
  padding: 8px 20px 10px;
  box-sizing: border-box;
  /* Fill the viewport and never scroll; the tab area flexes to absorb the
     HLA banner so everything stays on one screen (e.g. the 1180x820 iPad). */
  height: 100vh;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

/* Header (matches the app's top bar) */
.header {
  display: flex; align-items: center; gap: 12px;
  background: #eee; border-radius: 8px;
  padding: 6px 14px; margin-bottom: 8px;
}
.header .user { width: 36px; height: 36px; }
.header-title { font-size: 20px; font-weight: 700; color: #1f2937; }
.header-sub { font-size: 14px; color: #6e7e8e; }
.menu-btn {
  margin-left: auto; font-family: Verdana, sans-serif; font-size: 15px;
  font-weight: 700; padding: 10px 18px; border: none; border-radius: 8px;
  background: #6e7e8e; color: #fff; cursor: pointer; min-height: 44px;
}

/* Tab bar */
.tabbar { display: flex; gap: 8px; margin-bottom: 8px; }
.tab {
  flex: 1; font-family: Verdana, sans-serif; font-size: 16px; padding: 11px 0;
  color: #1f2937; background: #fff; border: 1px solid #ccc; border-radius: 8px;
  cursor: pointer;
}
.tab.active {
  font-weight: 700; background: #6e7e8e; color: #fff; border-color: #6e7e8e;
}

/* Step size */
.stepsize { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
.stepsize-label { font-size: 14px; color: #6e7e8e; min-width: 70px; }
.step {
  flex: 1; font-family: Verdana, sans-serif; font-size: 14px; padding: 11px 0;
  border: 1px solid #ccc; border-radius: 8px; background: #fff; color: #1f2937;
  cursor: pointer;
}
.step.active {
  font-weight: 700; background: #6e7e8e; color: #fff; border-color: #6e7e8e;
}

/* Status strip */
.status {
  font-size: 14px; padding: 7px 10px; border-radius: 8px; margin-bottom: 8px;
  text-align: center;
}
.status.idle { background: #eee; color: #6e7e8e; }
.status.moving { background: #FFE699; color: #6b5900; }
.status.aborted { background: #ffd6d6; color: #c0392b; }

.hla-banner {
  font-size: 15px; padding: 8px 10px; border-radius: 8px; margin-bottom: 8px;
  text-align: center; background: #e3edf7; color: #2c5777;
}

.pad-label { font-size: 14px; font-weight: 700; margin: 0 0 6px; color: #6e7e8e; }

/* Flexible tab area: fills the leftover height and keeps the Gripper / Retract
   / Done rows anchored whether Task space or Joint space is shown. */
.tab-content { flex: 1; min-height: 0; }

/* Move and Rotate pads side by side */
.pads { display: flex; gap: 16px; height: 100%; }
.pad { flex: 1; display: flex; flex-direction: column; min-height: 0; }

.grid {
  display: grid;
  grid-template-columns: 96px 1fr 96px;
  gap: 8px;
  flex: 1;
  min-height: 0;
}
/* Rows share the available height so the pads scale to fit the screen. */
.move-grid { grid-template-rows: repeat(4, 1fr); }
.rotate-grid { grid-template-rows: repeat(4, 1fr); }

/* Action buttons use the app's yellow #FFE699 */
.jog {
  font-family: Verdana, sans-serif; font-size: 17px; font-weight: 700;
  border: 1px solid #e6c95c; border-radius: 12px; color: #1f2937;
  background: #FFE699; cursor: pointer; min-height: 56px;
}
.jog:active { background: #f5d36b; }
.jog:disabled, button:disabled { opacity: 0.4; cursor: default; }

/* Move pad placement */
.move-grid .up      { grid-column: 1 / 4; grid-row: 1; }
.move-grid .left    { grid-column: 1; grid-row: 2 / 4; }
.move-grid .away    { grid-column: 2; grid-row: 2; }
.move-grid .towards { grid-column: 2; grid-row: 3; }
.move-grid .right   { grid-column: 3; grid-row: 2 / 4; }
.move-grid .down    { grid-column: 1 / 4; grid-row: 4; }

/* Rotate pad placement */
.rotate-grid .rtiltup    { grid-column: 1 / 4; grid-row: 1; }
.rotate-grid .rturnleft  { grid-column: 1; grid-row: 2 / 4; }
.rotate-grid .rrollright { grid-column: 2; grid-row: 2; }
.rotate-grid .rrollleft  { grid-column: 2; grid-row: 3; }
.rotate-grid .rturnright { grid-column: 3; grid-row: 2 / 4; }
.rotate-grid .rtiltdown  { grid-column: 1 / 4; grid-row: 4; }

/* Joints laid out horizontally: one column per joint. The list fills the same
   fixed tab area as the task pads, and the two buttons in each column grow to
   use that height, with large rotation icons. */
.joint-list { display: flex; gap: 8px; height: 100%; }
.joint-col {
  flex: 1;
  display: flex; flex-direction: column; gap: 8px;
  align-items: stretch; border-radius: 8px; padding: 6px;
}
.joint-col.moving { background: #fff4d6; }
.joint-label {
  font-size: 14px; color: #1f2937; text-align: center; line-height: 1.2;
  min-height: 40px; display: flex; align-items: center; justify-content: center;
}
.joint-col .jog { flex: 1; padding: 0; font-size: 46px; line-height: 1; }

/* Gripper */
.gripper { margin-top: 10px; }
.gripper-row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
.gripper-row .jog { padding: 12px 0; min-height: 56px; }

/* Bottom row */
.bottom {
  display: grid; grid-template-columns: 1fr 1fr; gap: 8px;
  border-top: 1px solid #ddd; padding-top: 10px; margin-top: 10px;
}
/* Mid-skill takeover adds a second Done button (redo vs. next). */
.bottom.three-col { grid-template-columns: 1fr 1fr 1fr; }
/* "Redo" is the app's slate secondary so it reads distinct from the green
   "next" Done. */
.done.redo { background: #6e7e8e; }
.retract {
  font-family: Verdana, sans-serif; font-size: 16px; font-weight: 700;
  padding: 12px 0; border: none; border-radius: 8px;
  background: #6e7e8e; color: #fff; cursor: pointer; min-height: 56px;
}
.stop {
  font-family: Verdana, sans-serif; font-size: 16px; font-weight: 700;
  padding: 12px 0; border: none; border-radius: 8px;
  background: #ff4d4f; color: #fff; cursor: pointer; min-height: 56px;
}
.done {
  font-family: Verdana, sans-serif; font-size: 16px; font-weight: 700;
  padding: 12px 0; border: none; border-radius: 8px;
  background: #28a745; color: #fff; cursor: pointer; min-height: 56px;
}
</style>
