<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">The robot is working…</div>
      </div>
      <div class="dot"></div>
    </div>

    <div class="ss" v-if="planSlots.current">
      <div class="sk past" v-if="planSlots.last">
        <span class="sl">Previous</span>
        <span class="sv">{{ skillLabel(planSlots.last) }}</span>
      </div>
      <div class="sa" v-if="planSlots.last">&#8594;</div>
      <div class="sk now">
        <span class="sl">Now</span>
        <span class="sv">{{ skillLabel(planSlots.current) }}</span>
      </div>
      <div class="sa" v-if="planSlots.next">&#8594;</div>
      <div class="sk upcoming" v-if="planSlots.next">
        <span class="sl">Next</span>
        <span class="sv">{{ skillLabel(planSlots.next) }}</span>
      </div>
    </div>

    <div class="bd exec-body">
      <div class="exec-text">{{ execText }}</div>
      <div class="exec-status" v-if="busy">
        <span class="spinner"></span>
        <span class="exec-elapsed">still working ({{ elapsedSec }}s)</span>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';
import { skillLabel } from '@/config/skillLabels';

// The safety watchdog publishes /watchdog_status at ~1 kHz and exits after an
// anomaly (having e-stopped the arm), so silence on the topic means the robot
// is in trouble or the stack is restarting. 5 s = 10 throttle intervals.
const WATCHDOG_SILENCE_MS = 5000

export default {
  data () {
    return {
      ros: null,
      username: USER,
      displayedMessage: '',
      // Deterministic activity line (from report_activity on the robot): a
      // concrete "what/why" phrase that takes precedence over displayedMessage
      // (the LLM fallback). `busy` drives the spinner + "still working (Ns)"
      // timer so long Opus/detection waits read as intentional, not frozen.
      activity: '',
      busy: false,
      elapsedSec: 0,
      timerHandle: null,
      listener: null,
      skillPlanListener: null,
      skillPlan: [],
      currentSkillIndex: -1,
      // Armed by the robot (button_arm:on) only while it is blocking on a transfer
      // button press. The physical button fires a global 'takeover-press' event on
      // every page; we relay it to the robot ONLY while armed so stray presses are
      // dropped here rather than queued on the robot side.
      awaitingButton: false,
      // Recovery banner: set while /watchdog_status reports an anomaly, goes
      // silent, or the rosbridge socket drops; cleared on the next healthy tick.
      recovering: false,
      lastWatchdogMs: 0,
      watchdogListener: null,
      watchdogCheckHandle: null
    }
  },
  computed: {
    execText () {
      if (this.recovering) return 'Recovering from an error, please wait…'
      return this.activity || this.displayedMessage
    },

    planSlots () {
      const plan = this.skillPlan
      const idx = this.currentSkillIndex
      if (idx < 0 || idx >= plan.length) {
        return { last: null, current: null, next: null }
      }
      return {
        last: idx > 0 ? plan[idx - 1] : null,
        current: plan[idx],
        next: idx < plan.length - 1 ? plan[idx + 1] : null
      }
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    // A dead rosbridge socket means we can't know the robot's state — treat it
    // like a watchdog outage (App.vue owns reconnection of its own socket).
    this.ros.on('error', () => this.tripRecovery())
    this.ros.on('close', () => this.tripRecovery())
    this.initSubscriber()
    this.initPublisher()
    // The physical transfer button is detected globally in App.vue and dispatched as
    // 'takeover-press'; this page relays it to the robot (only while armed).
    window.addEventListener('takeover-press', this.handleButtonPress)

    if (this.$route.query.plan) {
      this.skillPlan = String(this.$route.query.plan).split(',')
      this.currentSkillIndex = this.$route.query.current != null ? parseInt(this.$route.query.current, 10) : 0
    }
  },
  beforeRouteLeave (to, from, next) {
    window.removeEventListener('takeover-press', this.handleButtonPress)

    if (this.listener) {
      this.listener.unsubscribe();
      this.listener = null;
    }

    if (this.skillPlanListener) {
      this.skillPlanListener.unsubscribe();
      this.skillPlanListener = null;
    }

    if (this.watchdogListener) {
      this.watchdogListener.unsubscribe();
      this.watchdogListener = null;
    }

    if (this.watchdogCheckHandle) {
      clearInterval(this.watchdogCheckHandle);
      this.watchdogCheckHandle = null;
    }

    this.stopTimer();

    if (this.publisher) {
      this.publisher.unadvertise();
      this.publisher = null;
    }

    next(); 
  },
  methods: {
    handleRosMessage(message) {

      try {
        const parsedMessage = JSON.parse(message.data);
        if (parsedMessage.state === 'activity') {
          this.handleActivity(parsedMessage);
          return;
        }
        if (parsedMessage.state === 'button_arm') {
          // Robot is (dis)arming the transfer button; enable relaying while 'on'.
          this.awaitingButton = parsedMessage.status === 'on';
          return;
        }
        if (parsedMessage.state === 'explanation') {
          // LLM narrator fallback line (provide_continuous_explanations).
          if (parsedMessage.status) {
            this.displayedMessage = parsedMessage.status;
          }
          return;
        }
        // Anything else without a routeMap entry is an internal signal
        // (base_control enabled/disabled, auto_time, ...) — never show its
        // raw status string to the user.
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          if (typeof route === 'string') {
            this.$router.push(route); 
          } else if (typeof route === 'object') {
            this.$router.push(route); 
          }
        }
      } catch (error) {
      }
    },
    skillLabel(name) {
      return skillLabel(name);
    },
    handleActivity(msg) {
      const text = msg.status || '';
      const busy = !!msg.busy;
      if (!text || !busy) {
        // Cleared: stop the timer and fall back to the LLM explanation line.
        this.activity = '';
        this.busy = false;
        this.stopTimer();
        return;
      }
      // New phase -> restart the elapsed counter; same text -> keep counting.
      if (text !== this.activity) {
        this.elapsedSec = 0;
      }
      this.activity = text;
      this.busy = true;
      this.ensureTimer();
    },
    ensureTimer() {
      if (this.timerHandle) return;
      this.timerHandle = setInterval(() => { this.elapsedSec += 1; }, 1000);
    },
    stopTimer() {
      if (this.timerHandle) {
        clearInterval(this.timerHandle);
        this.timerHandle = null;
      }
      this.elapsedSec = 0;
    },
    handleSkillPlan(message) {
      try {
        const parsed = JSON.parse(message.data);
        if (Array.isArray(parsed.plan) && typeof parsed.current === 'number') {
          this.skillPlan = parsed.plan;
          this.currentSkillIndex = parsed.current;
        }
      } catch (error) {
      }
    },
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
    handleButtonPress() {
      // Relay the physical transfer button to the robot ONLY while it has armed us
      // (button_arm:on). Unarmed presses are dropped so they can't satisfy a later
      // wait. Stay on this page; the robot advances the flow.
      if (!this.awaitingButton || !this.publisher) return;
      this.publisher.publish(new ROSLIB.Message({
        data: JSON.stringify({ state: 'button_press', status: 'pressed' })
      }));
    },
    initSubscriber() {

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp', 
        messageType: 'std_msgs/String' 
      })

      this.skillPlanListener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/skill_plan',
        messageType: 'std_msgs/String'
      })
      this.skillPlanListener.subscribe((message) => this.handleSkillPlan(message))

      this.listener.subscribe((message) => {
        this.handleRosMessage(message);
      });

      // throttle_rate makes rosbridge forward at most one message per 500 ms,
      // so the watchdog's ~1 kHz publish rate never reaches the websocket.
      this.watchdogListener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/watchdog_status',
        messageType: 'std_msgs/Bool',
        throttle_rate: 500,
        queue_length: 1
      })
      this.watchdogListener.subscribe((message) => this.handleWatchdog(message))

      // Grace period from mount so we don't flash the recovery text before the
      // first (throttled) status arrives.
      this.lastWatchdogMs = Date.now()
      this.watchdogCheckHandle = setInterval(() => {
        if (Date.now() - this.lastWatchdogMs > WATCHDOG_SILENCE_MS) {
          this.tripRecovery()
        }
      }, 1000)
    },
    handleWatchdog(message) {
      this.lastWatchdogMs = Date.now()
      if (message.data === false) {
        this.tripRecovery()
      } else {
        this.recovering = false
      }
    },
    tripRecovery() {
      // Wipe the stale activity/explanation text and spinner; execText shows
      // the polite recovery line until the watchdog reports healthy again.
      if (this.recovering) return
      this.recovering = true
      this.activity = ''
      this.busy = false
      this.displayedMessage = ''
      this.stopTimer()
    },

  }
}
</script>

<style scoped>
.ss {
  background: var(--s1);
  padding: 1.2vh 2vw;
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 1.2vw;
  border-bottom: 1px solid var(--bd);
  flex-shrink: 0;
}

.sk {
  display: flex;
  flex-direction: column;
  align-items: center;
  min-width: 12vw;
  padding: 1vh 1.2vw;
  border-radius: 10px;
  background: var(--s2);
  color: var(--tm);
  border: 2px solid transparent;
}

.sk.now {
  background: rgba(240, 165, 0, .08);
  color: var(--t);
  border-color: var(--a);
}

.sk .sl {
  font-size: 1.4vh;
  text-transform: uppercase;
  letter-spacing: 1px;
  opacity: 0.8;
}

.sk .sv {
  font-size: 2.2vh;
  font-weight: bold;
  margin-top: 4px;
  color: inherit;
}

.sk.now .sv {
  color: var(--a);
}

.sa {
  font-size: 2.4vh;
  color: var(--tm);
}

.exec-body {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 3vh;
  flex: 1;
}

.exec-text {
  font-family: Verdana;
  font-size: 4.2vh;
  color: var(--t);
  text-align: center;
  max-width: 80vw;
  line-height: 1.4;
}

.exec-status {
  display: flex;
  align-items: center;
  gap: 1vw;
  color: var(--tm);
  font-size: 2.2vh;
}

.exec-elapsed {
  font-variant-numeric: tabular-nums;
}

.spinner {
  width: 2.2vh;
  height: 2.2vh;
  border: 0.35vh solid var(--s2);
  border-top-color: var(--a);
  border-radius: 50%;
  animation: spin 0.9s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}
</style>
