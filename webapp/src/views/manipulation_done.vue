<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">{{ subtitle }}</div>
      </div>
    </div>
    <div class="bd">
      <div class="choice-row">
        <button class="choice-card" @click="moveBase">
          <div class="cc-ico">🕹️</div>
          <div class="cc-lbl">Move the Base</div>
          <div class="cc-sub">drive the base, then come back</div>
        </button>
        <button class="choice-card" @click="redoSkill">
          <div class="cc-ico">🔁</div>
          <div class="cc-lbl">Redo Skill</div>
          <div class="cc-sub">re-run the current skill</div>
        </button>
        <button class="choice-card" @click="nextSkill">
          <div class="cc-ico">⏭️</div>
          <div class="cc-lbl">Next Skill</div>
          <div class="cc-sub">continue to the next skill</div>
        </button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'
import { skillLabel } from '@/config/skillLabels'

const HEARTBEAT_MS = 3000

export default {
  name: 'ManipulationDone',
  data () {
    return {
      ros: null,
      publisher: null,
      listener: null,
      heartbeatTimer: null,
      username: USER,
      currentHla: null
    }
  },
  computed: {
    subtitle () {
      return this.currentHla
        ? `Arm control done for "${skillLabel(this.currentHla)}" — what next?`
        : 'Arm control done — what next?'
    }
  },
  mounted () {
    this.currentHla = this.$route.query.hla || null
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: '/webapp_to_robot',
      messageType: 'std_msgs/String'
    })
    // Still follow executive page-jumps like every other page.
    this.listener = new ROSLIB.Topic({
      ros: this.ros,
      name: '/robot_to_webapp',
      messageType: 'std_msgs/String'
    })
    this.listener.subscribe((msg) => this.handleRosMessage(msg))

    // The arm-teleop session is only PAUSED here, not concluded -- it still
    // blocks on a {teleop, done} and times out after ~10s with no heartbeat.
    // Keep it alive so the user can deliberate (and detour to base driving)
    // without the backend giving up. The real done+post_action fires only when
    // Redo/Next is chosen.
    this.heartbeatTimer = setInterval(() => {
      if (this.publisher) {
        this.publisher.publish(new ROSLIB.Message({
          data: JSON.stringify({ state: 'teleop', status: 'heartbeat' })
        }))
      }
    }, HEARTBEAT_MS)
  },
  beforeUnmount () {
    this.clearHeartbeat()
  },
  beforeRouteLeave (to, from, next) {
    this.clearHeartbeat()
    if (this.listener) { this.listener.unsubscribe(); this.listener = null }
    next()
  },
  methods: {
    clearHeartbeat () {
      if (this.heartbeatTimer) { clearInterval(this.heartbeatTimer); this.heartbeatTimer = null }
    },
    handleRosMessage (msg) {
      try {
        const parsed = JSON.parse(msg.data)
        const route = routeMap[parsed.state]?.[parsed.status]
        if (route) this.$router.push(route)
      } catch (e) { /* ignore non-JSON */ }
    },
    publishDone (postAction) {
      if (this.publisher) {
        this.publisher.publish(new ROSLIB.Message({
          data: JSON.stringify({ state: 'teleop', status: 'done', post_action: postAction })
        }))
      }
    },
    moveBase () {
      // Base-driving detour. ?detour=1 tells navigation_teleop to follow the
      // shared-autonomy protocol (publish /shared_autonomy/takeover on entry,
      // /shared_autonomy/cancel on Return) and to keep the arm-teleop heartbeat
      // alive while driving. Its Return comes back to THIS page (referrer), so
      // the user can then pick Redo / Next. Arm teleop is NOT concluded here.
      this.$router.push({ path: '/navigation_teleop', query: { detour: '1', hla: this.currentHla || undefined } })
    },
    redoSkill () {
      this.publishDone('redo')
      this.$router.push('/robot_executing')
    },
    nextSkill () {
      this.publishDone('next')
      this.$router.push('/robot_executing')
    }
  }
}
</script>
