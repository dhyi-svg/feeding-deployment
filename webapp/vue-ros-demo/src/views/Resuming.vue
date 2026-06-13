<template>
  <div class="resuming">
    <div class="message">Resuming…</div>
    <div class="sub">The robot is continuing the task.</div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL } from '@/config/parameterConfig'

// Shown briefly after a mid-skill manual takeover ends, until the executive's
// next page jump arrives. Listens on /ServerComm and follows routeMap so the
// robot can move it on to the real execution page.
export default {
  data () {
    return { listener: null }
  },
  mounted () {
    const ros = new ROSLIB.Ros({ url: ROS_URL })
    this.listener = new ROSLIB.Topic({
      ros,
      name: '/ServerComm',
      messageType: 'std_msgs/String'
    })
    this.listener.subscribe((msg) => {
      try {
        const parsed = JSON.parse(msg.data)
        if (parsed.state === 'resuming') return
        const route = routeMap[parsed.state]?.[parsed.status]
        if (route) this.$router.push(route)
      } catch (e) {
        console.error('Resuming: failed to parse message:', e)
      }
    })
  },
  beforeRouteLeave (to, from, next) {
    if (this.listener) {
      this.listener.unsubscribe()
      this.listener = null
    }
    next()
  }
}
</script>

<style scoped>
.resuming {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 90vh;
  font-family: Verdana, sans-serif;
}
.message {
  font-size: 40px;
  font-weight: bold;
  color: #1f2937;
}
.sub {
  margin-top: 12px;
  font-size: 18px;
  color: #6e7e8e;
}
</style>
