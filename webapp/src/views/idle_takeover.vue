<template>
  <div class="page">
    <div class="bd idle-bd">
      <h1 class="idle-title">Robot is idle</h1>
      <p class="idle-sub">No skill is running. Choose what you want to teleoperate.</p>
      <div class="choice-row">
        <button class="choice-card" @click="takeoverBase">
          <div class="cc-ico">🕹️</div>
          <div class="cc-lbl">Navigation</div>
          <div class="cc-sub">drive the base</div>
        </button>
        <button class="choice-card" @click="takeoverArm">
          <div class="cc-ico">🦾</div>
          <div class="cc-lbl">Manipulation</div>
          <div class="cc-sub">move the arm</div>
        </button>
      </div>
    </div>
  </div>
</template>

<script>

import ROSLIB from 'roslib'
import { ROS_URL } from '@/config/parameterConfig'

export default {
  name: 'IdleTakeover',
  data () {
    return { ros: null, webAppPub: null }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.webAppPub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/webapp_to_robot',
      messageType: 'std_msgs/String'
    })
  },
  methods: {
    takeoverArm () {
      
      if (this.webAppPub) {
        this.webAppPub.publish(new ROSLIB.Message({
          data: JSON.stringify({ state: 'teleop', status: 'takeover' })
        }))
      }
      this.$router.push('/manipulation_teleop')
    },
    takeoverBase () {
      
      this.$router.push('/navigation_teleop')
    }
  }
}
</script>

<style scoped>
.idle-bd {
  align-items: center;
  justify-content: center;
  text-align: center;
  gap: 2vh;
}
.idle-title {
  font: normal 3.6vh/1.2 Georgia, serif;
  color: var(--t);
  margin: 0;
}
.idle-sub {
  font-size: 1.8vh;
  color: var(--tm);
  margin: 0 0 1vh;
}
.choice-row {
  width: 70%;
  max-width: 700px;
}
.choice-card {
  height: 24vh;
}
</style>
