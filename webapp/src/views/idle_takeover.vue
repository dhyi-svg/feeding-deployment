<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Robot is idle — choose what to control</div>
      </div>
    </div>
    <div class="bd">
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
import { ROS_URL, USER } from '@/config/parameterConfig'

export default {
  name: 'IdleTakeover',
  data () {
    return { ros: null, webAppPub: null, username: USER }
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

