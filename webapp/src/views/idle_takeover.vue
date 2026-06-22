<template>
  <div class="idle-takeover">
    <h1>Robot is idle</h1>
    <p>No skill is running. Choose what you want to teleoperate.</p>
    <div class="choices">
      <button class="choice base" @click="takeoverBase">
        Navigation
        <small>drive the base</small>
      </button>
      <button class="choice arm" @click="takeoverArm">
        Manipulation
        <small>move the arm</small>
      </button>
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
.idle-takeover {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  text-align: center;
  padding: 6vh 1rem;
}
.idle-takeover h1 { font-size: 2rem; margin-bottom: .3rem; }
.idle-takeover p { color: #555; margin-bottom: 2rem; }
.choices { display: flex; gap: 2rem; justify-content: center; flex-wrap: wrap; }
.choice {
  width: 240px;
  height: 160px;
  border: none;
  border-radius: 16px;
  color: #fff;
  font-size: 1.6rem;
  font-weight: 700;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: .4rem;
}
.choice small { font-size: 1rem; font-weight: 400; opacity: .9; }
.choice.base { background: #378add; }
.choice.base:active { background: #2f6fb0; }
.choice.arm { background: #ff7a45; }
.choice.arm:active { background: #e8602c; }
</style>
