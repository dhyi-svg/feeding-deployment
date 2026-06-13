<template>
  <div id="app">
    <!-- Global Take-Over button: lets the user grab manual control at any time,
         including in the middle of a skill. Hidden on the teleop/resuming pages
         themselves. -->
    <button
      v-if="showTakeOver"
      class="global-takeover"
      @click="takeOver()"
    >Take Over</button>
    <router-view></router-view>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import { ROS_URL } from '@/config/parameterConfig'

export default {
  name: 'app',
  data () {
    return {
      takeoverPublisher: null
    }
  },
  computed: {
    showTakeOver () {
      const p = this.$route.path
      return p !== '/teleop' && p !== '/resuming'
    }
  },
  mounted () {
    const ros = new ROSLIB.Ros({ url: ROS_URL })
    this.takeoverPublisher = new ROSLIB.Topic({
      ros,
      name: '/WebAppComm',
      messageType: 'std_msgs/String'
    })
  },
  methods: {
    takeOver () {
      if (this.takeoverPublisher) {
        this.takeoverPublisher.publish(new ROSLIB.Message({
          data: JSON.stringify({ state: 'teleop', status: 'takeover' })
        }))
      }
      this.$router.push('/teleop')
    }
  }
}
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 0px !important;
}

nav {
  padding: 30px;
}

nav a {
  font-weight: bold;
  color: #2c3e50;
}

nav a.router-link-exact-active {
  color: #42b983;
}

/* Global Take-Over button, fixed top-right above all pages. */
.global-takeover {
  position: fixed;
  top: 12px;
  right: 12px;
  z-index: 1000;
  font-family: Verdana, sans-serif;
  font-size: 14px;
  font-weight: 700;
  padding: 10px 16px;
  border: none;
  border-radius: 8px;
  background: #ff7a45;
  color: #fff;
  cursor: pointer;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.2);
}
.global-takeover:active {
  background: #e8602c;
}
</style>
