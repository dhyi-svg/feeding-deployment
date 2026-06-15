<template>
  <div id="app">
    <!-- Global manual-control buttons: let the user grab control at any time,
         including in the middle of a skill. Hidden on the teleop/resuming pages.
         Anchored as a group whose right edge clears the page's Finish Feeding
         button; the flex gap keeps the two from colliding with each other. -->
    <div v-if="showTakeOver" class="global-controls">
      <!-- Base control: from the task-selection menu, or while the robot is
           autonomously driving (the executive enables it during navigation). -->
      <button
        v-if="onTaskSelection || baseControlEnabled"
        class="global-btn base"
        @click="controlBase()"
      >Robot Base Control</button>
      <button class="global-btn arm" @click="controlArm()">Robot Arm Control</button>
    </div>
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
      takeoverPublisher: null,
      serverListener: null,
      // Set by the executive while the base is autonomously driving, so the
      // user can take over mid-drive (Robot Base Control is otherwise menu-only).
      baseControlEnabled: false
    }
  },
  computed: {
    showTakeOver () {
      const p = this.$route.path
      return p !== '/manipulation_teleop' && p !== '/navigation_teleop' && p !== '/resuming'
    },
    onTaskSelection () {
      return this.$route.path === '/task_selection'
    }
  },
  mounted () {
    const ros = new ROSLIB.Ros({ url: ROS_URL })
    this.takeoverPublisher = new ROSLIB.Topic({
      ros,
      name: '/WebAppComm',
      messageType: 'std_msgs/String'
    })
    // Listen for base-control availability from the executive (set during navigation).
    this.serverListener = new ROSLIB.Topic({
      ros,
      name: '/ServerComm',
      messageType: 'std_msgs/String'
    })
    this.serverListener.subscribe((msg) => {
      try {
        const parsed = JSON.parse(msg.data)
        if (parsed.state === 'base_control') {
          this.baseControlEnabled = parsed.status === 'enabled'
        }
      } catch (e) { /* ignore non-JSON */ }
    })
  },
  methods: {
    controlArm () {
      // Signal a mid-skill manipulation takeover, then open the arm teleop page.
      if (this.takeoverPublisher) {
        this.takeoverPublisher.publish(new ROSLIB.Message({
          data: JSON.stringify({ state: 'teleop', status: 'takeover' })
        }))
      }
      this.$router.push('/manipulation_teleop')
    },
    controlBase () {
      // The navigation page itself announces the base takeover on mount.
      this.$router.push('/navigation_teleop')
    }
  }
}
</script>

<style>
/* Reset the default body margin so full-height pages (height: 100vh, e.g. the
   teleop screens) fit the viewport exactly without scrolling. */
html, body {
  margin: 0;
  padding: 0;
}

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

/* Global control buttons, fixed top-right as a group. The group's RIGHT edge is
   pinned at right:240px so it always clears the page's ~220px-wide "Finish
   Feeding" button (which occupies the rightmost ~225px); the buttons grow
   leftward and the 10px gap keeps them apart. */
.global-controls {
  position: fixed;
  top: 14px;
  right: 240px;
  z-index: 1000;
  display: flex;
  gap: 10px;
}
.global-btn {
  height: 50px;
  font-family: Verdana, sans-serif;
  font-size: 14px;
  font-weight: 700;
  padding: 0 16px;
  border: none;
  border-radius: 8px;
  color: #fff;
  cursor: pointer;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.2);
  white-space: nowrap;
}
.global-btn.arm { background: #ff7a45; }
.global-btn.arm:active { background: #e8602c; }
.global-btn.base { background: #378add; }
.global-btn.base:active { background: #2f6fb0; }
</style>
