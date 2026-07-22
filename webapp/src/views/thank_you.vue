<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Meal complete</div>
      </div>
    </div>

    <div class="bd">
      <div class="waiting-card">
        <p class="eyebrow">Meal complete</p>
        <h1 class="thanks">Thank you!</h1>
        <p class="thanks-sub">Your feedback has been saved. See you at the next meal!</p>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'

// Terminal end-of-meal page: no buttons. The backend resends the thank_you
// jump until this page acks with "ready", then idles; a later backend jump
// (e.g. after an operator takeover) can still navigate away via routeMap.
export default {
  name: 'ThankYouPage',
  data() {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initSubscriber()
    // Ack the (non-latched, resent) thank_you jump on every (re)connection.
    this.ros.on('connection', () => this.sendReady())
  },
  beforeUnmount() {
    this.teardownRos()
  },
  beforeRouteLeave(to, from, next) {
    this.teardownRos()
    next()
  },
  methods: {
    initPublisher() {
      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
    sendReady() {
      if (!this.publisher) return
      this.publisher.publish(new ROSLIB.Message({
        data: JSON.stringify({ state: 'thank_you', status: 'ready' })
      }))
    },
    initSubscriber() {
      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })
      this.listener.subscribe((message) => {
        this.handleRosMessage(message)
      })
    },
    handleRosMessage(message) {
      try {
        const parsedMessage = JSON.parse(message.data)

        // Re-ack a resent jump instead of self-pushing via routeMap.
        if (parsedMessage.state === 'thank_you' && parsedMessage.status === 'jump') {
          this.sendReady()
          return
        }

        const route = routeMap[parsedMessage.state]?.[parsedMessage.status]
        if (route) {
          this.$router.push(route)
        }
      } catch (error) {
      }
    },
    teardownRos() {
      if (this.listener) {
        this.listener.unsubscribe()
        if (this.listener.ros) this.listener.ros.close()
        this.listener = null
      }
      if (this.publisher) {
        this.publisher.unadvertise()
        if (this.publisher.ros) this.publisher.ros.close()
        this.publisher = null
      }
    }
  }
}
</script>

<style scoped>
.thanks {
  font-size: 7vh;
}

.thanks-sub {
  margin-top: 2vh;
  font-size: 2.6vh;
  color: var(--tm);
  line-height: 1.5;
}
</style>
