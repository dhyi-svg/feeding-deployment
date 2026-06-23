<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Ready to start your meal</div>
      </div>
      <div class="dot"></div>
    </div>

    <div class="bd home-bd">
      <div class="home-left">
        <div class="h-robot">🤖</div>
        <div class="h-greet">Ready when <em>you are.</em></div>
        <div class="h-sub">Your robot is connected and ready to begin.</div>
      </div>
      <div class="home-right">
        <button class="btn xl amber" style="width:60%" @click="handleButtonClick">Start Meal</button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER } from '@/config/parameterConfig';

export default {
  data () {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initSubscriber()
    this.initPublisher()
  },
  beforeRouteLeave (to, from, next) {
    if (this.listener) {
      this.listener.unsubscribe();
      this.listener = null;
    }
    if (this.publisher) {
      this.publisher.unadvertise();
      this.publisher = null;
    }
    next();
  },
  methods: {
    handleRosMessage (message) {
      try {
        const parsedMessage = JSON.parse(message.data);
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (route) {
          this.$router.push(route);
        }
      } catch (error) {
      }
    },
    handleButtonClick () {
      this.publishMessage();
      this.$router.push('/preference_context');
    },
    initPublisher () {
      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot',
        messageType: 'std_msgs/String'
      })
    },
    publishMessage () {
      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'home',
          status: 'move_to_above_plate'
        })
      })
      this.publisher.publish(message);
    },
    initSubscriber () {
      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp',
        messageType: 'std_msgs/String'
      })
      this.listener.subscribe((message) => {
        this.handleRosMessage(message);
      });
    }
  }
}
</script>

<style scoped>
.home-bd {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 4vw;
  align-items: center;
  flex: 1;
}
.home-left {
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
}
.h-robot {
  width: 8vh;
  height: 8vh;
  border-radius: 50%;
  background: var(--s2);
  border: 2px solid var(--s3);
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 4vh;
}
.h-greet {
  font: normal 4.5vh/1.2 Georgia, serif;
  color: var(--t);
}
.h-greet em {
  font-style: normal;
  color: var(--a);
}
.h-sub {
  font-size: 2vh;
  color: var(--tm);
  line-height: 1.5;
}
.home-right {
  display: flex;
  align-items: center;
  justify-content: center;
}
</style>
