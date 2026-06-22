<template>
  <div class="top">
    <div class="left">
      <img class="user" alt="User" src="../assets/user_avatar.svg">
      <div class = "usertext">
        <div class="username">{{ username }}</div>
        <div class = "userslog">Enjoy your mealtime now!</div>
      </div>
    </div>
  </div>

  <div class="skill-plan" v-if="planSlots.current">
    <div class="skill-step past" v-if="planSlots.last">
      <span class="step-label">Previous</span>
      <span class="step-name">{{ skillLabel(planSlots.last) }}</span>
    </div>
    <div class="skill-arrow" v-if="planSlots.last">&#8594;</div>
    <div class="skill-step current">
      <span class="step-label">Now</span>
      <span class="step-name">{{ skillLabel(planSlots.current) }}</span>
    </div>
    <div class="skill-arrow" v-if="planSlots.next">&#8594;</div>
    <div class="skill-step upcoming" v-if="planSlots.next">
      <span class="step-label">Next</span>
      <span class="step-name">{{ skillLabel(planSlots.next) }}</span>
    </div>
  </div>

  <div class="content">
    <div class="message">
      {{ displayedMessage }}
    </div>
  </div>

</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';
import { skillLabel } from '@/config/skillLabels';

export default {
  data () {
    return {
      ros: null,
      username: USER,
      displayedMessage: '',
      listener: null,
      skillPlanListener: null,
      skillPlan: [],
      currentSkillIndex: -1
    }
  },
  computed: {

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
    this.initSubscriber()
    this.initPublisher()

    if (this.$route.query.plan) {
      this.skillPlan = String(this.$route.query.plan).split(',')
      this.currentSkillIndex = this.$route.query.current != null ? parseInt(this.$route.query.current, 10) : 0
    }
  },
  beforeRouteLeave (to, from, next) {
    if (this.listener) {
      this.listener.unsubscribe();
      this.listener = null;
    }

    if (this.skillPlanListener) {
      this.skillPlanListener.unsubscribe();
      this.skillPlanListener = null;
    }

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
        const route = routeMap[parsedMessage.state]?.[parsedMessage.status];
        if (!route && parsedMessage.status) {
          this.displayedMessage = parsedMessage.status;
        }
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
    },

  }
}
</script>

<style scoped>
.top {
  height: 9vh;
  background: #eee;
  display: flex;
  align-items: unset;
  justify-content: space-between;
  padding: 5px;
  margin-bottom: 5px;
  .right {
    display: flex;
    justify-content: center;
    align-items: center;
  }
  .left {
    display: flex;
    justify-content: space-between;
    padding:15px
  }
  .usertext{
    align-items: baseline;
    display: flex;
    justify-content: center;
    flex-flow: column;
    margin-left: 5px;
  }
  .username{
    font-family: Verdana;
    font-size: 20px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
  .userslog{
    font-family: Verdana;
    font-size: 16px;
    font-weight: 400;
    line-height: 18px;
    letter-spacing: 0.17499999701976776px;
    text-align: left;
  }
}

.left {
  display: flex;
  align-items: center;
}

.right {
  display: flex;
  align-items: center;
}


.skill-plan {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 12px;
  padding: 10px 10px;
}

.skill-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  min-width: 140px;
  padding: 10px 18px;
  border-radius: 10px;
  background: #eee;
  color: #6e7e8e;
  border: 2px solid transparent;
}

.skill-step.current {
  background: #6e7e8e;
  color: white;
  border-color: #3d4a57;
  transform: scale(1.08);
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.15);
}

.skill-step .step-label {
  font-family: Verdana;
  font-size: 12px;
  text-transform: uppercase;
  letter-spacing: 1px;
  opacity: 0.8;
}

.skill-step .step-name {
  font-family: Verdana;
  font-size: 18px;
  font-weight: bold;
  margin-top: 4px;
}

.skill-arrow {
  font-size: 26px;
  color: #6e7e8e;
}

.content {
  display: flex;
  justify-content: center;
  align-items: center;
  
  height: 60vh;
}

.message {
  font-weight: bold;
  font-family: Verdana;
  font-size: 40px;
  letter-spacing: 0.17499999701976776px;
  text-align: center;
}


</style>
