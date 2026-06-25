<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">The robot is working…</div>
      </div>
      <div class="dot"></div>
    </div>

    <div class="ss" v-if="planSlots.current">
      <div class="sk past" v-if="planSlots.last">
        <span class="sl">Previous</span>
        <span class="sv">{{ skillLabel(planSlots.last) }}</span>
      </div>
      <div class="sa" v-if="planSlots.last">&#8594;</div>
      <div class="sk now">
        <span class="sl">Now</span>
        <span class="sv">{{ skillLabel(planSlots.current) }}</span>
      </div>
      <div class="sa" v-if="planSlots.next">&#8594;</div>
      <div class="sk upcoming" v-if="planSlots.next">
        <span class="sl">Next</span>
        <span class="sv">{{ skillLabel(planSlots.next) }}</span>
      </div>
    </div>

    <div class="bd exec-body">
      <div class="icon-ring"><div class="pi">🦾</div></div>
      <div class="exec-text">{{ displayedMessage }}</div>
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
.ss {
  background: var(--s1);
  padding: 1.2vh 2vw;
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 1.2vw;
  border-bottom: 1px solid var(--bd);
  flex-shrink: 0;
}

.sk {
  display: flex;
  flex-direction: column;
  align-items: center;
  min-width: 12vw;
  padding: 1vh 1.2vw;
  border-radius: 10px;
  background: var(--s2);
  color: var(--tm);
  border: 2px solid transparent;
}

.sk.now {
  background: rgba(240, 165, 0, .08);
  color: var(--t);
  border-color: var(--a);
}

.sk .sl {
  font-size: 1.4vh;
  text-transform: uppercase;
  letter-spacing: 1px;
  opacity: 0.8;
}

.sk .sv {
  font-size: 2.2vh;
  font-weight: bold;
  margin-top: 4px;
  color: inherit;
}

.sk.now .sv {
  color: var(--a);
}

.sa {
  font-size: 2.4vh;
  color: var(--tm);
}

.exec-body {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 3vh;
  flex: 1;
}

.icon-ring {
  width: 12vh;
  height: 12vh;
  border-radius: 50%;
  border: 2px solid var(--s3);
  background: rgba(46, 196, 182, .07);
  display: flex;
  align-items: center;
  justify-content: center;
}

.pi {
  font-size: 6vh;
}

.exec-text {
  font-family: Verdana;
  font-size: 3vh;
  color: var(--t);
  text-align: center;
  max-width: 70vw;
  line-height: 1.4;
}
</style>
