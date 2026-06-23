<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Ready to transfer?</div>
      </div>
    </div>

    <div class="bd">
      <div class="simple-confirm">
        <p>The robot has picked up the mouth wipe.<br>Click 'Continue' to wipe your mouth when ready.</p>
        <button class="btn lg amber" style="min-width:24vw" @click="redirectToChangeItem()">Continue</button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib';
import routeMap from '@/router/routeMap';
import { ROS_URL, USER} from '@/config/parameterConfig';

export default {
  data () {
    return {
      ros: null,
      username: USER,
      publisher: null 
    }
  },
  mounted () {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher(); 
    this.initSubscriber()
  },
  methods: {
    initSubscriber() {

      this.listener = new ROSLIB.Topic({
        ros: this.ros,
        name: '/robot_to_webapp', 
        messageType: 'std_msgs/String' 
      })

      this.listener.subscribe((msg) => {

        try {
          const parsedData = JSON.parse(msg.data);
          if (parsedData.state === 'prepare_mouth_wiping' && parsedData.status === 'completed') {
            this.$router.push('/wipe_confirm_transfer'); 
          }
          const route = routeMap[parsedData.state]?.[parsedData.status];
          if (route) {
            if (typeof route === 'string') {
              this.$router.push(route); 
            } else if (typeof route === 'object') {
              this.$router.push(route); 
            }
          }
        } catch (error) {
        }
      })
    },
    initPublisher() {

      this.publisher = new ROSLIB.Topic({
        ros: this.ros,
        name: '/webapp_to_robot', 
        messageType: 'std_msgs/String' 
      });

    },

    publishReturnToMain() {
      if (this.publisher) {
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'wipe_confirm_transfer',
            status: 'confirm'
          })
        });
        this.publisher.publish(message);
      } else {
      }
    },
    redirectToChangeItem() {
      this.publishReturnToMain(); 
      this.$router.push('/robot_executing'); 
    }
  }
}
</script>


