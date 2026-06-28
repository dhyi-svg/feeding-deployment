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

import { USER } from '@/config/parameterConfig'

export default {
  name: 'IdleTakeover',
  data () {
    return { username: USER }
  },
  methods: {
    takeoverArm () {
      this.$router.push('/manipulation_teleop')
    },
    takeoverBase () {
      
      this.$router.push('/navigation_teleop')
    }
  }
}
</script>
