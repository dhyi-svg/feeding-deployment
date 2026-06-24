<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Review the robot's predicted preferences</div>
      </div>
    </div>

    <div class="bd">
      <div v-if="fieldEntries.length === 0" class="waiting-card">
        <p class="eyebrow">Preference Correction</p>
        <h1>Waiting for preference data from the backend...</h1>
      </div>

      <div v-else-if="isSubmitting || isApplied" class="waiting-card">
        <p class="eyebrow">Preference Correction</p>
        <h1 v-if="isApplied">Preferences Applied!</h1>
        <h1 v-else>Sending preferences to robot...</h1>
        <div class="response-box" style="margin-top:2vh">{{ feedbackMessage }}</div>
      </div>

      <template v-else>
        <div class="prg">
          <div
            class="pip"
            v-for="(entry, i) in fieldEntries"
            :key="entry[0]"
            :class="{ a: i === currentIndex, d: i < currentIndex }"
          ></div>
        </div>

        <p class="sc">Preference {{ currentIndex + 1 }} of {{ fieldEntries.length }}</p>

        <div class="pref-body">
          <div class="pref-q">
            <h1 class="pq">{{ formatLabel(currentField) }}</h1>
            <p class="pred">Predicted: <strong>{{ predictedBundle[currentField] }}</strong></p>
            <p class="pref-help">Change it if this doesn't match what you'd like — the robot learns from each correction.</p>
          </div>
          <div class="pref-options">
            <div class="opts">
              <div
                class="oc"
                v-for="option in currentOptions"
                :key="option"
                :class="{ sel: editableBundle[currentField] === option }"
                @click="selectOption(option)"
              >
                <span class="ot">{{ option }}</span>
                <div class="och" v-if="editableBundle[currentField] === option">✓</div>
              </div>
            </div>
            <p class="cdown auto-note">Auto-confirming <em>{{ editableBundle[currentField] }}</em> in <span>{{ countdown }}s</span></p>
          </div>
        </div>
      </template>

      <div v-if="fieldEntries.length > 0 && !isSubmitting && !isApplied" class="footer">
        <button class="btn lg ghost" @click="goBack" :disabled="currentIndex === 0">
          Go Back
        </button>
        <button
          v-if="currentIndex < fieldEntries.length - 1"
          class="btn lg amber"
          @click="goNext"
        >
          Continue
        </button>
        <button
          v-else
          class="btn lg amber"
          @click="submitBundle"
        >
          Confirm All Preferences
        </button>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'

const AUTOCONTINUE_SECONDS = 10

export default {
  name: 'PreferenceCorrection',
  data() {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
      predictedBundle: {},
      editableBundle: {},
      normalizedOptions: {},
      currentIndex: 0,
      isSubmitting: false,
      isApplied: false,
      feedbackMessage: '',
      countdown: AUTOCONTINUE_SECONDS,
      countdownTimer: null
    }
  },
  computed: {
    fieldEntries() {
      return Object.entries(this.editableBundle)
    },
    currentField() {
      return this.fieldEntries[this.currentIndex]?.[0] ?? ''
    },
    currentOptions() {
      return this.normalizedOptions[this.currentField] ?? []
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initSubscriber()
    this.maybeLoadFromUrl()
    // Tell the backend we've mounted and subscribed so it can send the
    // (non-latched) preference_correction_data without racing our subscription.
    // Sent on every (re)connection so a dropped socket re-announces readiness.
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
    formatLabel(field) {
      return field
        .replace(/_/g, ' ')
        .replace(/\b\w/g, (char) => char.toUpperCase())
    },
    selectOption(option) {
      this.editableBundle = { ...this.editableBundle, [this.currentField]: option }
      this.restartCountdown()
    },
    goNext() {
      if (this.currentIndex < this.fieldEntries.length - 1) {
        this.currentIndex++
        this.restartCountdown()
      }
    },
    goBack() {
      if (this.currentIndex > 0) {
        this.currentIndex--
        this.restartCountdown()
      }
    },
    restartCountdown() {
      this.clearCountdownTimer()
      this.countdown = AUTOCONTINUE_SECONDS
      this.countdownTimer = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown--
        } else {
          this.clearCountdownTimer()
          this.advanceFromCountdown()
        }
      }, 1000)
    },
    clearCountdownTimer() {
      if (this.countdownTimer) {
        clearInterval(this.countdownTimer)
        this.countdownTimer = null
      }
    },
    advanceFromCountdown() {
      if (this.currentIndex < this.fieldEntries.length - 1) {
        this.goNext()
      } else {
        this.submitBundle()
      }
    },
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
        data: JSON.stringify({ state: 'preference_correction', status: 'ready' })
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

        if (parsedMessage.state === 'preference_correction_data') {
          this.loadPreferenceData(parsedMessage)
          return
        }

        if (parsedMessage.state === 'preference_correction_applied') {
          this.clearCountdownTimer()
          this.isSubmitting = false
          this.isApplied = true
          this.feedbackMessage = parsedMessage.message || 'Preferences were applied successfully.'
          return
        }

        const route = routeMap[parsedMessage.state]?.[parsedMessage.status]
        if (route) {
          this.$router.push(route)
        }
      } catch (error) {
      }
    },

    maybeLoadFromUrl() {
      if (this.fieldEntries.length > 0) return
      const q = this.$route.query
      if (q.data) {
        try {
          this.loadPreferenceData(JSON.parse(q.data))
          return
        } catch (e) {
        }
      }
      const toList = (v) => String(v ?? '').split(',').map((s) => s.trim()).filter(Boolean)
      const predicted = {}
      const options = {}
      Object.keys(q).forEach((field) => {
        if (field === 'data') return
        const list = toList(q[field])
        if (!list.length) return
        predicted[field] = list[0]
        options[field] = list
      })
      if (Object.keys(predicted).length) {
        this.loadPreferenceData({ predicted_bundle: predicted, options })
      }
    },
    loadPreferenceData(message) {
      const predictedBundle = message.predicted_bundle ?? {}
      const options = message.options ?? {}
      const normalizedOptions = {}

      Object.keys(predictedBundle).forEach((field) => {
        const fieldOptions = Array.isArray(options[field]) ? [...options[field]] : []
        if (!fieldOptions.includes(predictedBundle[field])) {
          fieldOptions.unshift(predictedBundle[field])
        }
        normalizedOptions[field] = fieldOptions
      })

      this.predictedBundle = { ...predictedBundle }
      this.editableBundle = { ...predictedBundle }
      this.normalizedOptions = normalizedOptions
      this.currentIndex = 0
      this.isSubmitting = false
      this.isApplied = false
      this.feedbackMessage = ''
      this.restartCountdown()
    },
    submitBundle() {
      if (!this.publisher) return

      this.clearCountdownTimer()
      this.isSubmitting = true
      this.isApplied = false
      this.feedbackMessage = 'Confirmation sent. Waiting for backend to apply preferences...'

      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'preference_correction_response',
          bundle: this.editableBundle
        })
      })

      this.publisher.publish(message)

      this.$router.push('/robot_executing')
    },
    teardownRos() {
      this.clearCountdownTimer()
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
.pred {
  font-size: 1.9vh;
  color: var(--tm);
  margin-top: 1vh;
}

.pred strong {
  color: var(--a2);
}

.pref-options {
  display: flex;
  flex-direction: column;
  min-height: 0;
}

.opts {
  overflow-y: auto;
  min-height: 0;
  align-content: start;
}

.auto-note {
  text-align: left;
  margin-top: 1.2vh;
  flex-shrink: 0;
}
</style>
