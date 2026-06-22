<template>
  <div class="page">
    <header class="topbar">
      <div class="user-block">
        <img class="avatar" alt="User" src="../assets/user_avatar.svg">
        <div>
          <div class="username">{{ username }}</div>
          <div class="subtitle">Review the robot's predicted preferences</div>
        </div>
      </div>
    </header>

    <main class="content">
      
      <div v-if="fieldEntries.length === 0" class="waiting-card">
        <p class="eyebrow">Preference Correction</p>
        <h1>Waiting for preference data from the backend...</h1>
      </div>

      <div v-else-if="isSubmitting || isApplied" class="waiting-card">
        <p class="eyebrow">Preference Correction</p>
        <h1 v-if="isApplied">Preferences Applied!</h1>
        <h1 v-else>Sending preferences to robot...</h1>
        <div class="feedback-banner" :class="feedbackClass">
          <span v-if="isApplied" class="checkmark">✓</span>
          {{ feedbackMessage }}
        </div>
      </div>

      <div v-else class="pref-page">
        
        <div class="progress-bar">
          <div
            class="progress-step"
            v-for="(entry, i) in fieldEntries"
            :key="entry[0]"
            :class="{
              active: i === currentIndex,
              done: i < currentIndex
            }"
          ></div>
        </div>

        <p class="step-count">{{ currentIndex + 1 }} of {{ fieldEntries.length }}</p>

        <h1 class="pref-title">{{ formatLabel(currentField) }}</h1>
        <p class="predicted-label">Robot predicted: <strong>{{ predictedBundle[currentField] }}</strong></p>

        <div class="options-list">
          <div
            class="option-btn"
            v-for="option in currentOptions"
            :key="option"
            :class="{ selected: editableBundle[currentField] === option }"
            @click="selectOption(option)"
          >
            <span class="option-text">{{ option }}</span>
            <div class="option-check" v-if="editableBundle[currentField] === option">✓</div>
          </div>
        </div>
      </div>
    </main>

    <footer v-if="fieldEntries.length > 0 && !isSubmitting && !isApplied" class="footer">
      <button class="nav-btn back-nav" @click="goBack" :disabled="currentIndex === 0">
        Go Back
      </button>
      <button
        v-if="currentIndex < fieldEntries.length - 1"
        class="nav-btn continue-nav"
        @click="goNext"
      >
        Continue
      </button>
      <button
        v-else
        class="nav-btn confirm-nav"
        @click="submitBundle"
      >
        Confirm All Preferences
      </button>
    </footer>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'

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
      feedbackMessage: ''
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
    },
    feedbackClass() {
      if (this.isApplied) return 'success'
      if (this.isSubmitting) return 'pending'
      return ''
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initSubscriber()
    this.maybeLoadFromUrl()
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
    },
    goNext() {
      if (this.currentIndex < this.fieldEntries.length - 1) {
        this.currentIndex++
      }
    },
    goBack() {
      if (this.currentIndex > 0) {
        this.currentIndex--
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
    },
    submitBundle() {
      if (!this.publisher) return

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
    goToTaskSelection() {
      if (this.publisher) {
        const message = new ROSLIB.Message({
          data: JSON.stringify({
            state: 'task_selection',
            status: 'jump'
          })
        })
        this.publisher.publish(message)
      }
      this.$router.push('/task_selection')
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
.page {
  height: 100vh;
  overflow: hidden;
  background:
    radial-gradient(circle at top left, rgba(255, 209, 102, 0.35), transparent 30%),
    linear-gradient(160deg, #fffaf2 0%, #eef6ff 100%);
  color: #1f2937;
  padding: 24px;
  box-sizing: border-box;
  display: flex;
  flex-direction: column;
}

.topbar {
  max-width: 1100px;
  width: 100%;
  margin: 0 auto;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
}

.user-block {
  display: flex;
  align-items: center;
  gap: 14px;
}

.avatar {
  width: 52px;
  height: 52px;
}

.username {
  font-size: 24px;
  font-weight: 700;
}

.subtitle {
  color: #5b6472;
  font-size: 15px;
}

.back-button {
  border: 1px solid #d1d5db;
  border-radius: 999px;
  padding: 14px 22px;
  font-size: 16px;
  font-weight: 700;
  cursor: pointer;
  background: #fff;
  color: #1f2937;
}

.content {
  flex: 1;
  min-height: 0;
  max-width: 1100px;
  width: 100%;
  margin: 0 auto;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 12px 0;
  overflow: hidden;
}

.waiting-card {
  background: rgba(255, 255, 255, 0.88);
  border: 1px solid rgba(15, 23, 42, 0.08);
  border-radius: 24px;
  padding: 40px;
  box-shadow: 0 18px 40px rgba(31, 41, 55, 0.08);
  backdrop-filter: blur(8px);
  width: 100%;
  text-align: center;
}

.waiting-card h1 {
  font-size: clamp(28px, 4vw, 44px);
  margin: 0 0 16px;
}

.eyebrow {
  margin: 0 0 12px;
  color: #b45309;
  font-weight: 700;
  letter-spacing: 0.08em;
  text-transform: uppercase;
  font-size: 14px;
}

.pref-page {
  width: 100%;
  height: 100%;
  min-height: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 12px;
  overflow: hidden;
}

.progress-bar {
  display: flex;
  gap: 10px;
}

.progress-step {
  width: 14px;
  height: 14px;
  border-radius: 999px;
  background: #d1d5db;
  transition: background 0.25s;
}

.progress-step.done {
  background: #10b981;
}

.progress-step.active {
  background: #f59e0b;
  width: 36px;
}

.step-count {
  color: #6b7280;
  font-size: 15px;
  margin: 0;
}

.pref-title {
  font-size: clamp(24px, 3.5vw, 40px);
  font-weight: 800;
  margin: 0;
  text-align: center;
}

.predicted-label {
  color: #6b7280;
  font-size: 15px;
  margin: 0;
}

.options-list {
  display: flex;
  flex-direction: column;
  gap: 10px;
  width: 100%;
  max-width: 700px;
  overflow-y: auto;
  min-height: 0;
}

.option-btn {
  background: #FFE699;
  border-radius: 16px;
  width: 100%;
  min-height: 7vh;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 24px;
  cursor: pointer;
  border: 3px solid transparent;
  transition: border-color 0.2s, background 0.2s, transform 0.1s;
  box-sizing: border-box;
}

.option-btn:hover {
  transform: scale(1.02);
  border-color: #f59e0b;
}

.option-btn.selected {
  background: #f59e0b;
  border-color: #b45309;
}

.option-text {
  font-family: Verdana, sans-serif;
  font-size: clamp(16px, 1.8vw, 22px);
  font-weight: 700;
  color: #1f2937;
}

.option-check {
  font-size: 20px;
  font-weight: 900;
  color: #1f2937;
}

.footer {
  max-width: 1100px;
  width: 100%;
  margin: 0 auto;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
  padding-top: 12px;
}

.nav-btn {
  border: none;
  border-radius: 16px;
  font-family: Verdana, sans-serif;
  font-size: clamp(16px, 1.6vw, 22px);
  font-weight: 700;
  cursor: pointer;
  height: 8vh;
  min-width: 26vw;
  transition: opacity 0.2s, transform 0.1s;
}

.nav-btn:hover {
  transform: scale(1.02);
}

.nav-btn:active {
  transform: scale(0.98);
}

.nav-btn:disabled {
  opacity: 0.35;
  cursor: not-allowed;
  transform: none;
}

.back-nav {
  background: #d9d9d9;
  color: #1f2937;
}

.continue-nav,
.confirm-nav {
  background: #FFE699;
  color: #1f2937;
}

.confirm-nav {
  background: linear-gradient(135deg, #f59e0b, #ef4444);
  color: #fff;
}

.feedback-banner {
  margin-top: 18px;
  padding: 14px 16px;
  border-radius: 14px;
  font-size: 15px;
  font-weight: 600;
  display: flex;
  align-items: center;
  gap: 10px;
}

.feedback-banner.pending {
  background: #eff6ff;
  color: #1d4ed8;
}

.feedback-banner.success {
  background: #ecfdf5;
  color: #047857;
}

.checkmark {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 22px;
  height: 22px;
  border-radius: 999px;
  background: rgba(4, 120, 87, 0.12);
  font-weight: 800;
}

@media (max-width: 720px) {
  .page {
    padding: 16px;
  }

  .topbar,
  .footer {
    flex-direction: column;
    align-items: stretch;
  }

  .user-block {
    justify-content: center;
    text-align: center;
  }

  .nav-btn {
    min-width: unset;
    width: 100%;
    height: 10vh;
  }
}
</style>
