<template>
  <div class="page">
    <header class="topbar">
      <div class="user-block">
        <img class="avatar" alt="User" src="../assets/user_avatar.svg">
        <div>
          <div class="username">{{ username }}</div>
          <div class="subtitle">Tell the robot about this meal before we begin.</div>
        </div>
      </div>
    </header>

    <main class="content">
      
      <div v-if="!hasOptions" class="waiting-card">
        <p class="eyebrow">Meal Context</p>
        <h1>Waiting for meal context options from the backend...</h1>
      </div>

      <div v-else class="pref-page">
        
        <div class="progress-bar">
          <div
            class="progress-step"
            v-for="(step, i) in steps"
            :key="step.key"
            :class="{
              active: i === currentIndex,
              done: i < currentIndex
            }"
          ></div>
        </div>

        <p class="step-count">{{ currentIndex + 1 }} of {{ steps.length }}</p>

        <h1 class="pref-title">{{ currentStep.label }}</h1>

        <div class="options-list" :class="{ 'two-col': currentStep.key === 'meal' || currentStep.key === 'setting' }">
          <div
            class="option-btn"
            v-for="option in currentOptions"
            :key="option"
            :class="{ selected: selection[currentStep.key] === option }"
            @click="selectOption(option)"
          >
            <span class="option-text">{{ option }}</span>
            <div class="option-check" v-if="selection[currentStep.key] === option">✓</div>
          </div>
        </div>
      </div>
    </main>

    <footer v-if="hasOptions" class="footer">
      <button class="nav-btn back-nav" @click="goBack" :disabled="currentIndex === 0">
        Go Back
      </button>
      <button
        v-if="currentIndex < steps.length - 1"
        class="nav-btn continue-nav"
        :disabled="!currentSelected"
        @click="goNext"
      >
        Continue
      </button>
      <button
        v-else
        class="nav-btn confirm-nav"
        :disabled="!isComplete"
        @click="submitSelection"
      >
        Confirm Meal Context
      </button>
    </footer>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'

export default {
  name: 'PreferenceContext',
  data() {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
      steps: [
        { key: 'meal',        label: 'Meal',        optionsKey: 'meals'      },
        { key: 'setting',     label: 'Setting',     optionsKey: 'settings'   },
        { key: 'time_of_day', label: 'Time of Day', optionsKey: 'time_of_day'}
      ],
      currentIndex: 0,
      options: {
        meals: [],
        settings: [],
        time_of_day: []
      },
      defaults: {
        meal: '',
        setting: '',
        time_of_day: ''
      },
      selection: {
        meal: '',
        setting: '',
        time_of_day: ''
      }
    }
  },
  computed: {
    hasOptions() {
      return (
        this.options.meals.length > 0 ||
        this.options.settings.length > 0 ||
        this.options.time_of_day.length > 0
      )
    },
    currentStep() {
      return this.steps[this.currentIndex]
    },
    currentOptions() {
      return this.options[this.currentStep.optionsKey] ?? []
    },
    currentSelected() {
      return Boolean(this.selection[this.currentStep.key])
    },
    isComplete() {
      return Boolean(
        this.selection.meal &&
        this.selection.setting &&
        this.selection.time_of_day
      )
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
    selectOption(option) {
      this.selection = { ...this.selection, [this.currentStep.key]: option }
    },
    goNext() {
      if (this.currentIndex < this.steps.length - 1) {
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

        if (parsedMessage.state === 'preference_context_data') {
          this.loadOptions(parsedMessage)
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
      if (this.hasOptions) return
      const q = this.$route.query
      if (q.data) {
        try {
          this.loadOptions(JSON.parse(q.data))
          return
        } catch (e) {
        }
      }
      const toList = (v) => String(v ?? '').split(',').map((s) => s.trim()).filter(Boolean)
      if (q.meals || q.settings || q.times) {
        this.loadOptions({
          meals: toList(q.meals),
          settings: toList(q.settings),
          time_of_day: toList(q.times),
          defaults: { meal: q.meal || '', setting: q.setting || '', time_of_day: q.time || '' }
        })
      }
    },
    loadOptions(message) {
      const meals = Array.isArray(message.meals) ? message.meals : []
      const settings = Array.isArray(message.settings) ? message.settings : []
      const timeOfDay = Array.isArray(message.time_of_day) ? message.time_of_day : []
      const defaults = message.defaults ?? {}

      this.options = { meals, settings, time_of_day: timeOfDay }

      this.defaults = {
        meal:        defaults.meal        && meals.includes(defaults.meal)               ? defaults.meal        : '',
        setting:     defaults.setting     && settings.includes(defaults.setting)         ? defaults.setting     : '',
        time_of_day: defaults.time_of_day && timeOfDay.includes(defaults.time_of_day)   ? defaults.time_of_day : ''
      }

      this.selection = { ...this.defaults }
      this.currentIndex = 0
    },
    submitSelection() {
      if (!this.publisher || !this.isComplete) return

      const message = new ROSLIB.Message({
        data: JSON.stringify({
          state: 'preference_context_response',
          meal: this.selection.meal,
          setting: this.selection.setting,
          time_of_day: this.selection.time_of_day
        })
      })

      this.publisher.publish(message)
      
      this.$router.push('/preference_correction')
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
    radial-gradient(circle at top right, rgba(110, 231, 183, 0.28), transparent 28%),
    linear-gradient(155deg, #fffef6 0%, #eef7f3 45%, #edf4ff 100%);
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
  background: rgba(255, 255, 255, 0.9);
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
  color: #0f766e;
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
  gap: 16px;
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
  transition: background 0.25s, width 0.25s;
}

.progress-step.done {
  background: #10b981;
}

.progress-step.active {
  background: #0f766e;
  width: 36px;
}

.step-count {
  color: #6b7280;
  font-size: 15px;
  margin: 0;
}

.pref-title {
  font-size: clamp(32px, 5vw, 56px);
  font-weight: 800;
  margin: 0;
  text-align: center;
}

.options-list {
  display: flex;
  flex-direction: column;
  gap: 12px;
  width: 100%;
  max-width: 700px;
  overflow-y: auto;
  min-height: 0;
}

.options-list.two-col {
  display: grid;
  grid-template-columns: 1fr 1fr;
  max-width: 900px;
  overflow-y: auto;
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
  border-color: #0f766e;
}

.option-btn.selected {
  background: #6ee7b7;
  border-color: #0f766e;
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
  padding-top: 16px;
}

.nav-btn {
  border: none;
  border-radius: 20px;
  font-family: Verdana, sans-serif;
  font-size: clamp(20px, 2vw, 28px);
  font-weight: 700;
  cursor: pointer;
  height: 12vh;
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

.continue-nav {
  background: #FFE699;
  color: #1f2937;
}

.confirm-nav {
  background: linear-gradient(135deg, #0f766e, #2563eb);
  color: #fff;
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
