<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Tell us about this meal</div>
      </div>
    </div>

    <div class="bd">
      <div v-if="!hasOptions" class="waiting-card">
        <p class="eyebrow">Meal Context</p>
        <h1>Waiting for meal context options from the backend...</h1>
      </div>

      <template v-else>
        <div class="prg">
          <div
            class="pip"
            v-for="(step, i) in steps"
            :key="step.key"
            :class="{ a: i === currentIndex, d: i < currentIndex }"
          ></div>
        </div>

        <p class="sc">Step {{ currentIndex + 1 }} of {{ steps.length }}</p>

        <div class="pref-body">
          <div class="pref-q">
            <h1 class="pq">{{ currentStep.label }}</h1>
            <p class="pref-help">Select one to continue.</p>
          </div>
          <div class="opts">
            <div
              class="oc"
              v-for="option in currentOptions"
              :key="option"
              :class="{ sel: selection[currentStep.key] === option }"
              @click="selectOption(option)"
            >
              <span class="ot">{{ option }}</span>
              <div class="och" v-if="selection[currentStep.key] === option">✓</div>
            </div>
          </div>
        </div>
      </template>

      <div v-if="hasOptions" class="footer">
        <button class="btn lg ghost" @click="goBack" :disabled="currentIndex === 0">
          Go Back
        </button>
        <button
          v-if="currentIndex < steps.length - 1"
          class="btn lg amber"
          :disabled="!currentSelected"
          @click="goNext"
        >
          Continue
        </button>
        <button
          v-else
          class="btn lg amber"
          :disabled="!isComplete"
          @click="submitSelection"
        >
          Confirm Meal Context
        </button>
      </div>
    </div>
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
.opts {
  overflow-y: auto;
  min-height: 0;
  align-content: start;
}
</style>
