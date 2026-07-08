<template>
  <div class="page" @click="cancelAutocontinue">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">Review the robot's predicted preferences</div>
      </div>
    </div>

    <div class="bd">
      <div v-if="!current" class="waiting-card">
        <p class="eyebrow">Preference Correction</p>
        <h1>{{ waitingMessage }}</h1>
      </div>

      <template v-else>
        <div class="prg">
          <div
            class="pip"
            v-for="i in total"
            :key="i"
            :class="{ a: (i - 1) === step, d: (i - 1) < step }"
          ></div>
        </div>

        <p class="sc">Preference {{ step + 1 }} of {{ total }}</p>

        <div class="pref-body">
          <div class="pref-q">
            <h1 class="pq">{{ current.label }}</h1>
            <p v-if="current.description" class="pref-sub">{{ current.description }}</p>
            <p v-if="current.kind !== 'text'" class="pred">Predicted: <strong>{{ current.predicted }}</strong></p>
            <p class="pref-help">Change it if this doesn't match what you'd like — the robot learns from each correction.</p>
          </div>

          <!-- Free-text dim (e.g. bite ordering): accept the predicted sentence, or write your own. -->
          <div v-if="current.kind === 'text'" class="pref-options">
            <div class="opts">
              <div class="oc" :class="{ sel: !otherMode }" @click="choosePredicted">
                <span class="ot">{{ current.predicted }}</span>
                <div class="och" v-if="!otherMode">✓</div>
              </div>
              <div class="oc" :class="{ sel: otherMode }" @click="chooseOther">
                <span class="ot">Other…</span>
                <div class="och" v-if="otherMode">✓</div>
              </div>
            </div>
            <div v-if="otherMode" class="field-box tall other-box">
              <textarea
                v-model="otherText"
                placeholder="Type your preferred bite ordering..."
                ref="otherTextarea"
                class="field-input"
                @input="userInteracted = true"
              ></textarea>
              <button @click="startOtherSpeech"
                      class="icon-btn"
                      :class="{ 'amber-ic': isRecognizingOther }"
                      :disabled="isRecognizingOther"
                      title="voice">
                <img alt="voice" src="../assets/voice.png">
              </button>
              <button @click="otherText = ''" class="icon-btn" title="clear">
                <img alt="clear" src="../assets/clear.png">
              </button>
            </div>
            <p v-if="!userInteracted" class="cdown auto-note">Auto-confirming the predicted ordering in <span>{{ countdown }}s</span></p>
          </div>

          <!-- Categorical dim: option chips. -->
          <div v-else class="pref-options">
            <div class="opts">
              <div
                class="oc"
                v-for="option in currentOptions"
                :key="option"
                :class="{ sel: selected === option }"
                @click="selectOption(option)"
              >
                <span class="ot">{{ option }}</span>
                <div class="och" v-if="selected === option">✓</div>
              </div>
            </div>
            <p v-if="!userInteracted" class="cdown auto-note">Auto-confirming <em>{{ selected }}</em> in <span>{{ countdown }}s</span></p>
          </div>
        </div>

        <div class="footer">
          <button class="btn lg amber" @click="confirmStep" :disabled="isSubmitting">
            {{ step < total - 1 ? 'Continue' : 'Confirm' }}
          </button>
        </div>
      </template>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import routeMap from '@/router/routeMap'
import { ROS_URL, USER } from '@/config/parameterConfig'

const DEFAULT_AUTOCONTINUE_SECONDS = 10

export default {
  name: 'PreferenceCorrection',
  data() {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
      // Current single-dimension step (null while waiting for the next one).
      current: null,
      selected: '',
      // True once the user clicks an option — cancels auto-confirm so they must
      // explicitly confirm their correction.
      userInteracted: false,
      // Free-text ("text" kind) dim state: otherMode toggles the custom editor,
      // otherText holds the typed/spoken override.
      otherMode: false,
      otherText: '',
      isRecognizingOther: false,
      recognitionOther: null,
      step: 0,
      total: 0,
      autocontinueSeconds: DEFAULT_AUTOCONTINUE_SECONDS,
      isSubmitting: false,
      waitingMessage: 'Waiting for preference data from the backend...',
      countdown: DEFAULT_AUTOCONTINUE_SECONDS,
      countdownTimer: null
    }
  },
  computed: {
    currentOptions() {
      return this.current ? this.current.options : []
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initSubscriber()
    this.maybeLoadFromUrl()
    // Tell the backend we've mounted/subscribed so it can send the (non-latched)
    // step data without racing our subscription. Sent on every (re)connection.
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
    selectOption(option) {
      this.selected = option
      // User has taken over — stop the auto-confirm countdown and require an
      // explicit Continue/Confirm press.
      this.userInteracted = true
      this.clearCountdownTimer()
    },
    restartCountdown() {
      this.clearCountdownTimer()
      this.countdown = this.autocontinueSeconds
      this.countdownTimer = setInterval(() => {
        if (this.countdown > 0) {
          this.countdown--
        } else {
          this.clearCountdownTimer()
          this.confirmStep()
        }
      }, 1000)
    },
    clearCountdownTimer() {
      if (this.countdownTimer) {
        clearInterval(this.countdownTimer)
        this.countdownTimer = null
      }
    },
    cancelAutocontinue() {
      // Any tap on the page cancels the auto-confirm countdown; the user must
      // then press Continue/Confirm explicitly (same effect as selectOption's
      // takeover, without changing the selection).
      this.userInteracted = true
      this.clearCountdownTimer()
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
          this.loadStep(parsedMessage)
          return
        }

        // Stage finished: the backend has all answers for this stage.
        if (parsedMessage.state === 'preference_correction' && parsedMessage.status === 'done') {
          this.clearCountdownTimer()
          this.current = null
          this.isSubmitting = false
          this.$router.push('/robot_executing')
          return
        }

        const route = routeMap[parsedMessage.state]?.[parsedMessage.status]
        if (route) {
          this.$router.push(route)
        }
      } catch (error) {
      }
    },

    loadStep(message) {
      const field = message.field
      const predicted = message.predicted
      const rawOptions = Array.isArray(message.options) ? [...message.options] : []
      // Ensure the robot's prediction is always selectable.
      if (predicted !== undefined && predicted !== null && !rawOptions.includes(predicted)) {
        rawOptions.unshift(predicted)
      }

      this.step = Number.isInteger(message.step) ? message.step : 0
      this.total = Number.isInteger(message.total) ? message.total : 1
      this.autocontinueSeconds = Number.isFinite(message.autocontinue_seconds)
        ? message.autocontinue_seconds
        : DEFAULT_AUTOCONTINUE_SECONDS

      const kind = message.kind || 'categorical'
      this.current = { field, label: message.label || this.formatLabel(field), description: message.description || '', predicted, options: rawOptions, kind }
      this.selected = predicted
      this.userInteracted = false
      this.isSubmitting = false
      // Reset free-text editor state for the new step.
      this.otherMode = false
      this.otherText = ''
      this.stopOtherSpeech()
      this.restartCountdown()
    },
    choosePredicted() {
      this.otherMode = false
      this.selected = this.current.predicted
      this.userInteracted = true
      this.clearCountdownTimer()
    },
    chooseOther() {
      this.otherMode = true
      this.userInteracted = true
      this.clearCountdownTimer()
      // Seed the editor with the prediction so the user can tweak rather than
      // retype; they can clear it to start fresh.
      if (!this.otherText) this.otherText = this.current.predicted || ''
      this.$nextTick(() => {
        if (this.$refs.otherTextarea) this.$refs.otherTextarea.focus()
      })
    },
    startOtherSpeech() {
      if (this.isRecognizingOther) return
      if (!this.recognitionOther) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition
        if (!SpeechRecognition) return
        this.recognitionOther = new SpeechRecognition()
        this.recognitionOther.lang = 'en-US'
        this.recognitionOther.continuous = false
        this.recognitionOther.onresult = (event) => {
          this.otherText += event.results[0][0].transcript
          this.isRecognizingOther = false
        }
        this.recognitionOther.onerror = () => { this.isRecognizingOther = false }
        this.recognitionOther.onend = () => { this.isRecognizingOther = false }
      }
      this.userInteracted = true
      this.isRecognizingOther = true
      this.recognitionOther.start()
    },
    stopOtherSpeech() {
      this.isRecognizingOther = false
      if (this.recognitionOther) {
        try { this.recognitionOther.abort() } catch (e) { /* ignore */ }
      }
    },
    confirmStep() {
      if (!this.publisher || !this.current || this.isSubmitting) return
      this.clearCountdownTimer()
      this.isSubmitting = true
      this.waitingMessage = 'Sending your choice to the robot...'

      // For a free-text dim in "Other" mode, the typed/spoken text is the value;
      // an empty entry falls back to the prediction.
      let value = this.selected
      if (this.current.kind === 'text' && this.otherMode) {
        value = (this.otherText || '').trim() || this.current.predicted
      }
      this.stopOtherSpeech()

      this.publisher.publish(new ROSLIB.Message({
        data: JSON.stringify({
          state: 'preference_correction_response',
          field: this.current.field,
          value
        })
      }))
      // Wait for the next step (or "done"); clear the current dim so the
      // waiting card shows until then.
      this.current = null
    },
    formatLabel(field) {
      return String(field || '')
        .replace(/_/g, ' ')
        .replace(/\b\w/g, (char) => char.toUpperCase())
    },
    maybeLoadFromUrl() {
      const q = this.$route.query
      if (q.data) {
        try {
          this.loadStep(JSON.parse(q.data))
          return
        } catch (e) {
        }
      }
      if (q.field) {
        const toList = (v) => String(v ?? '').split(',').map((s) => s.trim()).filter(Boolean)
        this.loadStep({
          field: q.field,
          label: q.label,
          predicted: q.predicted,
          options: toList(q.options),
          step: q.step ? Number(q.step) : 0,
          total: q.total ? Number(q.total) : 1,
          autocontinue_seconds: q.autocontinue_seconds ? Number(q.autocontinue_seconds) : undefined
        })
      }
    },
    teardownRos() {
      this.clearCountdownTimer()
      this.stopOtherSpeech()
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
.pref-sub {
  font-size: 2.4vh;
  color: var(--tm);
  line-height: 1.5;
  margin-top: 1vh;
}

.pred {
  font-size: 2.3vh;
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

.other-box {
  margin-top: 1.2vh;
  flex-shrink: 0;
}
</style>
