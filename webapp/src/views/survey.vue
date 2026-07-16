<template>
  <div class="page">
    <div class="tb">
      <div class="av"><img src="../assets/user_avatar.svg" alt="User"></div>
      <div>
        <div class="tb-n">{{ username }}</div>
        <div class="tb-s">A few quick questions about today's meal</div>
      </div>
    </div>

    <div class="bd">
      <div v-if="!current" class="waiting-card">
        <p class="eyebrow">End-of-Meal Survey</p>
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

        <p class="sc">Question {{ step + 1 }} of {{ total }}</p>

        <div class="survey-body">
          <div class="survey-q">
            <h1 class="pq">{{ current.title }}</h1>
            <p class="survey-sub">{{ current.question }}</p>
          </div>

          <!-- Open-ended question: type or dictate an answer (may be left empty). -->
          <div v-if="current.kind === 'text'" class="survey-answer">
            <div class="field-box tall">
              <textarea
                v-model="answerText"
                placeholder="Type or use the microphone to answer..."
                ref="answerTextarea"
                class="field-input"
              ></textarea>
              <button @click="startSpeech"
                      class="icon-btn"
                      :class="{ 'amber-ic': isRecognizing }"
                      :disabled="isRecognizing"
                      title="voice">
                <img alt="voice" src="../assets/voice.png">
              </button>
              <button @click="answerText = ''" class="icon-btn" title="clear">
                <img alt="clear" src="../assets/clear.png">
              </button>
            </div>
            <p class="voice-status" :class="{ empty: !voiceStatus }" aria-live="polite">
              <img alt="" src="../assets/voice.png">
              <span v-if="voiceStatus">{{ voiceStatus }}</span>
              <span v-else>&nbsp;</span>
            </p>
          </div>

          <!-- Likert question: pick a number, then Continue. -->
          <div v-else class="survey-answer">
            <div class="likert">
              <button
                class="lk"
                v-for="n in scaleNumbers"
                :key="n"
                :class="{ sel: selected === n }"
                @click="selected = n"
              >{{ n }}</button>
            </div>
            <div class="likert-anchors">
              <span>{{ current.scaleMin }} &ndash; {{ current.minLabel }}</span>
              <span>{{ current.scaleMax }} &ndash; {{ current.maxLabel }}</span>
            </div>
          </div>
        </div>

        <div class="footer">
          <button class="btn lg amber" :disabled="!canSubmit" @click="confirmStep">
            {{ step < total - 1 ? 'Continue' : 'Finish' }}
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

export default {
  name: 'SurveyPage',
  data() {
    return {
      ros: null,
      username: USER,
      listener: null,
      publisher: null,
      // Current question (null while waiting for the next one).
      current: null,
      selected: null,
      answerText: '',
      isRecognizing: false,
      recognition: null,
      voiceStatus: '',
      // The question just answered; the backend resends the current question
      // every second until it processes our response, so without this a resend
      // arriving in that gap would re-show the answered question.
      lastAnswered: null,
      step: 0,
      total: 9,
      isSubmitting: false,
      waitingMessage: 'Waiting for the first question...'
    }
  },
  computed: {
    scaleNumbers() {
      if (!this.current) return []
      const nums = []
      for (let n = this.current.scaleMin; n <= this.current.scaleMax; n++) nums.push(n)
      return nums
    },
    canSubmit() {
      if (!this.current || this.isSubmitting) return false
      // Likert answers are mandatory; the open-ended question may be left
      // empty ("What, if anything, ...").
      return this.current.kind === 'text' || this.selected !== null
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL })
    this.initPublisher()
    this.initSubscriber()
    this.maybeLoadFromUrl()
    // Tell the backend we've mounted/subscribed so it can send the (non-latched)
    // question data without racing our subscription. Sent on every (re)connection.
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
        data: JSON.stringify({ state: 'survey', status: 'ready' })
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

        if (parsedMessage.state === 'survey_data') {
          // The backend resends the current question every second (reload
          // robustness); ignore duplicates so an in-progress or just-sent
          // answer isn't reset. (The loaded question's step lives on
          // this.step, not inside this.current.)
          if (this.current && this.current.field === parsedMessage.field && this.step === parsedMessage.step) return
          if (this.lastAnswered && this.lastAnswered.field === parsedMessage.field && this.lastAnswered.step === parsedMessage.step) return
          this.loadStep(parsedMessage)
          return
        }

        // Jump received while already mounted (the backend's resend beat our
        // ready): just re-ack; falling through to routeMap would self-push.
        if (parsedMessage.state === 'survey' && parsedMessage.status === 'jump') {
          this.sendReady()
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
      this.step = Number.isInteger(message.step) ? message.step : 0
      this.total = Number.isInteger(message.total) ? message.total : 1
      this.current = {
        field: message.field,
        title: message.title || '',
        question: message.question || '',
        kind: message.kind === 'text' ? 'text' : 'likert',
        scaleMin: Number.isInteger(message.scale_min) ? message.scale_min : 1,
        scaleMax: Number.isInteger(message.scale_max) ? message.scale_max : 7,
        minLabel: message.min_label || 'Very Low',
        maxLabel: message.max_label || 'Very High'
      }
      this.selected = null
      this.answerText = ''
      this.isSubmitting = false
      this.stopSpeech()
    },
    releaseTakeoverMic() {
      // App.vue holds a persistent mic stream (physical-button detection);
      // SpeechRecognition can't start while it's open. Ask App.vue to release
      // it and wait briefly (same handshake as adaptability/transparency).
      return new Promise((resolve) => {
        let settled = false
        const done = () => {
          if (settled) return
          settled = true
          resolve()
        }
        window.dispatchEvent(new CustomEvent('release-takeover-mic', {
          detail: { done }
        }))
        setTimeout(done, 300)
      })
    },
    async startSpeech() {
      if (this.isRecognizing) return

      if (this.$refs.answerTextarea) {
        this.$refs.answerTextarea.blur()
      }

      await this.releaseTakeoverMic()

      const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition
      if (!SpeechRecognition) {
        this.voiceStatus = 'speech recognition not supported in this browser'
        return
      }

      if (!this.recognition) {
        this.recognition = new SpeechRecognition()
        this.recognition.lang = 'en-US'
        this.recognition.continuous = false
        this.recognition.onstart = () => {
          this.voiceStatus = 'listening...'
        }
        this.recognition.onresult = (event) => {
          this.answerText += event.results[0][0].transcript
          this.isRecognizing = false
          this.voiceStatus = ''
        }
        this.recognition.onerror = (event) => {
          this.isRecognizing = false
          this.voiceStatus = 'error: ' + (event.error || 'unknown') +
            (event.message ? ' - ' + event.message : '')
        }
        this.recognition.onend = () => {
          this.isRecognizing = false
          if (this.voiceStatus === 'listening...') this.voiceStatus = 'no speech captured'
        }
      }

      this.isRecognizing = true
      this.voiceStatus = 'starting...'
      try {
        this.recognition.start()
      } catch (e) {
        this.isRecognizing = false
        this.voiceStatus = 'start failed: ' + (e && e.message ? e.message : e)
      }
    },
    stopSpeech() {
      this.isRecognizing = false
      this.voiceStatus = ''
      if (this.recognition) {
        try { this.recognition.abort() } catch (e) { /* ignore */ }
      }
    },
    confirmStep() {
      if (!this.publisher || !this.current || !this.canSubmit) return
      this.isSubmitting = true
      this.waitingMessage = 'Saving your answer...'

      const value = this.current.kind === 'text'
        ? (this.answerText || '').trim()
        : this.selected
      this.stopSpeech()

      this.publisher.publish(new ROSLIB.Message({
        data: JSON.stringify({
          state: 'survey_response',
          field: this.current.field,
          value,
          user_action: 'tap'
        })
      }))
      // Wait for the next question (or the thank_you jump); clear the current
      // question so the waiting card shows until then.
      this.lastAnswered = { field: this.current.field, step: this.step }
      this.current = null
    },
    maybeLoadFromUrl() {
      // ROS-free styling aid: /#/survey?data=<url-encoded JSON survey_data msg>
      const q = this.$route.query
      if (q.data) {
        try {
          this.loadStep(JSON.parse(q.data))
        } catch (e) {
        }
      }
    },
    teardownRos() {
      this.stopSpeech()
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
.survey-sub {
  font-size: 2.6vh;
  color: var(--tm);
  line-height: 1.5;
  margin-top: 1vh;
}

.survey-body {
  display: flex;
  flex-direction: column;
  min-height: 0;
  flex: 1;
}

.survey-answer {
  margin-top: 3vh;
}

.likert {
  display: flex;
  gap: 1.2vh;
}

.lk {
  flex: 1;
  height: 11vh;
  font-size: 3.6vh;
  font-weight: 600;
  font-family: inherit;
  border-radius: 1.5vh;
  background: var(--s2);
  border: 2px solid var(--s3);
  color: var(--t);
  cursor: pointer;
}

.lk.sel {
  border-color: var(--a);
  background: rgba(240, 165, 0, .12);
}

.likert-anchors {
  display: flex;
  justify-content: space-between;
  margin-top: 1.5vh;
  font-size: 3.2vh;
  font-weight: 600;
  color: var(--tm);
}
</style>
