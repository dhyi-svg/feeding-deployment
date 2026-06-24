// Classifies a stream of button "clicks" (audio-click transients from the
// switch-button mic) into single vs double presses, purely by timing.
//
// The mono mic can't tell two physical buttons apart, but it sees each press as
// a discrete transient, so we count transients within a window:
//   - one click, no follow-up within DOUBLE_WINDOW   -> single press
//   - two clicks within DOUBLE_WINDOW                 -> double press
//
// Tradeoff: a single press only fires after DOUBLE_WINDOW elapses (we have to
// wait to be sure no second click is coming), whereas a double press fires
// immediately on the second click. Assign the latency-tolerant action to single
// and the time-critical one (e.g. STOP) to double.

export const CLICK_DEBOUNCE = 150 // ms — ignore re-triggers within this of a click (one press's ringing/bounce)
export const DOUBLE_WINDOW = 1000 // ms — max gap between two clicks to count them as a double

export class PressClassifier {
  constructor ({ onSingle, onDouble, onClick, clickDebounce = CLICK_DEBOUNCE, doubleWindow = DOUBLE_WINDOW } = {}) {
    this.onSingle = onSingle
    this.onDouble = onDouble
    this.onClick = onClick // optional: fired for every fresh click (for raw debugging/UI)
    this.clickDebounce = clickDebounce
    this.doubleWindow = doubleWindow
    this._lastClick = 0
    this._pendingSingle = null
  }

  // Call this on every rising edge above the detection threshold, passing the
  // current timestamp (ms). Returns true if it counted as a fresh click.
  edge (now) {
    if (now - this._lastClick <= this.clickDebounce) return false
    this._lastClick = now
    if (this.onClick) this.onClick()
    if (this._pendingSingle) {
      // Second click arrived in time -> it's a double; cancel the pending single.
      clearTimeout(this._pendingSingle)
      this._pendingSingle = null
      if (this.onDouble) this.onDouble()
    } else {
      // First click -> wait to see whether a second one follows.
      this._pendingSingle = setTimeout(() => {
        this._pendingSingle = null
        if (this.onSingle) this.onSingle()
      }, this.doubleWindow)
    }
    return true
  }

  reset () {
    if (this._pendingSingle) {
      clearTimeout(this._pendingSingle)
      this._pendingSingle = null
    }
    this._lastClick = 0
  }
}
