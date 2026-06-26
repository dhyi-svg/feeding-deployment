// Detects button presses from a stream of audio-click transients (from the
// switch-button mic). The mic hears a click on press AND a click on release
// (two transients, with the signal quiet during the hold), so we fire on the
// first edge and then debounce long enough to swallow the release click (and
// any bounce) of one physical press — however long it's held.
//
// There is no longer any single/double distinction: every press fires
// immediately (no waiting on a follow-up click).

// ms — after a press, ignore further clicks for this long, so press + release
// of one physical press collapse into a single event. Matches the 2 s debounce
// the Python audio button (safety/button.py) uses for the same reason.
export const CLICK_DEBOUNCE = 2000

export class PressDetector {
  constructor ({ onPress, clickDebounce = CLICK_DEBOUNCE } = {}) {
    this.onPress = onPress
    this.clickDebounce = clickDebounce
    this._lastClick = 0
  }

  // Call this on every rising edge above the detection threshold, passing the
  // current timestamp (ms). Returns true if it counted as a fresh press.
  edge (now) {
    if (now - this._lastClick <= this.clickDebounce) return false
    this._lastClick = now
    if (this.onPress) this.onPress()
    return true
  }

  reset () {
    this._lastClick = 0
  }
}
