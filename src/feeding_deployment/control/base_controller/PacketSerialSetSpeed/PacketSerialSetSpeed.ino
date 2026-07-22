#include <Arduino.h>
#include <Basicmicro.h>
#include <SoftwareSerial.h>

static const uint32_t PC_BAUD = 115200;

static const uint32_t CONTROLLER_BAUD = 9600;
static const uint32_t TIMEOUT_US = 10000;

static const uint8_t ADDR_A = 128;
static const uint8_t ADDR_B = 128;

// Fresh-command watchdog:
// if no valid A=... B=... line arrives within this many ms,
// force commanded speeds back to zero.
static const uint32_t CMD_STALE_MS = 5000;

// Encoder polling (v7): one "E <millis> <a1> <a2> <b1> <b2> <okA> <okB>" line
// per poll. Polls run only on loop iterations that neither sent a speed
// command nor have host bytes pending: SoftwareSerial RX masks interrupts for
// ~1 ms per byte at 9600 baud, which drops USB RX bytes arriving at the same
// time (the 328P USART has a ~3-byte hardware FIFO), so we poll only while the
// host link is quiet. A controller that keeps failing (e.g. motor power off)
// is retried at 1 Hz instead of 10 Hz -- a dead read burns ~48 ms in library
// retries.
static const uint32_t ENC_POLL_MS = 100;
static const uint32_t ENC_BACKOFF_MS = 1000;
static const uint8_t ENC_FAIL_BACKOFF = 5;
// Hard ceiling: under a continuously-changing command stream every loop
// iteration takes the (~70 ms healthy) send branch, so the "quiet iteration"
// poll gate below would never fire and E-lines would stop for the whole
// maneuver -- exactly when odometry matters, and it would also silence the
// host's echo-confirm (which is gated on fresh encoder data). Force a poll
// once we have gone this long without one, accepting the small extra
// command-mangling risk (the host repairs it via echo-confirm).
static const uint32_t ENC_POLL_MAX_MS = 300;

// Command-send backoff: with an unpowered/dead controller every SpeedM1 call
// burns ~100 ms in library retries, and because lastA/lastB only advance on
// SUCCESS the un-backed-off loop retried failed sends on every iteration --
// starving encoder polling entirely and mangling inbound host lines (each
// SoftwareSerial TX byte masks interrupts ~1 ms). After SEND_FAIL_BACKOFF
// consecutive failures a controller's pending setpoint is retried only every
// SEND_BACKOFF_MS (single attempt): delivery within ~1 s of the controller
// coming back, zero change in behavior while controllers are healthy.
static const uint32_t SEND_BACKOFF_MS = 1000;
static const uint8_t SEND_FAIL_BACKOFF = 3;

// Controller A: TX=11, RX=10
static const uint8_t RX_A = 10, TX_A = 11;
SoftwareSerial serialA(RX_A, TX_A);
Basicmicro ctrlA(&serialA, TIMEOUT_US);

// Controller B: TX=9, RX=8
static const uint8_t RX_B = 8, TX_B = 9;
SoftwareSerial serialB(RX_B, TX_B);
Basicmicro ctrlB(&serialB, TIMEOUT_US);

static int32_t speedA = 0, speedB = 0;
static int32_t lastA = INT32_MIN, lastB = INT32_MIN;
static uint32_t lastValidCmdMs = 0;

// Encoder state: last values read, per-side validity of the LAST poll cycle
// (ok=0 also while a side is backed off -- values printed then are stale),
// and consecutive-failure counters driving the backoff.
static uint32_t encA1 = 0, encA2 = 0, encB1 = 0, encB2 = 0;
static bool encOkA = false, encOkB = false;
static uint8_t encFailA = 0, encFailB = 0;
static uint32_t lastEncPollMs = 0;
static uint32_t lastEncTryA = 0, lastEncTryB = 0;

// Consecutive send-failure counters driving the command-send backoff, and the
// last setpoint we ATTEMPTED to deliver per side (distinct from lastA/lastB,
// which track the last SUCCESSFUL send). A changed target clears the backoff
// so a NEW command -- crucially a stop -- is attempted immediately, instead of
// waiting out a backoff that a still-executing-but-unacked controller earned
// while running the OLD speed.
static uint8_t sendFailA = 0, sendFailB = 0;
static uint32_t lastSendTryA = 0, lastSendTryB = 0;
static int32_t attemptA = INT32_MIN, attemptB = INT32_MIN;

bool sendSpeedToController(Basicmicro &ctrl, SoftwareSerial &ss, uint8_t addr, int32_t speed, int attempts = 3) {
  for (int attempt = 0; attempt < attempts; attempt++) {
    ss.listen();
    if (!ctrl.SpeedM1(addr, speed)) {
      Serial.print("WARN SpeedM1 failed addr="); Serial.print(addr);
      Serial.print(" speed="); Serial.print(speed);
      Serial.print(" attempt="); Serial.println(attempt);
      delay(5);
      continue;
    }
    delay(5);
    ss.listen();
    if (!ctrl.SpeedM2(addr, speed)) {
      Serial.print("WARN SpeedM2 failed addr="); Serial.print(addr);
      Serial.print(" speed="); Serial.print(speed);
      Serial.print(" attempt="); Serial.println(attempt);
      delay(5);
      continue;
    }
    return true;
  }
  Serial.print("ERROR send failed addr="); Serial.print(addr);
  Serial.print(" speed="); Serial.println(speed);
  return false;
}

bool parseLineAB(const char *line, int32_t &outA, int32_t &outB) {
  long a, b;
  char extra;
  if (sscanf(line, "A=%ld B=%ld %c", &a, &b, &extra) == 2) {
    outA = (int32_t)a;
    outB = (int32_t)b;
    return true;
  }
  else {
    Serial.print("WARN unparseable line: '"); Serial.print(line); Serial.println("'");
  }
  return false;
}

// Read one full line from Serial
bool readLine(char *buf, size_t buflen) {
  static size_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      idx = 0;
      return true;
    }
    if (idx < buflen - 1) buf[idx++] = c;
    else idx = 0; // overflow reset
  }
  return false;
}

static bool retryDue(uint8_t fails, uint8_t failThreshold,
                     uint32_t lastTry, uint32_t backoffMs, uint32_t now) {
  if (fails < failThreshold) return true;
  return (uint32_t)(now - lastTry) >= backoffMs;
}

void pollEncoders(uint32_t now) {
  if (retryDue(encFailA, ENC_FAIL_BACKOFF, lastEncTryA, ENC_BACKOFF_MS, now)) {
    lastEncTryA = now;
    serialA.listen();
    encOkA = ctrlA.ReadEncoders(ADDR_A, encA1, encA2);
    if (encOkA) encFailA = 0;
    else if (encFailA < 255) encFailA++;
  } else {
    encOkA = false; // backed off this cycle; printed values are stale
  }

  if (retryDue(encFailB, ENC_FAIL_BACKOFF, lastEncTryB, ENC_BACKOFF_MS, now)) {
    lastEncTryB = now;
    serialB.listen();
    encOkB = ctrlB.ReadEncoders(ADDR_B, encB1, encB2);
    if (encOkB) encFailB = 0;
    else if (encFailB < 255) encFailB++;
  } else {
    encOkB = false;
  }

  // One line per poll, printed strictly AFTER both SoftwareSerial
  // transactions (never interleave USB TX with an SS read). Stamp is taken
  // AFTER the reads, not at poll entry: ReadEncoders can spend tens of ms in
  // internal retries on a noisy link, and the host differentiates counts over
  // this stamp -- a pre-read stamp would inflate dt and corrupt that tick's
  // twist.
  uint32_t stamp = millis();
  Serial.print(F("E "));
  Serial.print(stamp);
  Serial.print(' ');
  Serial.print(encA1);
  Serial.print(' ');
  Serial.print(encA2);
  Serial.print(' ');
  Serial.print(encB1);
  Serial.print(' ');
  Serial.print(encB2);
  Serial.print(' ');
  Serial.print(encOkA ? 1 : 0);
  Serial.print(' ');
  Serial.println(encOkB ? 1 : 0);
}

void setup() {
  Serial.begin(PC_BAUD);
  while (!Serial && millis() < 3000) {}

  serialA.begin(CONTROLLER_BAUD);
  serialB.begin(CONTROLLER_BAUD);
  ctrlA.begin(CONTROLLER_BAUD);
  ctrlB.begin(CONTROLLER_BAUD);

  delay(200);

  // stop both
  sendSpeedToController(ctrlA, serialA, ADDR_A, 0);
  delay(10);
  sendSpeedToController(ctrlB, serialB, ADDR_B, 0);

  lastValidCmdMs = millis();

  Serial.println("Ready v7 enc. Send: A=<int> B=<int>");
}

void loop() {
  // Drain ALL pending lines and keep only the latest valid one.
  char line[64];
  int32_t newA = speedA, newB = speedB;
  bool got = false;

  while (readLine(line, sizeof(line))) {
    if (parseLineAB(line, newA, newB)) {
      got = true;
    }
  }

  if (got) {
    speedA = newA;
    speedB = newB;
    lastValidCmdMs = millis();

    Serial.print("Parsed A="); Serial.print(speedA);
    Serial.print(" B="); Serial.println(speedB);
  }

  // Fresh-command watchdog:
  // if command stream goes stale, force zero velocities.
  if ((uint32_t)(millis() - lastValidCmdMs) > CMD_STALE_MS) {
    speedA = 0;
    speedB = 0;
  }

  // Only send when changed; only update last if send succeeded (so failed
  // sends are retried) -- but back retries off after repeated failures (see
  // SEND_BACKOFF_MS above), else a dead controller starves the whole loop.
  bool sentThisIter = false;
  // A changed target bypasses the backoff for its first attempt (see above).
  if (speedA != attemptA) { sendFailA = 0; attemptA = speedA; }
  if (speedB != attemptB) { sendFailB = 0; attemptB = speedB; }
  uint32_t nowSend = millis();
  if (speedA != lastA &&
      retryDue(sendFailA, SEND_FAIL_BACKOFF, lastSendTryA, SEND_BACKOFF_MS, nowSend)) {
    sentThisIter = true;
    lastSendTryA = nowSend;
    if (sendSpeedToController(ctrlA, serialA, ADDR_A, speedA,
                              sendFailA >= SEND_FAIL_BACKOFF ? 1 : 3)) {
      lastA = speedA;
      sendFailA = 0;
    } else if (sendFailA < 255) {
      sendFailA++;
    }
  }
  nowSend = millis();
  if (speedB != lastB &&
      retryDue(sendFailB, SEND_FAIL_BACKOFF, lastSendTryB, SEND_BACKOFF_MS, nowSend)) {
    sentThisIter = true;
    lastSendTryB = nowSend;
    delay(15); // spacing helps SoftwareSerial
    if (sendSpeedToController(ctrlB, serialB, ADDR_B, speedB,
                              sendFailB >= SEND_FAIL_BACKOFF ? 1 : 3)) {
      lastB = speedB;
      sendFailB = 0;
    } else if (sendFailB < 255) {
      sendFailB++;
    }
  }

  // Encoder poll. Preferred path: a quiet iteration at the ENC_POLL_MS cadence
  // (deferral, not skipping -- lastEncPollMs only advances on an actual poll,
  // so a busy iteration delays the poll by ~2 ms). Fallback path: force a poll
  // once ENC_POLL_MAX_MS has elapsed regardless of send/RX activity, so a
  // sustained command stream can't starve odometry (see ENC_POLL_MAX_MS).
  uint32_t now = millis();
  uint32_t sinceEnc = now - lastEncPollMs;
  bool quiet = !sentThisIter && Serial.available() == 0;
  if ((quiet && sinceEnc >= ENC_POLL_MS) || sinceEnc >= ENC_POLL_MAX_MS) {
    lastEncPollMs = now;
    pollEncoders(now);
  }

  delay(2);
}
