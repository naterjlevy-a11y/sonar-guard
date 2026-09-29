/*
  SONAR GUARD
  A sonar turret that learns a room, patrols it with smooth sweeps,
  detects anything new, tracks it, and demands an access code.
  Works with dashboard/index.html (open in Chrome or Edge).

  WIRING:
    Ultrasonic Trig -> pin 3, Echo -> pin 4
    Servo signal    -> pin 13
    Buzzer +        -> pin 6
    IR receiver Y   -> pin 2
    Button          -> A1
    LCD             -> pins 7, 8, 9, 10, 11, 12

  REMOTE:  PLAY = arm   0-9 = code (default 1234)   |<< = delete digit
  WEBSITE: Arm button   keypad or keyboard for the code
  BUTTON:  tap = arm

  Needs the IRremote library (Armin Joachimsmeyer) for TinyIRReceiver.
*/

#define IR_RECEIVE_PIN 2
#define NO_LED_FEEDBACK_CODE
#include "TinyIRReceiver.hpp"
#include <Servo.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
Servo turret;

// Pins
const byte TRIG_PIN   = 3;
const byte ECHO_PIN   = 4;
const byte BUZZER_PIN = 6;
const byte SERVO_PIN  = 13;
const byte BUTTON_PIN = A1;

// Settings
const char PIN_CODE[] = "1234";
const int  MIN_ANGLE = 10;
const int  MAX_ANGLE = 170;
const int  BIN_DEG   = 4;
const int  N_BINS    = (MAX_ANGLE - MIN_ANGLE) / BIN_DEG + 1;
const unsigned int SWEEP_TICK_MS = 22;
const unsigned int TRACK_TICK_MS = 18;
const unsigned int PING_MS = 30;
const int  LAG_DEG = 2;
const byte CAL_SWEEPS = 4;
const int  DETECT_RANGE_CM = 250;
const int  MIN_DIFF_CM = 20;
const byte NOISE_MULT = 3;
const unsigned long ENTRY_MS = 20000;
const byte EXIT_DELAY_S = 5;
const byte MAX_WRONG = 3;
const int  TRACK_HALF = 30;
const unsigned long ECHO_MAX_US = 25000;
const byte MAX_SAMPLES = 6;

// States
enum { ST_DISARMED, ST_ARMING, ST_LEARNING, ST_ARMED, ST_ALERT, ST_ALARM };
const char* const STATE_NAMES[] = {"DISARMED", "ARMING", "LEARNING", "ARMED", "ALERT", "ALARM"};
byte state = ST_DISARMED;

// Sweep
bool sweeping = false;
int  lo = MIN_ANGLE, hi = MAX_ANGLE;
int  cmdAngle = 90;
int  sweepDir = 1;
unsigned int servoTick = SWEEP_TICK_MS;
unsigned long nextServoAt = 0;
unsigned long sweepStartedAt = 0, lastSweepMs = 0;
unsigned long sweepCount = 0;
unsigned int sessionSweeps = 0;

// Sonar
volatile unsigned long echoRise = 0;
volatile unsigned long echoWidth = 0;
volatile bool echoReady = false;
bool pingInFlight = false;
unsigned long pingSentAt = 0;
int  pingAngle = 90;
unsigned long pingCount = 0;

// Binning
int  curBin = -1;
int  samples[MAX_SAMPLES];
byte nSamples = 0;
int  sweepVal[N_BINS];
byte stampOf[N_BINS];
bool anomOf[N_BINS];
int  lastFinal = -1;
int  lastAngle = 90, lastCm = -1;

// Learning
int  calVals[CAL_SWEEPS][N_BINS];
int  baseline[N_BINS];
byte noise[N_BINS];
byte calSweep = 0;
unsigned long learnStartedAt = 0, learnPings0 = 0;

// Target and alert
int  targetAngle = 90, targetCm = 0, targetWidth = 0;
byte lostPasses = 0;
unsigned long alertDeadline = 0, lastAlertPing = 0;
unsigned long armAt = 0, armedAt = 0;
byte lastCount = 255;
unsigned int detections = 0, wrongTotal = 0;

// Code entry
char pinBuf[5];
byte pinLen = 0;
byte wrongTries = 0;

// Display
unsigned long lastLcd = 0, msgUntil = 0;
char msg0[17], msg1[17];

// Button
bool buttonWasDown = false;
unsigned long lastButtonChange = 0;

// Sound queue
struct Note { unsigned int f; unsigned int ms; };
Note noteQ[8];
byte qHead = 0, qLen = 0;
unsigned long noteEnd = 0;
bool sirenHigh = false;


// Echo pin interrupt: times the ultrasonic pulse in the background
ISR(PCINT2_vect) {
  unsigned long t = micros();
  if (PIND & _BV(4)) {
    echoRise = t;
  } else if (echoRise) {
    echoWidth = t - echoRise;
    echoRise = 0;
    echoReady = true;
  }
}


void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.begin(115200);

  PCICR  |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT20);

  initPCIInterruptForTinyReceiver();

  turret.attach(SERVO_PIN, 500, 2400);
  turret.write(cmdAngle);

  lcd.begin(16, 2);
  Serial.println(F("READY"));
  setState(ST_DISARMED);
  queueTone(880, 80);
  queueTone(1319, 120);
}

void loop() {
  unsigned long now = millis();
  handleIR();
  serialInput();
  handleButton();
  servoUpdate(now);
  pingUpdate(now);
  stateUpdate(now);
  soundUpdate(now);
  if (now - lastLcd >= 150) {
    lastLcd = now;
    drawScreen(now);
  }
}


// ===== STATES =====

void setState(byte s) {
  state = s;
  Serial.print(F("STATE,"));
  Serial.println(STATE_NAMES[s]);
  lastLcd = 0;
}

void startArming() {
  pinLen = 0;
  wrongTries = 0;
  wrongTotal = 0;
  detections = 0;
  sessionSweeps = 0;
  armedAt = 0;
  sweeping = false;
  cmdAngle = MIN_ANGLE;
  sweepDir = 1;
  turret.write(cmdAngle);
  armAt = millis() + EXIT_DELAY_S * 1000UL;
  lastCount = 255;
  setState(ST_ARMING);
}

void startLearning() {
  for (byte p = 0; p < CAL_SWEEPS; p++) {
    for (int b = 0; b < N_BINS; b++) calVals[p][b] = -2;
  }
  calSweep = 0;
  lo = MIN_ANGLE;
  hi = MAX_ANGLE;
  servoTick = SWEEP_TICK_MS;
  curBin = -1;
  nSamples = 0;
  lastFinal = -1;
  learnStartedAt = millis();
  learnPings0 = pingCount;
  sweepStartedAt = millis();
  sweeping = true;
  setState(ST_LEARNING);
}

void finishLearning() {
  int covered = 0, maxNoise = 0;
  long noiseSum = 0;
  int nearB = -1, farB = -1;

  for (int b = 0; b < N_BINS; b++) {
    int vals[CAL_SWEEPS];
    byte nv = 0, measured = 0;
    for (byte p = 0; p < CAL_SWEEPS; p++) {
      int v = calVals[p][b];
      if (v == -2) continue;
      measured++;
      if (v >= 0) vals[nv++] = v;
    }
    if (measured == 0 || nv * 2 < measured) {
      baseline[b] = -1;
      noise[b] = 0;
    } else {
      sortInts(vals, nv);
      baseline[b] = vals[nv / 2];
      noise[b] = min(255, (vals[nv - 1] - vals[0] + 1) / 2);
      covered++;
      noiseSum += noise[b];
      if (noise[b] > maxNoise) maxNoise = noise[b];
      if (nearB < 0 || baseline[b] < baseline[nearB]) nearB = b;
      if (farB < 0 || baseline[b] > baseline[farB]) farB = b;
    }
    anomOf[b] = false;
    Serial.print(F("BASE,"));
    Serial.print(binAngle(b));
    Serial.print(',');
    Serial.print(baseline[b]);
    Serial.print(',');
    Serial.println(thrFor(b));
  }

  int coverage = covered * 100 / N_BINS;
  int avgNoise10 = covered ? (int)(noiseSum * 10 / covered) : 0;
  unsigned long learnMs = millis() - learnStartedAt;
  int pingHz = learnMs ? (int)((pingCount - learnPings0) * 1000UL / learnMs) : 0;

  Serial.print(F("CALSTATS,"));
  Serial.print(coverage);                          Serial.print(',');
  Serial.print(avgNoise10);                        Serial.print(',');
  Serial.print(maxNoise);                          Serial.print(',');
  Serial.print(nearB >= 0 ? binAngle(nearB) : 0);  Serial.print(',');
  Serial.print(nearB >= 0 ? baseline[nearB] : 0);  Serial.print(',');
  Serial.print(farB >= 0 ? binAngle(farB) : 0);    Serial.print(',');
  Serial.print(farB >= 0 ? baseline[farB] : 0);    Serial.print(',');
  Serial.print(lastSweepMs);                       Serial.print(',');
  Serial.println(pingHz);

  armedAt = millis();
  setState(ST_ARMED);
  snprintf(msg0, sizeof(msg0), "ROOM LEARNED");
  snprintf(msg1, sizeof(msg1), "cov %d%% +-%d.%dcm", coverage, avgNoise10 / 10, avgNoise10 % 10);
  msgUntil = millis() + 3000;
  queueTone(1319, 80);
  queueTone(1760, 120);
}

void startAlert(int angle, int cm, int width) {
  detections++;
  targetAngle = angle;
  targetCm = cm;
  targetWidth = width;
  alertDeadline = millis() + ENTRY_MS;
  pinLen = 0;
  lostPasses = 0;
  lastAlertPing = 0;
  setState(ST_ALERT);
  sendTarget('D');
  setTrackWindow();
}

void triggerAlarm() {
  setState(ST_ALARM);
  noteEnd = 0;
}

void disarm() {
  noTone(BUZZER_PIN);
  qLen = 0;
  noteEnd = 0;
  unsigned long armedSec = armedAt ? (millis() - armedAt) / 1000 : 0;
  Serial.print(F("SESSION,"));
  Serial.print(armedSec);      Serial.print(',');
  Serial.print(sessionSweeps); Serial.print(',');
  Serial.print(detections);    Serial.print(',');
  Serial.println(wrongTotal);

  sweeping = false;
  lo = MIN_ANGLE;
  hi = MAX_ANGLE;
  servoTick = SWEEP_TICK_MS;
  curBin = -1;
  cmdAngle = 90;
  turret.write(cmdAngle);
  pinLen = 0;
  wrongTries = 0;
  setState(ST_DISARMED);
  snprintf(msg0, sizeof(msg0), " ACCESS GRANTED");
  snprintf(msg1, sizeof(msg1), "  PLAY = arm");
  msgUntil = millis() + 3000;
  queueTone(1047, 90);
  queueTone(1319, 90);
  queueTone(1568, 90);
  queueTone(2093, 160);
}

void stateUpdate(unsigned long now) {
  if (state == ST_ARMING) {
    if (now >= armAt) {
      startLearning();
      return;
    }
    byte left = (armAt - now + 999) / 1000;
    if (left != lastCount) {
      lastCount = left;
      queueTone(1000, 60);
    }
  } else if (state == ST_ALERT) {
    if (now >= alertDeadline) {
      triggerAlarm();
      return;
    }
    if (now - lastAlertPing >= 800) {
      lastAlertPing = now;
      queueTone(map(constrain(targetCm, 20, 250), 20, 250, 2200, 900), 70);
    }
  }
}


// ===== SMOOTH SWEEP =====

void servoUpdate(unsigned long now) {
  if (!sweeping || now < nextServoAt) return;
  nextServoAt = now + servoTick;

  if (cmdAngle > hi) { sweepDir = -1; cmdAngle--; turret.write(cmdAngle); return; }
  if (cmdAngle < lo) { sweepDir = 1;  cmdAngle++; turret.write(cmdAngle); return; }

  int next = cmdAngle + sweepDir;
  if (next > hi || next < lo) {
    onSweepEnd();
    if (!sweeping) return;
    if (cmdAngle < lo)      sweepDir = 1;
    else if (cmdAngle > hi) sweepDir = -1;
    else                    sweepDir = -sweepDir;
    next = cmdAngle + sweepDir;
  }
  cmdAngle = next;
  turret.write(cmdAngle);
}

void onSweepEnd() {
  if (curBin >= 0) {
    finalizeBin(curBin);
    curBin = -1;
    nSamples = 0;
  }
  unsigned long now = millis();
  lastSweepMs = now - sweepStartedAt;
  sweepStartedAt = now;

  if (state == ST_LEARNING) {
    calSweep++;
    if (calSweep >= CAL_SWEEPS) finishLearning();
  } else if (state == ST_ALERT || state == ST_ALARM) {
    evaluateTrack();
  }
  if (state >= ST_ARMED) sessionSweeps++;
  sweepCount++;
  lastFinal = -1;
}


// ===== SONAR =====

void pingUpdate(unsigned long now) {
  if (echoReady) {
    noInterrupts();
    unsigned long w = echoWidth;
    echoReady = false;
    interrupts();
    pingInFlight = false;
    onReading(pingAngle, (w < 116 || w > ECHO_MAX_US) ? -1 : (int)(w / 58));
  } else if (pingInFlight && now - pingSentAt > 35) {
    pingInFlight = false;
    noInterrupts();
    echoRise = 0;
    interrupts();
    onReading(pingAngle, -1);
  }

  if (sweeping && !pingInFlight && now - pingSentAt >= PING_MS && !(PIND & _BV(4))) {
    noInterrupts();
    echoRise = 0;
    interrupts();
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    pingSentAt = now;
    pingInFlight = true;
    pingAngle = constrain(cmdAngle - sweepDir * LAG_DEG, MIN_ANGLE, MAX_ANGLE);
    pingCount++;
  }
}

void onReading(int angle, int cm) {
  int b = binFor(angle);
  if (b != curBin) {
    if (curBin >= 0) finalizeBin(curBin);
    curBin = b;
    nSamples = 0;
  }
  if (nSamples < MAX_SAMPLES) samples[nSamples++] = cm;
}

void finalizeBin(int b) {
  if (nSamples == 0) return;
  sortInts(samples, nSamples);
  int v = samples[nSamples / 2];
  sweepVal[b] = v;
  stampOf[b] = (byte)sweepCount;
  lastAngle = binAngle(b);
  lastCm = v;

  Serial.print(F("S,"));
  Serial.print(lastAngle);
  Serial.print(',');
  Serial.println(v);

  if (state == ST_LEARNING) {
    if (calSweep < CAL_SWEEPS) calVals[calSweep][b] = v;
    tick();
  } else if (state >= ST_ARMED) {
    anomOf[b] = isAnomaly(b, v);
    if (state == ST_ARMED) {
      tick();
      if (anomOf[b] && lastFinal >= 0 && abs(lastFinal - b) == 1 &&
          anomOf[lastFinal] && stampOf[lastFinal] == (byte)sweepCount) {
        int first, last, a, cm, w;
        findCluster(b, first, last);
        clusterStats(first, last, a, cm, w);
        lastFinal = b;
        startAlert(a, cm, w);
        return;
      }
    }
  }
  lastFinal = b;
}


// ===== DETECTION MATH =====

int binFor(int angle) {
  return constrain((angle - MIN_ANGLE + BIN_DEG / 2) / BIN_DEG, 0, N_BINS - 1);
}

int binAngle(int b) {
  return MIN_ANGLE + b * BIN_DEG;
}

int thrFor(int b) {
  if (baseline[b] < 0) return 0;
  int t = max(MIN_DIFF_CM, NOISE_MULT * noise[b]);
  return max(t, baseline[b] / 10);
}

bool isAnomaly(int b, int v) {
  if (v < 0 || v > DETECT_RANGE_CM) return false;
  if (baseline[b] < 0) return true;
  return (baseline[b] - v) > thrFor(b);
}

void findCluster(int seed, int &first, int &last) {
  first = seed;
  last = seed;
  while (first > 0 && anomOf[first - 1] && stampOf[first - 1] == (byte)sweepCount) first--;
  while (last < N_BINS - 1 && anomOf[last + 1] && stampOf[last + 1] == (byte)sweepCount) last++;
}

void clusterStats(int first, int last, int &angle, int &cm, int &width) {
  cm = 9999;
  for (int b = first; b <= last; b++) {
    if (sweepVal[b] >= 0 && sweepVal[b] < cm) cm = sweepVal[b];
  }
  if (cm == 9999) cm = 0;
  angle = (binAngle(first) + binAngle(last)) / 2;
  float span = (last - first + 1) * BIN_DEG;
  width = (int)(2.0 * cm * tan(radians(span / 2.0)));
}

void setTrackWindow() {
  lo = max(MIN_ANGLE, targetAngle - TRACK_HALF);
  hi = min(MAX_ANGLE, targetAngle + TRACK_HALF);
  servoTick = TRACK_TICK_MS;
}

void evaluateTrack() {
  int bLo = binFor(lo), bHi = binFor(hi);
  int bestFirst = -1, bestLast = -1, bestDist = 9999;
  for (int b = bLo; b <= bHi; b++) {
    if (anomOf[b] && stampOf[b] == (byte)sweepCount) {
      int f = b;
      while (b + 1 <= bHi && anomOf[b + 1] && stampOf[b + 1] == (byte)sweepCount) b++;
      int center = (binAngle(f) + binAngle(b)) / 2;
      int d = abs(center - targetAngle);
      if (d < bestDist) {
        bestDist = d;
        bestFirst = f;
        bestLast = b;
      }
    }
  }
  if (bestFirst >= 0) {
    clusterStats(bestFirst, bestLast, targetAngle, targetCm, targetWidth);
    lostPasses = 0;
    sendTarget('T');
    setTrackWindow();
  } else if (++lostPasses == 3) {
    Serial.println(F("LOST"));
    lo = MIN_ANGLE;
    hi = MAX_ANGLE;
    servoTick = SWEEP_TICK_MS;
  }
}

void sendTarget(char kind) {
  Serial.print(kind);
  Serial.print(',');
  Serial.print(targetAngle);
  Serial.print(',');
  Serial.print(targetCm);
  Serial.print(',');
  Serial.println(targetWidth);
}

void sortInts(int *a, byte n) {
  for (byte i = 1; i < n; i++) {
    int v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = v;
  }
}


// ===== CODE ENTRY AND INPUTS =====

int remoteDigit(uint8_t cmd) {
  const uint8_t codes[10] = {0x16, 0x0C, 0x18, 0x5E, 0x08, 0x1C, 0x5A, 0x42, 0x52, 0x4A};
  for (byte i = 0; i < 10; i++) {
    if (codes[i] == cmd) return i;
  }
  return -1;
}

void enterDigit(byte d) {
  if (state == ST_DISARMED) return;
  pinBuf[pinLen++] = '0' + d;
  queueTone(1800, 40);
  Serial.print(F("PIN,"));
  Serial.println(pinLen);
  lastLcd = 0;
  if (pinLen == 4) {
    pinBuf[4] = 0;
    if (strcmp(pinBuf, PIN_CODE) == 0) disarm();
    else wrongCode();
  }
}

void backspace() {
  if (pinLen == 0) return;
  pinLen--;
  queueTone(900, 30);
  Serial.print(F("PIN,"));
  Serial.println(pinLen);
  lastLcd = 0;
}

void wrongCode() {
  pinLen = 0;
  wrongTries++;
  wrongTotal++;
  Serial.print(F("WRONG,"));
  Serial.println(wrongTries);
  snprintf(msg0, sizeof(msg0), " ACCESS DENIED");
  snprintf(msg1, sizeof(msg1), " tries left: %d", max(0, MAX_WRONG - wrongTries));
  msgUntil = millis() + 1200;
  queueTone(220, 400);
  if (wrongTries >= MAX_WRONG && state != ST_ALARM) triggerAlarm();
}

void handleIR() {
  if (!TinyReceiverDecode()) return;
  if (TinyIRReceiverData.Flags & IRDATA_FLAGS_IS_REPEAT) return;
  uint8_t cmd = TinyIRReceiverData.Command;
  int d = remoteDigit(cmd);
  if (d >= 0) {
    enterDigit(d);
    return;
  }
  if (cmd == 0x40 && state == ST_DISARMED) startArming();
  else if (cmd == 0x44) backspace();
}

void serialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '9') enterDigit(c - '0');
    else if ((c == 'A' || c == 'a') && state == ST_DISARMED) startArming();
    else if (c == '<') backspace();
  }
}

void handleButton() {
  bool down = digitalRead(BUTTON_PIN) == LOW;
  if (down != buttonWasDown && millis() - lastButtonChange > 30) {
    lastButtonChange = millis();
    buttonWasDown = down;
    if (down && state == ST_DISARMED) startArming();
  }
}


// ===== DISPLAY =====

void lcdLine(byte row, const char* text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void drawScreen(unsigned long now) {
  if (now < msgUntil) {
    lcdLine(0, msg0);
    lcdLine(1, msg1);
    return;
  }
  char r0[17], r1[17], code[5];
  for (byte i = 0; i < 4; i++) code[i] = (i < pinLen) ? '*' : '_';
  code[4] = 0;

  switch (state) {
    case ST_DISARMED:
      strcpy(r0, "GUARD DISARMED");
      strcpy(r1, "PLAY = arm");
      break;

    case ST_ARMING:
      snprintf(r0, sizeof(r0), "ARMING IN %d", (int)((armAt - now + 999) / 1000));
      strcpy(r1, "Clear the area");
      break;

    case ST_LEARNING: {
      long range = MAX_ANGLE - MIN_ANGLE;
      long pos = (sweepDir > 0) ? cmdAngle - MIN_ANGLE : MAX_ANGLE - cmdAngle;
      int pct = (int)((calSweep * range + pos) * 100 / (CAL_SWEEPS * range));
      snprintf(r0, sizeof(r0), "LEARNING  %3d%%", pct);
      snprintf(r1, sizeof(r1), "sweep %d/%d  %3d%c", calSweep + 1, CAL_SWEEPS, cmdAngle, (char)223);
      break;
    }

    case ST_ARMED:
      snprintf(r0, sizeof(r0), "ARMED  sweep %u", sessionSweeps);
      if (pinLen)          snprintf(r1, sizeof(r1), "CODE %s", code);
      else if (lastCm < 0) snprintf(r1, sizeof(r1), "%3d%c   clear", lastAngle, (char)223);
      else                 snprintf(r1, sizeof(r1), "%3d%c   %3d cm", lastAngle, (char)223, lastCm);
      break;

    default: {
      if ((now / 500) % 2) strcpy(r0, state == ST_ALARM ? " !! INTRUDER !!" : "OBJECT DETECTED!");
      else snprintf(r0, sizeof(r0), "TGT %3d%c %3dcm", targetAngle, (char)223, targetCm);
      if (state == ST_ALERT) {
        long left = ((long)(alertDeadline - now) + 999) / 1000;
        snprintf(r1, sizeof(r1), "CODE %s  %2lds", code, left);
      } else {
        snprintf(r1, sizeof(r1), "CODE %s  ALARM", code);
      }
    }
  }
  lcdLine(0, r0);
  lcdLine(1, r1);
}


// ===== SOUND =====

void queueTone(unsigned int f, unsigned int ms) {
  if (qLen < 8) {
    byte i = (qHead + qLen) % 8;
    noteQ[i].f = f;
    noteQ[i].ms = ms;
    qLen++;
  }
}

void soundUpdate(unsigned long now) {
  if (state == ST_ALARM) {
    if (now >= noteEnd) {
      sirenHigh = !sirenHigh;
      tone(BUZZER_PIN, sirenHigh ? 1400 : 900);
      noteEnd = now + 220;
    }
    return;
  }
  if (now < noteEnd || qLen == 0) return;
  unsigned int f = noteQ[qHead].f;
  unsigned int ms = noteQ[qHead].ms;
  qHead = (qHead + 1) % 8;
  qLen--;
  tone(BUZZER_PIN, f, ms);
  noteEnd = now + ms + 20;
}

void tick() {
  if (qLen == 0 && millis() >= noteEnd) tone(BUZZER_PIN, 2600, 3);
}
