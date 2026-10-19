#include <Arduino.h>
#include <Servo.h>

// =====================================================================
//  WATER TREATMENT STATION  -  PID flow control + Virtual Terminal
//
//  Terminal (9600 baud, 8N1):  Arduino TX(D1) -> Terminal RXD
//                              Arduino RX(D0) <- Terminal TXD
//
//  MODES (press 1 or 2 in the terminal; no other terminal commands):
//    1 = SIMULATION : tank level AND flow are both simulated in software
//                     (flow setpoint changes automatically in steps)
//    2 = PROTEUS    : tank level is read from the HC-SR04 (D A2/A5) and
//                     flow from the flow-sensor pulses on D2; you change
//                     both by hand inside Proteus
//
//  ALL SETTINGS (setpoint, PID gains, auto steps...) are in the
//  "USER SETTINGS" block below - edit them here and re-upload.
//
//  Oscilloscope outputs (PWM -> RC filter -> scope):
//    D3  = measured flow     (0..5 V  =  0..SCOPE_FULL_SCALE_LPM)
//    D11 = flow setpoint     (same scale)
//    D5  = pump-2 drive      (copy of the pump-2 PWM, 0..5 V = 0..255)
// =====================================================================

// ================= USER SETTINGS (edit here) =================
// Flow setpoint (L/min), used in PROTEUS mode, and in SIM mode when auto is off
const float SETPOINT_LPM = 20.0;

// PID gains
const float KP_INIT = 2.0;    // PWM counts per (L/min) of error
const float KI_INIT = 3.0;    // PWM counts per (L/min * s)
const float KD_INIT = 0.0;    // PWM counts per (L/min / s)

// Automatic setpoint steps (SIM mode only)
const bool  AUTO_SETPOINT_ENABLED = true;
const unsigned long AUTO_STEP_MS  = 10000UL;                  // time per step
const float AUTO_SP[] = { 15.0, 30.0, 20.0, 40.0, 25.0 };     // L/min
const byte  AUTO_N = 5;                                       // entries in AUTO_SP

// ================= MODES =================
const byte MODE_NONE    = 0;
const byte MODE_SIM     = 1;
const byte MODE_PROTEUS = 2;
byte mode = MODE_NONE;

// ================= PINS =================
const int PH_PIN       = A0;
const int TURB_PIN     = A1;

const int TRIG_PIN     = A2;
const int ECHO_PIN     = A5;

const int FLOW_PIN     = 2;

// Oscilloscope outputs (PWM pins on Timer2, not touched by Servo)
const int SCOPE_FLOW_PIN = 3;
const int SCOPE_SP_PIN   = 11;
const int SCOPE_PUMP_PIN = 5;     // PWM copy of pump-2 drive (free PWM pin)

// Pump 1 - L298 (fills the tank)
const int PUMP1_EN     = 4;       // plain ON/OFF now (was D5 PWM)
const int PUMP1_IN1    = 7;
const int PUMP1_IN2    = 8;

// Pump 2 - L298 (empties the tank) - speed is PID controlled
const int PUMP2_EN     = 6;
const int PUMP2_IN3    = 10;
const int PUMP2_IN4    = A4;

// Servo = 3-way diverter valve
const int SERVO_PIN    = 9;

// ON/OFF switch (moved from D0 to D12 because D0/D1 are now the serial port)
const int SWITCH_PIN      = 12;
const int SWITCH_ON_LEVEL = LOW;   // closed to ground = ON

// ================= WATER QUALITY SETTINGS =================
const float PH_MIN = 6.5;
const float PH_MAX = 8.5;

const int   PH_SAMPLES = 20;
const float PH_ALPHA   = 0.2;
float smoothedPH = -1.0;

const int   TURB_SAMPLES = 20;
const float TURB_ALPHA   = 0.2;
const float TURBIDITY_MAX_NTU = 5.0;
float smoothedNTU = -1.0;

// ================= LEVEL SETTINGS =================
const int LEVEL_START_PUMP1 = 30;
const int LEVEL_STOP_PUMP1  = 75;
const int LEVEL_MIN_PUMP2   = 25;
const int LEVEL_OVERFLOW    = 90;

const float TANK_HEIGHT_CM = 400.0;

// ================= PID / FLOW SETTINGS =================
float Kp = KP_INIT;
float Ki = KI_INIT;
float Kd = KD_INIT;

const float SP_MIN = 5.0;     // L/min
const float SP_MAX = 45.0;    // L/min
float setpoint = SETPOINT_LPM;

const int PUMP2_PWM_MIN = 40;   // below this the pump does not turn
const int PUMP2_PWM_MAX = 255;

// Used only for the start-up feed-forward guess (L/min at full PWM)
const float FF_MAX_LPM = 60.0;

const float MIN_FLOW = 10.0;                 // low-flow alarm threshold (L/min)
const unsigned long FLOW_ALARM_MS = 4000;    // pump saturated + flow low for this long

float pidI = 0.0;
float pidPrevMeas = 0.0;
float pidDFilt = 0.0;
int   pump2Pwm = 0;

// Diverter valve positions
const int VALVE_CLOSED = 0;
const int VALVE_OPEN   = 90;
const int VALVE_WASTE  = 180;

// ================= BAD WATER FLUSH SETTINGS =================
const unsigned long BAD_WATER_DELAY_MS = 3000;
const int  FLUSH_END_LEVEL = 10;
const int  MAX_FLUSHES     = 3;

// ================= SIMULATION SETTINGS =================
float simLevel = 20.0;                    // tank level (%)
const float SIM_FILL_PER_S      = 4.0;    // % per second while pump 1 runs
const float SIM_TANK_CAPACITY_L = 100.0;  // virtual tank volume (L)
unsigned long lastSimMillis = 0;

// Simulated pump-2 / pipe model (first-order lag)
const float SIM_PUMP_MAX_LPM = 60.0;      // flow at full PWM, full tank
const float SIM_PUMP_TAU_S   = 1.0;       // time constant
float simFlow = 0.0;                      // true simulated flow (L/min)
unsigned long lastPlantMillis = 0;

// Automatic setpoint state (SIM mode)
bool autoSetpoint = AUTO_SETPOINT_ENABLED;
byte autoIdx = 0;
unsigned long lastAutoStep = 0;

// ================= FLOW SENSOR =================
const float PULSES_PER_LITRE = 450.0;
const unsigned long FLOW_WINDOW_SIM_MS     = 100;
const unsigned long FLOW_WINDOW_PROTEUS_MS = 250;
const float FLOW_ALPHA_SIM     = 0.6;
const float FLOW_ALPHA_PROTEUS = 0.5;

// Oscilloscope scale: 5 V = this many L/min
const float SCOPE_FULL_SCALE_LPM = 50.0;

volatile unsigned long pulseCount = 0;

float flowLpm = 0.0;
float flowDt  = 0.1;                      // seconds between flow samples
unsigned long lastFlowMillis = 0;

// ================= ULTRASONIC =================
float distanceCm = -1.0;
bool levelSensorValid = true;
unsigned long lastLevelMillis = 0;

// ================= OBJECTS =================
Servo valveServo;

// ================= SYSTEM STATE =================
bool systemRunning = false;
bool pump1Running = false;
bool pump2Running = false;

bool flowTimerStarted = false;
bool flowAlarmLatched = false;
unsigned long lowFlowStart = 0;

bool flushing = false;
bool sourceAlarm = false;
bool badTimerStarted = false;
unsigned long badStart = 0;
int flushCount = 0;

int valveAngle = VALVE_CLOSED;

bool previousSwitchOn = false;

unsigned long lastButtonCheck = 0;
unsigned long lastPrint = 0;
unsigned long lastMenuReminder = 0;

// ================= FLOW INTERRUPT =================
void pulseISR() {
  pulseCount++;
}

// ================= READ pH =================
float readPH() {
  long sum = 0;
  for (int i = 0; i < PH_SAMPLES; i++) {
    sum += analogRead(PH_PIN);
    delayMicroseconds(500);
  }
  float averageRaw = sum / (float)PH_SAMPLES;
  float rawPH = averageRaw * (14.0 / 1023.0);

  if (smoothedPH < 0.0) smoothedPH = rawPH;
  else smoothedPH = PH_ALPHA * rawPH + (1.0 - PH_ALPHA) * smoothedPH;

  return constrain(smoothedPH, 0.0, 14.0);
}

// ================= READ TURBIDITY IN NTU =================
float readTurbidityNTU() {
  long sum = 0;
  for (int i = 0; i < TURB_SAMPLES; i++) {
    sum += analogRead(TURB_PIN);
    delayMicroseconds(500);
  }
  float averageRaw = sum / (float)TURB_SAMPLES;
  float voltage = averageRaw * (5.0 / 1023.0);

  // 4 V = 0 NTU, 1 V = 4000 NTU
  float rawNTU = (4.0 - voltage) * (4000.0 / 3.0);
  rawNTU = constrain(rawNTU, 0.0, 4000.0);

  if (smoothedNTU < 0.0) smoothedNTU = rawNTU;
  else smoothedNTU = TURB_ALPHA * rawNTU + (1.0 - TURB_ALPHA) * smoothedNTU;

  return smoothedNTU;
}

// ================= ULTRASONIC DISTANCE =================
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0;
  return duration * 0.0343 / 2.0;
}

// ================= TANK SIMULATION =================
void updateSimTank() {
  unsigned long now = millis();
  float dt = (now - lastSimMillis) / 1000.0;
  lastSimMillis = now;

  if (pump1Running) simLevel += SIM_FILL_PER_S * dt;
  // Drain follows the measured flow (L/min -> L/s) as % of tank capacity
  if (pump2Running) simLevel -= (flowLpm / 60.0) * dt / SIM_TANK_CAPACITY_L * 100.0;

  simLevel = constrain(simLevel, 0.0, 100.0);
}

// ================= WATER LEVEL =================
// SIM mode: simulated level.  PROTEUS mode: HC-SR04 (adjust distance in Proteus)
int readLevel() {
  if (mode == MODE_SIM) {
    levelSensorValid = true;
    return (int)(simLevel + 0.5);
  }

  unsigned long now = millis();
  if (now - lastLevelMillis >= 200 || lastLevelMillis == 0) {
    lastLevelMillis = now;
    float d = readDistanceCm();
    if (d < 0 || d > TANK_HEIGHT_CM) {
      levelSensorValid = false;
      distanceCm = -1.0;
    } else {
      levelSensorValid = true;
      distanceCm = d;
    }
  }
  if (!levelSensorValid) return 100;
  float pct = ((TANK_HEIGHT_CM - distanceCm) / TANK_HEIGHT_CM) * 100.0;
  return constrain((int)(pct + 0.5), 0, 100);
}

// ================= SIMULATED PUMP-2 PLANT =================
void updateSimPlant(float dt) {
  float target = 0.0;

  if (pump2Running && valveAngle != VALVE_CLOSED) {
    float drive = (pump2Pwm - PUMP2_PWM_MIN) / (float)(255 - PUMP2_PWM_MIN);
    drive = constrain(drive, 0.0, 1.0);
    float head = 0.85 + 0.15 * (simLevel / 100.0);   // less head when tank is low
    target = drive * SIM_PUMP_MAX_LPM * head;
  }

  simFlow += (target - simFlow) * constrain(dt / SIM_PUMP_TAU_S, 0.0, 1.0);
}

// ================= UPDATE FLOW =================
// Returns true when a new flow sample is ready (PID runs on that tick)
bool updateFlow() {
  unsigned long now = millis();

  if (mode == MODE_SIM) {
    float pdt = (now - lastPlantMillis) / 1000.0;
    lastPlantMillis = now;
    updateSimPlant(pdt);
  }

  unsigned long window = (mode == MODE_SIM) ? FLOW_WINDOW_SIM_MS : FLOW_WINDOW_PROTEUS_MS;
  unsigned long elapsed = now - lastFlowMillis;
  if (elapsed < window) return false;

  lastFlowMillis = now;
  flowDt = elapsed / 1000.0;

  float raw;
  float alpha;

  if (mode == MODE_SIM) {
    float noise = (simFlow > 0.5) ? (random(-20, 21) / 100.0) : 0.0;   // +-0.2 L/min
    raw = simFlow + noise;
    alpha = FLOW_ALPHA_SIM;
  } else {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();
    raw = (count * 60000.0) / (PULSES_PER_LITRE * elapsed);
    alpha = FLOW_ALPHA_PROTEUS;
  }

  if (raw < 0.0) raw = 0.0;
  flowLpm = alpha * raw + (1.0 - alpha) * flowLpm;
  return true;
}

// ================= PID =================
float feedForwardPwm() {
  float ff = PUMP2_PWM_MIN + (setpoint / FF_MAX_LPM) * (PUMP2_PWM_MAX - PUMP2_PWM_MIN);
  return constrain(ff, PUMP2_PWM_MIN, PUMP2_PWM_MAX);
}

void pidReset() {
  pidI = feedForwardPwm();      // bumpless start
  pidPrevMeas = flowLpm;
  pidDFilt = 0.0;
  pump2Pwm = (int)pidI;
}

int pidCompute(float sp, float meas, float dt) {
  if (dt < 0.001) dt = 0.001;

  float e = sp - meas;
  float P = Kp * e;

  // derivative on measurement (no kick on setpoint steps), low-pass filtered
  float dRaw = -(meas - pidPrevMeas) / dt;
  pidPrevMeas = meas;
  pidDFilt = 0.7 * pidDFilt + 0.3 * dRaw;
  float D = Kd * pidDFilt;

  float out = P + pidI + D;

  // anti-windup: do not integrate further into saturation
  bool satHigh = (out >= PUMP2_PWM_MAX && e > 0);
  bool satLow  = (out <= PUMP2_PWM_MIN && e < 0);
  if (!satHigh && !satLow) pidI += Ki * e * dt;
  pidI = constrain(pidI, 0.0, 255.0);

  out = P + pidI + D;
  return (int)constrain(out, PUMP2_PWM_MIN, PUMP2_PWM_MAX);
}

// ================= VALVE CONTROL =================
void setValve(int angle) {
  valveAngle = angle;
  valveServo.write(angle);
}

// ================= PUMP CONTROL =================
void stopPump1() {
  digitalWrite(PUMP1_EN, LOW);
  digitalWrite(PUMP1_IN1, LOW);
  digitalWrite(PUMP1_IN2, LOW);
  pump1Running = false;
}

void startPump1() {
  digitalWrite(PUMP1_IN1, HIGH);
  digitalWrite(PUMP1_IN2, LOW);
  digitalWrite(PUMP1_EN, HIGH);
  pump1Running = true;
}

void stopPump2() {
  analogWrite(PUMP2_EN, 0);
  digitalWrite(PUMP2_IN3, LOW);
  digitalWrite(PUMP2_IN4, LOW);
  pump2Running = false;
}

void startPump2() {
  digitalWrite(PUMP2_IN3, HIGH);
  digitalWrite(PUMP2_IN4, LOW);
  analogWrite(PUMP2_EN, pump2Pwm);
  pump2Running = true;
}

// ================= RESET ALARMS / FLUSH STATE =================
void resetStates() {
  flowTimerStarted = false;
  flowAlarmLatched = false;

  flushing = false;
  sourceAlarm = false;
  badTimerStarted = false;
  flushCount = 0;
}

// ================= START / STOP SYSTEM =================
void startSystem() {
  if (mode == MODE_NONE) {
    Serial.println(F("[!] Select a mode first: 1 = SIM, 2 = PROTEUS"));
    return;
  }
  systemRunning = true;
  resetStates();
  pidReset();

  if (mode == MODE_SIM && autoSetpoint) {
    autoIdx = 0;
    setpoint = AUTO_SP[0];
    lastAutoStep = millis();
  }
  Serial.println(F("[SYSTEM] STARTED"));
}

void stopSystem() {
  systemRunning = false;

  stopPump1();
  stopPump2();
  pump2Pwm = 0;

  setValve(VALVE_CLOSED);
  resetStates();
}

// ================= MODE SELECTION =================
void selectMode(byte m) {
  stopSystem();
  mode = m;

  simLevel = 20.0;
  simFlow = 0.0;
  flowLpm = 0.0;
  pidI = 0.0;

  noInterrupts();
  pulseCount = 0;
  interrupts();

  unsigned long now = millis();
  lastFlowMillis = now;
  lastPlantMillis = now;
  lastSimMillis = now;

  lastLevelMillis = 0;         // force a fresh ultrasonic reading
  levelSensorValid = true;
  setpoint = SETPOINT_LPM;
  autoSetpoint = (m == MODE_SIM) && AUTO_SETPOINT_ENABLED;
  previousSwitchOn = false;    // if the switch is already ON it starts the system

  if (m == MODE_SIM) {
    Serial.println(F("[MODE] SIMULATION: level + flow simulated"));
  } else {
    Serial.println(F("[MODE] PROTEUS: level = HC-SR04, flow = sensor pulses on D2"));
  }
  Serial.println(F("Turn the switch ON to start."));
}

// ================= TERMINAL OUTPUT =================
void printMenu() {
  Serial.println();
  Serial.println(F("=== WATER TREATMENT STATION ==="));
  Serial.println(F("Select mode (press key):"));
  Serial.println(F("  1 = SIMULATION"));
  Serial.println(F("  2 = PROTEUS"));
}

// Only the mode keys are accepted from the terminal
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '1') selectMode(MODE_SIM);
    else if (c == '2') selectMode(MODE_PROTEUS);
  }
}

// ================= AUTO SETPOINT (SIM MODE) =================
void updateAutoSetpoint() {
  if (mode != MODE_SIM || !autoSetpoint || !systemRunning) return;

  if (millis() - lastAutoStep >= AUTO_STEP_MS) {
    lastAutoStep = millis();
    autoIdx = (autoIdx + 1) % AUTO_N;
    setpoint = AUTO_SP[autoIdx];

    Serial.print(F("[AUTO] setpoint -> "));
    Serial.print(setpoint, 1);
    Serial.println(F(" L/min"));
  }
}

// ================= OSCILLOSCOPE OUTPUTS =================
void updateScope() {
  float f = constrain(flowLpm / SCOPE_FULL_SCALE_LPM, 0.0, 1.0);
  float s = systemRunning ? constrain(setpoint / SCOPE_FULL_SCALE_LPM, 0.0, 1.0) : 0.0;

  analogWrite(SCOPE_FLOW_PIN, (int)(f * 255.0));
  analogWrite(SCOPE_SP_PIN,   (int)(s * 255.0));
  analogWrite(SCOPE_PUMP_PIN, pump2Running ? pump2Pwm : 0);
}

// ================= STATE CHANGE MESSAGES =================
void printStateChange(bool qualityAlarm, bool overflow) {
  static int lastKey = -1;
  byte status;
  byte bits = 0;

  if (!systemRunning)        status = 0;
  else if (sourceAlarm)      status = 4;
  else if (flushing)         status = 3;
  else if (qualityAlarm || overflow || flowAlarmLatched) {
    status = 2;
    if (qualityAlarm)     bits |= 1;
    if (overflow)         bits |= 2;
    if (flowAlarmLatched) bits |= 4;
  }
  else status = 1;

  int key = status * 16 + bits;
  if (key == lastKey) return;
  lastKey = key;

  Serial.print(F("[STATE] "));
  switch (status) {
    case 0: Serial.println(F("STOPPED")); break;
    case 1: Serial.println(F("RUNNING - NORMAL")); break;
    case 2:
      Serial.print(F("RUNNING - ALARM:"));
      if (bits & 1) Serial.print(F(" pH/turbidity"));
      if (bits & 2) Serial.print(F(" overflow/level"));
      if (bits & 4) Serial.print(F(" LOW FLOW(pump2 off, toggle switch to reset)"));
      Serial.println();
      break;
    case 3: Serial.println(F("FLUSHING BAD WATER")); break;
    case 4: Serial.println(F("ALARM: SOURCE BAD (toggle switch to reset)")); break;
  }
}

void printStatusLine(float ph, float ntu, int level) {
  Serial.print(mode == MODE_SIM ? F("SIM ") : F("PRO "));
  Serial.print(F("SP:"));
  Serial.print(systemRunning ? setpoint : 0.0, 1);
  Serial.print(F(" F:"));
  Serial.print(flowLpm, 1);
  Serial.print(F(" PWM:"));
  Serial.print(pump2Running ? pump2Pwm : 0);
  Serial.print(F(" pH:"));
  Serial.print(ph, 1);
  Serial.print(F(" T:"));
  Serial.print(ntu, 0);
  Serial.print(F(" L:"));
  if (levelSensorValid) {
    Serial.print(level);
    Serial.print(F("%"));
  } else {
    Serial.print(F("ERR"));
  }
  Serial.print(F(" P1:"));
  Serial.print(pump1Running ? F("ON") : F("OFF"));
  Serial.print(F(" P2:"));
  Serial.print(pump2Running ? F("ON") : F("OFF"));
  Serial.print(F(" V:"));
  if (valveAngle == VALVE_OPEN)       Serial.println(F("OUT"));
  else if (valveAngle == VALVE_WASTE) Serial.println(F("WST"));
  else                                Serial.println(F("OFF"));
}

// ================= SETUP =================
void setup() {
  pinMode(PUMP1_EN, OUTPUT);
  pinMode(PUMP1_IN1, OUTPUT);
  pinMode(PUMP1_IN2, OUTPUT);

  pinMode(PUMP2_EN, OUTPUT);
  pinMode(PUMP2_IN3, OUTPUT);
  pinMode(PUMP2_IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(SCOPE_FLOW_PIN, OUTPUT);
  pinMode(SCOPE_SP_PIN, OUTPUT);
  pinMode(SCOPE_PUMP_PIN, OUTPUT);

  pinMode(FLOW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseISR, RISING);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  valveServo.attach(SERVO_PIN);

  Serial.begin(9600);
  randomSeed(analogRead(A3));

  stopPump1();
  stopPump2();
  setValve(VALVE_CLOSED);

  delay(500);
  printMenu();
  lastMenuReminder = millis();
}

// ================= MAIN LOOP =================
void loop() {

  handleSerial();

  // ---------- WAIT FOR MODE SELECTION ----------
  if (mode == MODE_NONE) {
    if (millis() - lastMenuReminder >= 8000) {
      lastMenuReminder = millis();
      printMenu();
    }
    return;
  }

  // ---------- ON / OFF SWITCH ----------
  if (millis() - lastButtonCheck >= 30) {
    lastButtonCheck = millis();

    bool switchOn = (digitalRead(SWITCH_PIN) == SWITCH_ON_LEVEL);

    if (switchOn && !previousSwitchOn) {
      startSystem();
    }
    if (!switchOn && previousSwitchOn) {
      stopSystem();
      Serial.println(F("[SYSTEM] STOPPED"));
    }
    previousSwitchOn = switchOn;
  }

  // ---------- SIMULATED TANK ----------
  if (mode == MODE_SIM) updateSimTank();

  // ---------- AUTO SETPOINT ----------
  updateAutoSetpoint();

  // ---------- READ SENSORS ----------
  float ph = readPH();
  float turbidityNTU = readTurbidityNTU();
  int level = readLevel();

  bool newSample = updateFlow();
  float flow = flowLpm;

  // ---------- WATER QUALITY ----------
  bool phGood = (ph >= PH_MIN && ph <= PH_MAX);
  bool turbidityGood = (turbidityNTU <= TURBIDITY_MAX_NTU);
  bool waterQualityGood = phGood && turbidityGood;

  bool overflow = (!levelSensorValid || level >= LEVEL_OVERFLOW);
  bool qualityAlarm = !waterQualityGood;

  // ---------- SYSTEM CONTROL ----------
  if (systemRunning) {

    // ----- Bad water detection (debounced) -----
    if (!waterQualityGood && !flushing && !sourceAlarm) {

      if (!badTimerStarted) {
        badStart = millis();
        badTimerStarted = true;
      }

      if (millis() - badStart >= BAD_WATER_DELAY_MS && level > FLUSH_END_LEVEL) {
        badTimerStarted = false;

        if (flushCount >= MAX_FLUSHES) {
          sourceAlarm = true;
        } else {
          flushing = true;
          flushCount++;
        }
      }
    } else {
      badTimerStarted = false;
    }

    // ----- End of flush -----
    if (flushing && level <= FLUSH_END_LEVEL) {
      flushing = false;
    }

    // ----- Tank full of good water: forget previous flushes -----
    if (waterQualityGood && level >= LEVEL_STOP_PUMP1) {
      flushCount = 0;
    }

    // ----- Pump 1: level hysteresis -----
    if (level <= LEVEL_START_PUMP1) {
      pump1Running = true;
    } else if (level >= LEVEL_STOP_PUMP1) {
      pump1Running = false;
    }

    if (overflow || flushing || sourceAlarm) {
      pump1Running = false;
    }

    if (pump1Running) startPump1();
    else stopPump1();

    // ----- Pump 2: output good water OR flush bad water -----
    bool pump2Allowed;

    if (sourceAlarm) {
      pump2Allowed = false;
    } else if (flushing) {
      pump2Allowed = !flowAlarmLatched;
    } else {
      pump2Allowed = waterQualityGood && !overflow &&
                     level > LEVEL_MIN_PUMP2 && !flowAlarmLatched;
    }

    if (pump2Allowed) {

      if (!pump2Running) {
        pidReset();                     // (re)starting: bumpless start
        flowTimerStarted = false;
      }

      // PID runs once per new flow sample
      if (newSample) {
        pump2Pwm = pidCompute(setpoint, flow, flowDt);
      }

      startPump2();

      // Low-flow alarm: pump is at full drive but flow is still too low
      bool starved = (pump2Pwm >= PUMP2_PWM_MAX - 5) && (flow < MIN_FLOW);

      if (starved) {
        if (!flowTimerStarted) {
          lowFlowStart = millis();
          flowTimerStarted = true;
        }
        if (millis() - lowFlowStart >= FLOW_ALARM_MS) {
          flowAlarmLatched = true;
          flowTimerStarted = false;
          stopPump2();
          pump2Allowed = false;
        }
      } else {
        flowTimerStarted = false;
      }

    } else {
      stopPump2();
      flowTimerStarted = false;
    }

    // ----- Diverter valve -----
    if (!pump2Allowed)  setValve(VALVE_CLOSED);
    else if (flushing)  setValve(VALVE_WASTE);
    else                setValve(VALVE_OPEN);

  } else {

    stopPump1();
    stopPump2();
    setValve(VALVE_CLOSED);
    flowTimerStarted = false;
  }

  // ---------- SCOPE OUTPUTS ----------
  updateScope();

  // ---------- TERMINAL ----------
  printStateChange(qualityAlarm, overflow);

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    printStatusLine(ph, turbidityNTU, level);
  }
}
