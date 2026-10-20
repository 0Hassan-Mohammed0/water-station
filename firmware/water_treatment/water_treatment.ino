#include <Arduino.h>
#include <Servo.h>

// =====================================================================
//  WATER STATION  -  PID flow control + Virtual Terminal (output only)
//
//  Terminal (9600 baud, 8N1):  Arduino TX(D1) -> Terminal RXD
//
//  Level : HC-SR04 (adjust distance in Proteus)
//  Flow  : flow-sensor pulses on D2 (adjust flow in Proteus)
//  Pump 2 speed is PID controlled to hold the flow setpoint.
// =====================================================================

// ================= USER SETTINGS (edit here) =================
// Flow setpoint (L/min)
const float SETPOINT_LPM = 20.0;

// PID gains
const float KP = 2.0;    // PWM counts per (L/min) of error
const float KI = 3.0;    // PWM counts per (L/min * s)
const float KD = 0.0;    // PWM counts per (L/min / s)

// ================= PINS =================
const int TRIG_PIN     = A2;
const int ECHO_PIN     = A5;

const int FLOW_PIN     = 2;

// Pump 1 - L298 (fills the tank)
const int PUMP1_EN     = 5;       // PWM
const int PUMP1_IN1    = 7;
const int PUMP1_IN2    = 8;

// Pump 2 - L298 (empties the tank) - speed is PID controlled
const int PUMP2_EN     = 6;
const int PUMP2_IN3    = 10;
const int PUMP2_IN4    = 4;

// Servo = valve
const int SERVO_PIN    = 9;

// ON/OFF switch (D0/D1 are the serial port, so the switch is on D12)
const int SWITCH_PIN      = 12;
const int SWITCH_ON_LEVEL = LOW;   // closed to ground = ON

// ================= LEVEL SETTINGS =================
const int LEVEL_START_PUMP1 = 30;
const int LEVEL_STOP_PUMP1  = 75;
const int LEVEL_MIN_PUMP2   = 25;
const int LEVEL_OVERFLOW    = 90;

const float TANK_HEIGHT_CM = 400.0;

// ================= PID / FLOW SETTINGS =================
float Kp = KP;
float Ki = KI;
float Kd = KD;

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

// Valve positions
const int VALVE_CLOSED = 0;
const int VALVE_OPEN   = 90;

// ================= FLOW SENSOR =================
const float PULSES_PER_LITRE = 450.0;
const unsigned long FLOW_WINDOW_MS = 250;
const float FLOW_ALPHA = 0.5;

volatile unsigned long pulseCount = 0;

float flowLpm = 0.0;
float flowDt  = 0.25;                     // seconds between flow samples
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

int valveAngle = VALVE_CLOSED;

bool previousSwitchOn = false;

unsigned long lastButtonCheck = 0;
unsigned long lastPrint = 0;

// ================= FLOW INTERRUPT =================
void pulseISR() {
  pulseCount++;
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

// ================= WATER LEVEL =================
int readLevel() {
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

// ================= UPDATE FLOW =================
// Returns true when a new flow sample is ready (PID runs on that tick)
bool updateFlow() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastFlowMillis;
  if (elapsed < FLOW_WINDOW_MS) return false;

  lastFlowMillis = now;
  flowDt = elapsed / 1000.0;

  noInterrupts();
  unsigned long count = pulseCount;
  pulseCount = 0;
  interrupts();

  float raw = (count * 60000.0) / (PULSES_PER_LITRE * elapsed);
  flowLpm = FLOW_ALPHA * raw + (1.0 - FLOW_ALPHA) * flowLpm;
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

  // derivative on measurement (no kick on setpoint changes), low-pass filtered
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
  analogWrite(PUMP1_EN, 0);
  digitalWrite(PUMP1_IN1, LOW);
  digitalWrite(PUMP1_IN2, LOW);
  pump1Running = false;
}

void startPump1() {
  digitalWrite(PUMP1_IN1, HIGH);
  digitalWrite(PUMP1_IN2, LOW);
  analogWrite(PUMP1_EN, 200);
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

// ================= START / STOP SYSTEM =================
void startSystem() {
  systemRunning = true;
  flowTimerStarted = false;
  flowAlarmLatched = false;
  pidReset();
  Serial.println(F("[SYSTEM] STARTED"));
}

void stopSystem() {
  systemRunning = false;

  stopPump1();
  stopPump2();
  pump2Pwm = 0;

  setValve(VALVE_CLOSED);
  flowTimerStarted = false;
  flowAlarmLatched = false;
}

// ================= TERMINAL OUTPUT =================
void printStateChange(bool overflow) {
  static int lastKey = -1;
  byte status;
  byte bits = 0;

  if (!systemRunning) status = 0;
  else if (overflow || flowAlarmLatched) {
    status = 2;
    if (overflow)         bits |= 1;
    if (flowAlarmLatched) bits |= 2;
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
      if (bits & 1) Serial.print(F(" overflow/level sensor"));
      if (bits & 2) Serial.print(F(" LOW FLOW (pump2 off, toggle switch to reset)"));
      Serial.println();
      break;
  }
}

void printStatusLine(int level) {
  Serial.print(F("SP:"));
  Serial.print(systemRunning ? setpoint : 0.0, 1);
  Serial.print(F(" F:"));
  Serial.print(flowLpm, 1);
  Serial.print(F(" PWM:"));
  Serial.print(pump2Running ? pump2Pwm : 0);
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
  Serial.println(valveAngle == VALVE_OPEN ? F("OPEN") : F("CLOSED"));
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

  pinMode(FLOW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseISR, RISING);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  valveServo.attach(SERVO_PIN);

  Serial.begin(9600);

  stopPump1();
  stopPump2();
  setValve(VALVE_CLOSED);

  delay(500);
  Serial.println(F("=== WATER STATION - PID FLOW CONTROL ==="));
  Serial.println(F("Turn the switch ON to start."));

  lastFlowMillis = millis();
  noInterrupts();
  pulseCount = 0;          // drop pulses counted during startup
  interrupts();
}

// ================= MAIN LOOP =================
void loop() {

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

  // ---------- READ SENSORS ----------
  int level = readLevel();

  bool newSample = updateFlow();
  float flow = flowLpm;

  bool overflow = (!levelSensorValid || level >= LEVEL_OVERFLOW);

  // ---------- SYSTEM CONTROL ----------
  if (systemRunning) {

    // ----- Pump 1: fill tank using level hysteresis -----
    if (level <= LEVEL_START_PUMP1) {
      pump1Running = true;
    } else if (level >= LEVEL_STOP_PUMP1) {
      pump1Running = false;
    }

    if (overflow) {
      pump1Running = false;
    }

    if (pump1Running) startPump1();
    else stopPump1();

    // ----- Pump 2: empties the tank at the PID flow setpoint -----
    bool pump2Allowed = !overflow &&
                        level > LEVEL_MIN_PUMP2 &&
                        !flowAlarmLatched;

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

    // ----- Valve -----
    setValve(pump2Allowed ? VALVE_OPEN : VALVE_CLOSED);

  } else {

    stopPump1();
    stopPump2();
    setValve(VALVE_CLOSED);
    flowTimerStarted = false;
  }

  // ---------- TERMINAL ----------
  printStateChange(overflow);

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    printStatusLine(level);
  }
}
