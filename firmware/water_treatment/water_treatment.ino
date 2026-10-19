#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>

// ================= SIMULATION MODE =================
// 1 = tank LEVEL is simulated in software (for Proteus)
// 0 = real HC-SR04 is used (for hardware)
// FLOW is always measured from the flow sensor pulses on D2 (Proteus model or real).
#define SIMULATE_LEVEL 1

// ================= LCD =================
LiquidCrystal lcd(12, 11, A3, 3, 4, 13);

// ================= PINS =================
const int PH_PIN       = A0;
const int TURB_PIN     = A1;

const int TRIG_PIN     = A2;
const int ECHO_PIN     = A5;

const int FLOW_PIN     = 2;

// Pump 1 - L298 (fills the tank)
const int PUMP1_EN     = 5;
const int PUMP1_IN1    = 7;
const int PUMP1_IN2    = 8;

// Pump 2 - L298 (empties the tank)
const int PUMP2_EN     = 6;
const int PUMP2_IN3    = 10;
const int PUMP2_IN4    = A4;

// Servo = 3-way diverter valve
const int SERVO_PIN    = 9;

// Single ON/OFF switch (level based)
// 1 (HIGH) = system ON, 0 (LOW) = system OFF.
// Wired with a pull-up, so open/driven-high = ON and tied to ground = OFF.
// If you prefer "switch closed to ground = ON", change SWITCH_ON_LEVEL to LOW.
const int SWITCH_PIN   = 0;
const int SWITCH_ON_LEVEL = LOW;

// ================= SETTINGS =================
const float PH_MIN = 6.5;
const float PH_MAX = 8.5;

const int PH_SAMPLES = 20;
const float PH_ALPHA = 0.2;
float smoothedPH = -1.0;

// Turbidity in NTU
const int TURB_SAMPLES = 20;
const float TURB_ALPHA = 0.2;
const float TURBIDITY_MAX_NTU = 5.0;

float smoothedNTU = -1.0;

const int LEVEL_START_PUMP1 = 30;
const int LEVEL_STOP_PUMP1  = 75;
const int LEVEL_MIN_PUMP2   = 25;
const int LEVEL_OVERFLOW    = 90;

const float TANK_HEIGHT_CM = 400.0;

const float MIN_FLOW = 10.0;
const int PUMP2_SPEED = 180;

// Diverter valve positions
const int VALVE_CLOSED = 0;     // nothing leaves the tank
const int VALVE_OPEN   = 90;    // route to treated-water output
const int VALVE_WASTE  = 180;   // route to waste / drain

// ================= BAD WATER FLUSH SETTINGS =================
const unsigned long BAD_WATER_DELAY_MS = 3000;  // water must stay bad this long
const int  FLUSH_END_LEVEL = 10;                // flush until level <= this (%)
const int  MAX_FLUSHES     = 3;                 // flushes in a row before "source bad" alarm

// ================= TANK SIMULATION SETTINGS =================
#if SIMULATE_LEVEL
float simLevel = 20.0;                    // starting level (%)
const float SIM_FILL_PER_S  = 4.0;        // % per second while Pump 1 runs
const float SIM_TANK_CAPACITY_L = 10.0;   // virtual tank volume (L); drain rate follows the MEASURED flow
unsigned long lastSimMillis = 0;
#endif

// ================= FLOW SENSOR =================
const float PULSES_PER_LITRE = 450.0;

volatile unsigned long pulseCount = 0;

float flowLpm = 0.0;
unsigned long lastFlowMillis = 0;

// ================= ULTRASONIC =================
float distanceCm = -1.0;
bool levelSensorValid = false;

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

// Bad-water handling
bool flushing = false;
bool sourceAlarm = false;
bool badTimerStarted = false;
unsigned long badStart = 0;
int flushCount = 0;

int valveAngle = VALVE_CLOSED;

bool previousSwitchOn = false;

unsigned long lastButtonCheck = 0;
unsigned long lastLcdUpdate = 0;

// ================= FLOW INTERRUPT =================
void pulseISR() {
  pulseCount++;
}

// ================= READ pH =================
float readPH() {
  long sum = 0;

  for (int i = 0; i < PH_SAMPLES; i++) {
    sum += analogRead(PH_PIN);
    delay(5);
  }

  float averageRaw = sum / (float)PH_SAMPLES;

  // Simulated linear pH mapping
  float rawPH = averageRaw * (14.0 / 1023.0);

  if (smoothedPH < 0.0) {
    smoothedPH = rawPH;
  } else {
    smoothedPH =
      PH_ALPHA * rawPH +
      (1.0 - PH_ALPHA) * smoothedPH;
  }

  return constrain(smoothedPH, 0.0, 14.0);
}

// ================= READ TURBIDITY IN NTU =================
float readTurbidityNTU() {
  long sum = 0;

  for (int i = 0; i < TURB_SAMPLES; i++) {
    sum += analogRead(TURB_PIN);
    delay(5);
  }

  float averageRaw = sum / (float)TURB_SAMPLES;

  // Convert ADC reading to voltage
  float voltage = averageRaw * (5.0 / 1023.0);

  // Linear mapping used by the Proteus model:
  // 4 V = 0 NTU, 1 V = 4000 NTU
  float rawNTU = (4.0 - voltage) * (4000.0 / 3.0);

  rawNTU = constrain(rawNTU, 0.0, 4000.0);

  if (smoothedNTU < 0.0) {
    smoothedNTU = rawNTU;
  } else {
    smoothedNTU =
      TURB_ALPHA * rawNTU +
      (1.0 - TURB_ALPHA) * smoothedNTU;
  }

  return smoothedNTU;
}

// ================= ULTRASONIC DISTANCE =================
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
    pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}

// ================= TANK SIMULATION =================
#if SIMULATE_LEVEL
void updateSimTank() {
  unsigned long now = millis();
  float dt = (now - lastSimMillis) / 1000.0;
  lastSimMillis = now;

  if (pump1Running) simLevel += SIM_FILL_PER_S * dt;
  // Drain = measured flow (L/min) -> L/s, times dt, as % of tank capacity
  if (pump2Running) simLevel -= (flowLpm / 60.0) * dt / SIM_TANK_CAPACITY_L * 100.0;

  simLevel = constrain(simLevel, 0.0, 100.0);
}
#endif

// ================= UPDATE WATER LEVEL =================
int readLevel() {
#if SIMULATE_LEVEL

  levelSensorValid = true;
  return (int)(simLevel + 0.5);

#else

  unsigned long now = millis();

  if (now - lastLevelMillis >= 200 ||
      lastLevelMillis == 0) {

    lastLevelMillis = now;

    float measuredDistance = readDistanceCm();

    if (measuredDistance < 0 ||
        measuredDistance > TANK_HEIGHT_CM) {

      levelSensorValid = false;
      distanceCm = -1.0;

    } else {
      levelSensorValid = true;
      distanceCm = measuredDistance;
    }
  }

  if (!levelSensorValid) {
    return 100;
  }

  float percentage =
    ((TANK_HEIGHT_CM - distanceCm) /
      TANK_HEIGHT_CM) * 100.0;

  return constrain((int)(percentage + 0.5), 0, 100);

#endif
}

// ================= UPDATE FLOW =================
void updateFlow() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastFlowMillis;

  if (elapsed >= 1000) {

    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // pulses -> litres -> per minute, using the REAL window length
    flowLpm = (count * 60000.0) / (PULSES_PER_LITRE * elapsed);

    lastFlowMillis = now;
  }
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
  analogWrite(PUMP2_EN, PUMP2_SPEED);

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

// ================= STOP SYSTEM =================
void stopSystem() {
  systemRunning = false;

  stopPump1();
  stopPump2();

  setValve(VALVE_CLOSED);

  resetStates();
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

  attachInterrupt(
    digitalPinToInterrupt(FLOW_PIN),
    pulseISR,
    RISING
  );

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  valveServo.attach(SERVO_PIN);

  lcd.begin(20, 4);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Water Treatment");

  lcd.setCursor(0, 1);
  lcd.print("System Initializing");

  stopPump1();
  stopPump2();

  setValve(VALVE_CLOSED);

  delay(1000);
  lcd.clear();

  lastFlowMillis = millis();
  noInterrupts();
  pulseCount = 0;          // drop pulses counted during startup
  interrupts();

#if SIMULATE_LEVEL
  lastSimMillis = millis();
#endif
}

// ================= MAIN LOOP =================
void loop() {

  // ---------- ON / OFF SWITCH ----------
  if (millis() - lastButtonCheck >= 30) {
    lastButtonCheck = millis();

    bool switchOn = (digitalRead(SWITCH_PIN) == SWITCH_ON_LEVEL);

    if (switchOn && !previousSwitchOn) {
      // OFF -> ON: start the system and clear any latched alarms
      systemRunning = true;
      resetStates();
    }

    if (!switchOn && previousSwitchOn) {
      // ON -> OFF: stop everything
      stopSystem();
    }

    previousSwitchOn = switchOn;
  }

  // ---------- SIMULATED TANK UPDATE ----------
#if SIMULATE_LEVEL
  updateSimTank();
#endif

  // ---------- READ SENSORS ----------
  float ph = readPH();
  float turbidityNTU = readTurbidityNTU();

  int level = readLevel();

  updateFlow();
  float flow = flowLpm;

  // ---------- WATER QUALITY ----------
  bool phGood = (ph >= PH_MIN && ph <= PH_MAX);

  bool turbidityGood =
    (turbidityNTU <= TURBIDITY_MAX_NTU);

  bool waterQualityGood = phGood && turbidityGood;

  bool overflow =
    (!levelSensorValid || level >= LEVEL_OVERFLOW);

  bool qualityAlarm = !waterQualityGood;

  // ---------- SYSTEM CONTROL ----------
  if (systemRunning) {

    // ----- Bad water detection (debounced) -----
    if (!waterQualityGood && !flushing && !sourceAlarm) {

      if (!badTimerStarted) {
        badStart = millis();
        badTimerStarted = true;
      }

      if (millis() - badStart >= BAD_WATER_DELAY_MS &&
          level > FLUSH_END_LEVEL) {

        badTimerStarted = false;

        if (flushCount >= MAX_FLUSHES) {
          // Fresh water keeps coming in bad: stop and alarm
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

    // ----- Pump 1: fill tank using level hysteresis -----
    if (level <= LEVEL_START_PUMP1) {
      pump1Running = true;
    }
    else if (level >= LEVEL_STOP_PUMP1) {
      pump1Running = false;
    }

    // No filling while flushing, on source alarm, or on overflow
    if (overflow || flushing || sourceAlarm) {
      pump1Running = false;
    }

    if (pump1Running) {
      startPump1();
    } else {
      stopPump1();
    }

    // ----- Pump 2: output good water OR flush bad water -----
    bool pump2Allowed;

    if (sourceAlarm) {
      pump2Allowed = false;
    }
    else if (flushing) {
      pump2Allowed = !flowAlarmLatched;
    }
    else {
      pump2Allowed =
        waterQualityGood &&
        !overflow &&
        level > LEVEL_MIN_PUMP2 &&
        !flowAlarmLatched;
    }

    if (pump2Allowed) {
      startPump2();

      if (flow < MIN_FLOW) {

        if (!flowTimerStarted) {
          lowFlowStart = millis();
          flowTimerStarted = true;
        }

        if (millis() - lowFlowStart >= 3000) {
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
    if (!pump2Allowed) {
      setValve(VALVE_CLOSED);
    }
    else if (flushing) {
      setValve(VALVE_WASTE);
    }
    else {
      setValve(VALVE_OPEN);
    }

  } else {

    stopPump1();
    stopPump2();

    setValve(VALVE_CLOSED);

    flowTimerStarted = false;
  }

  // ---------- LCD DISPLAY ----------
  if (millis() - lastLcdUpdate >= 300) {
    lastLcdUpdate = millis();

    bool alarm =
      qualityAlarm || overflow || flowAlarmLatched || sourceAlarm;

    lcd.setCursor(0, 0);

    if (!systemRunning) {
      lcd.print("STOPPED             ");
    }
    else if (sourceAlarm) {
      lcd.print("ALARM: SOURCE BAD   ");
    }
    else if (flushing) {
      lcd.print("FLUSHING BAD WATER  ");
    }
    else if (alarm) {
      lcd.print("RUNNING - ALARM     ");
    }
    else {
      lcd.print("RUNNING - NORMAL    ");
    }

    // Row 1: pH and turbidity in NTU
    lcd.setCursor(0, 1);
    lcd.print("pH:");
    lcd.print(ph, 1);
    lcd.print(" T:");
    lcd.print(turbidityNTU, 0);
    lcd.print(" NTU   ");

    // Row 2: level and flow
    lcd.setCursor(0, 2);
    lcd.print("Lvl:");

    if (levelSensorValid) {
      lcd.print(level);
      lcd.print("% ");
    } else {
      lcd.print("ERR ");
    }

    lcd.print("Flow:");
    lcd.print(flow, 1);
    lcd.print("L/m ");

    // Row 3: pumps and valve (OUT = treated output, WST = waste)
    lcd.setCursor(0, 3);

    lcd.print("P1:");
    lcd.print(pump1Running ? "ON " : "OFF");

    lcd.print(" P2:");
    lcd.print(pump2Running ? "ON " : "OFF");

    lcd.print(" V:");
    if (valveAngle == VALVE_OPEN) {
      lcd.print("OUT");
    } else if (valveAngle == VALVE_WASTE) {
      lcd.print("WST");
    } else {
      lcd.print("OFF");
    }

    lcd.print(" ");
  }
}
