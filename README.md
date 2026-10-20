# Water Station: PID Flow Control & Tank Level System

An Arduino-based tank management system simulated in **Proteus 8.12**. Pump 1 keeps a tank filled between two level limits, and Pump 2 empties it at a **PID-controlled flow rate** measured by a flow sensor. A servo valve opens only while Pump 2 is delivering, and the system protects itself against overflow, level-sensor failure and a starved Pump 2.

Operation is a **single ON/OFF switch**. Status is printed to a **Proteus Virtual Terminal** (serial, 9600 baud, output only).

![System flow](/docs/images/water_station_illustration.svg)

---

## How It Works

1. **Fill:** Pump 1 starts when the tank level drops to **30 %** or below and stops at **75 %** or above (hysteresis, so it does not chatter).
2. **Deliver:** while the level is above **25 %**, Pump 2 runs and the servo valve opens. A **PID controller** adjusts Pump 2's PWM so the measured flow tracks the setpoint (default **20 L/min**).
3. **Protect:**
    - **Overflow / sensor fault:** at **90 %** or above, or if the level sensor reading is invalid, both pumps stop and the valve closes.
    - **Low flow:** if Pump 2 is driven at (nearly) full PWM but the flow stays below **10 L/min** for **4 s**, Pump 2 stops and the alarm latches until the switch is toggled.

---

## Key Features

- **PID flow control on Pump 2:** gains `Kp = 2.0`, `Ki = 3.0`, `Kd = 0.0`, with derivative on measurement (no kick on setpoint changes), a low-pass filtered D term, integrator anti-windup, and a feed-forward **bumpless start**.
- **Automatic tank level control:** Pump 1 ON at **≤ 30 %**, OFF at **≥ 75 %**. Pump 2 only runs above **25 %**.
- **Level sensing:** HC-SR04 ultrasonic sensor, sampled every 200 ms, tank height reference **400 cm**.
- **Accurate flow measurement:** pulses on `D2` (interrupt) are counted over the real elapsed time (250 ms windows), converted at **450 pulses/L**, and smoothed with an exponential filter (`alpha = 0.5`).
- **Servo valve:** `0°` closed, `90°` open. It is open only while Pump 2 is allowed to run.
- **Single ON/OFF switch:** closed to ground = ON. Switching ON clears any latched alarm.
- **Live serial dashboard:** a one-line status every second plus a `[STATE]` message whenever the system state changes.

---

## Virtual Terminal

Connect the Proteus **Virtual Terminal** at **9600 baud, 8N1**. The terminal is output only, so only the Arduino `TX` line is needed:

|Arduino|Terminal|
|:--|:--|
|`TX` (D1)|`RXD`|

Example session:

```text
=== WATER STATION - PID FLOW CONTROL ===
Turn the switch ON to start.
[SYSTEM] STARTED
[STATE] RUNNING - NORMAL
SP:20.0 F:0.0 PWM:0 L:20% P1:ON P2:OFF V:CLOSED
SP:20.0 F:0.0 PWM:0 L:28% P1:ON P2:OFF V:CLOSED
SP:20.0 F:9.6 PWM:112 L:36% P1:ON P2:ON V:OPEN
```

### Status line

|Field|Meaning|
|:--|:--|
|`SP`|Flow setpoint in L/min (`0.0` when stopped)|
|`F`|Measured (filtered) flow in L/min|
|`PWM`|Pump 2 PWM output (0-255, `0` when Pump 2 is off)|
|`L`|Tank level in % (`ERR` if the level sensor fails)|
|`P1`, `P2`|Pump 1 / Pump 2 `ON` or `OFF`|
|`V`|Valve: `OPEN` or `CLOSED`|

### State messages

|Message|Meaning|
|:--|:--|
|`[SYSTEM] STARTED` / `[SYSTEM] STOPPED`|Switch turned ON / OFF|
|`[STATE] STOPPED`|System off|
|`[STATE] RUNNING - NORMAL`|Everything fine|
|`[STATE] RUNNING - ALARM: overflow/level sensor`|Level ≥ 90 % or invalid level reading|
|`[STATE] RUNNING - ALARM: LOW FLOW (pump2 off, toggle switch to reset)`|Pump 2 saturated but flow below 10 L/min for 4 s|

Both alarms can appear together on the same line.

---

## Proteus Behaviour

Both the level and the flow are **inputs you set by hand** in Proteus:

- **Level:** adjust the HC-SR04 distance (`RV2`).
- **Flow:** set the flow sensor output.

Because the flow sensor is a manual input, the PID cannot actually change the flow in Proteus. Pump 2's PWM will move in response to the error, but the measured flow only changes when you change it. This is useful for testing the sensors, the level logic and the alarms.

---

## Repository Structure

```text
Water Treatment/
├── .gitignore
├── README.md
├── docs/
│   └── images/                         # Screenshots and diagrams used in this README
│       ├── system_flow.png
│       ├── system_flow.svg
│       └── proteus-schematic.png
├── firmware/
│   └── water_treatment/
│       └── water_treatment.ino         # Main Arduino sketch
└── Simulation/
    ├── industerial Control.pdsprj      # Proteus 8 simulation project
    ├── HEX/                            # Compiled firmware & sensor models
    │   ├── water_treatment.ino.hex
    │   ├── pHMeterLibraryTEP.HEX
    │   └── UltraSonicTEP.HEX
    └── libraries/                      # Required Proteus 8 component libraries
        ├── Arduino Library for Proteus.rar
        ├── Electronics-Tree-Flow-Sensor-Library.zip
        ├── Electronics-Tree-Turbidity-Sensor-Library.zip
        ├── PHMeterLibraryforProteus.zip
        ├── Ultrasonic Sensor Library for Proteus.rar
        └── Password.txt
```

> The pH and turbidity sensors are **not read by this version of the firmware**. Their libraries are only needed if they are still on the schematic.

---

## Hardware Pinout

|Component|Signal / Pin|Arduino Uno Pin|Notes|
|:--|:--|:--|:--|
|**Ultrasonic Sensor**|Trigger / Echo|`A2` / `A5`|400 cm tank height reference|
|**Flow Sensor**|Pulse Out|`2` (INT0)|External interrupt (rising edge), `450 pulses/L`, on the Pump 2 outlet line|
|**Pump 1 (Intake, L298)**|EN / IN1 / IN2|`5` / `7` / `8`|ON/OFF with level hysteresis, EN driven at PWM `200`|
|**Pump 2 (Transfer, L298)**|EN / IN3 / IN4|`6` / `10` / `4`|PWM from the PID (`40` - `255`)|
|**Servo Valve**|PWM Signal|`9`|`0°` closed, `90°` open|
|**ON/OFF Switch**|Level input|`12`|Closed to ground = ON (`INPUT_PULLUP`)|
|**Virtual Terminal**|TX|`1`|Serial, 9600 baud, 8N1 (output only)|

> **Note:** the switch is on `D12` because `D0`/`D1` are the serial port. Compared to the previous version, Pump 1's enable pin moved from `D4` to `D5`, and Pump 2's `IN4` moved from `A4` to `D4`. The pH (`A0`), turbidity (`A1`) and random-seed (`A3`) pins are no longer used. Make sure the Proteus schematic wiring matches this table.

### ON/OFF switch wiring

The switch input uses the internal pull-up, so an **open input reads OFF** and an input **closed to ground reads ON**:

```cpp
const int SWITCH_ON_LEVEL = LOW;   // closed to ground = ON
```

---

## Settings (edit in the sketch, then re-upload)

All main settings are in the `USER SETTINGS` block at the top of `water_treatment.ino`:

|Constant|Default|Meaning|
|:--|:--|:--|
|`SETPOINT_LPM`|`20.0`|Flow setpoint (L/min)|
|`KP` / `KI` / `KD`|`2.0` / `3.0` / `0.0`|PID gains|

Other tunables further down the sketch:

|Constant|Default|Meaning|
|:--|:--|:--|
|`LEVEL_START_PUMP1` / `LEVEL_STOP_PUMP1`|`30` / `75`|Pump 1 ON / OFF level (%)|
|`LEVEL_MIN_PUMP2`|`25`|Pump 2 only runs above this level (%)|
|`LEVEL_OVERFLOW`|`90`|Overflow cut-off (%)|
|`TANK_HEIGHT_CM`|`400.0`|Sensor-to-bottom distance used for the level calculation|
|`PUMP2_PWM_MIN` / `PUMP2_PWM_MAX`|`40` / `255`|PWM limits for Pump 2|
|`FF_MAX_LPM`|`60.0`|Flow at full PWM, used only for the start-up feed-forward guess|
|`MIN_FLOW`|`10.0`|Low-flow alarm threshold (L/min)|
|`FLOW_ALARM_MS`|`4000`|Time Pump 2 must be starved before the alarm latches|
|`PULSES_PER_LITRE`|`450.0`|Flow sensor calibration|
|`FLOW_WINDOW_MS` / `FLOW_ALPHA`|`250` / `0.5`|Flow sample window and filter coefficient|

---

## Setting Up & Running the Simulation in Proteus 8

### 1. Install the Proteus Libraries

Before opening the project, install the custom sensor and Arduino libraries located in `Simulation/libraries/`:

1. **Extract the library archives:** extract all `.zip` and `.rar` files inside `Simulation/libraries/` (check `Password.txt` if an archive prompts for a password).
2. **Copy the library files:** copy all extracted `.LIB` and `.IDX` files into your Proteus `LIBRARY` folder:
    - `C:\ProgramData\Labcenter Electronics\Proteus 8 Professional\LIBRARY` _(note: `ProgramData` is a hidden folder)_
    - or `C:\Program Files (x86)\Labcenter Electronics\Proteus 8 Professional\DATA\LIBRARY`
3. **Restart Proteus:** close and reopen Proteus 8 so the newly installed components load properly.

### 2. Link the `.HEX` Files

1. Open `Simulation/industerial Control.pdsprj` in Proteus 8.
2. Double-click each of the following components on the schematic and set their **Program File** field to the corresponding file inside `Simulation/HEX/`:

|Component in Proteus|`.HEX` File to Assign (`Simulation/HEX/`)|
|:--|:--|
|**Arduino Uno**|`water_treatment.ino.hex`|
|**Ultrasonic Sensor**|`UltraSonicTEP.HEX`|
|**pH Sensor** _(only if still on the schematic)_|`pHMeterLibraryTEP.HEX`|

### 3. Run

1. Click **Play** at the bottom-left of the Proteus window. The **Virtual Terminal** opens and shows the start banner.
2. Close the **START** switch (to ground). The terminal prints `[SYSTEM] STARTED`.
3. Set the flow sensor to a value **above 10 L/min** _before_ Pump 2 runs at full PWM, otherwise the low-flow alarm triggers after 4 seconds.

![Proteus schematic](/docs/images/proteus-schematic_v3.png)

---

## Testing the Behaviour

|To test|Do this|Expected result|
|:--|:--|:--|
|**Normal cycle**|Switch ON, level around 20 %, flow above 10 L/min|Pump 1 fills to 75 %, Pump 2 starts above 25 % with the valve `OPEN`, PWM adjusts toward the 20 L/min setpoint|
|**Level hysteresis**|Move the level between 30 % and 75 %|Pump 1 turns ON at ≤ 30 % and OFF at ≥ 75 %|
|**Pump 2 cut-off**|Lower the level to 25 % or below|Pump 2 stops and the valve closes|
|**Low-flow alarm**|Set flow below 10 L/min while Pump 2 runs at full PWM|After 4 s Pump 2 stops, `RUNNING - ALARM: LOW FLOW (...)`|
|**Overflow**|Raise the level to 90 % or above|Both pumps stop, valve closes, `ALARM: overflow/level sensor`|
|**Level-sensor fault**|Make the HC-SR04 reading invalid (no echo or out of range)|`L:ERR`, treated as overflow, both pumps stop|
|**Reset an alarm**|Switch OFF, then ON|Alarms cleared, normal operation resumes|

---

## Modifying the Firmware

If you edit `firmware/water_treatment/water_treatment.ino`:

1. Open the sketch in the **Arduino IDE** and select **Sketch -> Export Compiled Binary** (or compile via VS Code / Arduino CLI).
2. Copy the newly generated `water_treatment.ino.hex` file into `Simulation/HEX/` to update the simulation firmware.