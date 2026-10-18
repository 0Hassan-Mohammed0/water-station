# Automated Water Treatment & Tank Control System

An Arduino-based automated water quality monitoring and tank management system simulated in **Proteus 8**. The system automates intake tank filling, continuously tests water quality (pH and Turbidity), controls an outlet servo valve, and protects the delivery pump against dry-running or pipe blockages.

---

## Key Features

* **Automated Tank Level Control:** Uses an ultrasonic sensor with hysteresis control to automatically start the intake pump (Pump 1) at **≤ 30%** capacity and stop it at **≥ 75%** (with an overflow safety cutoff at **90%**).
* **Real-Time Water Quality Inspection:** Continuously monitors acidity (**pH 6.5 – 8.5**) and cloudiness (**Turbidity ≤ 5.0 NTU**) using exponential moving average filtering.
* **Safe Water Delivery & Flow Protection:** Engages the transfer pump (Pump 2) and opens the servo outlet valve only when water quality passes inspection and the tank level exceeds **25%**. Includes a **3-second low-flow cutoff (< 10.0 L/min)** to protect Pump 2 from dry runs or pipe blockages.
* **Live 20x4 LCD Dashboard:** Displays system status (`STOPPED`, `RUNNING - NORMAL`, `RUNNING - ALARM`), live sensor readings, and pump/valve states.

---

## Repository Structure

```text
Water Treatment/
├── .gitignore
├── README.md
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

---

## Hardware Pinout

| Component | Signal / Pin | Arduino Uno Pin | Notes |
| :--- | :--- | :--- | :--- |
| **pH Sensor** | Analog Out | `A0` | Valid range: `6.5 – 8.5` |
| **Turbidity Sensor** | Analog Out | `A1` | Valid range: `≤ 5.0 NTU` |
| **Ultrasonic Sensor** | Trigger / Echo | `A2` / `A5` | 400 cm tank height reference |
| **Flow Sensor** | Pulse Out | `2` (INT0) | External interrupt (`450 pulses/L`) |
| **Pump 1 (Intake - L298N)** | EN / IN1 / IN2 | `5` / `7` / `8` | PWM speed: `200` (ON ≤ 30%, OFF ≥ 75%) |
| **Pump 2 (Transfer - L298N)** | EN / IN3 / IN4 | `6` / `10` / `A4` | PWM speed: `180` (Runs when level > 25% & quality good) |
| **Outlet Servo Valve** | PWM Signal | `9` | `90°` Open, `0°` Closed |
| **Control Buttons** | Start / Stop | `0` / `1` | Active `LOW` (`INPUT_PULLUP`) |
| **20x4 LCD** | RS, EN, D4, D5, D6, D7 | `12, 11, A3, 3, 4, 13` | 4-bit parallel mode |

---

## Setting Up & Running the Simulation in Proteus 8

### 1. Install the Proteus Libraries
Before opening the project, install the custom sensor and Arduino libraries located in `Simulation/libraries/`:

1. **Extract the library archives:** Extract all `.zip` and `.rar` files inside `Simulation/libraries/` (check `Password.txt` if an archive prompts for a password).
2. **Copy the library files:** Copy all extracted `.LIB` and `.IDX` files into your Proteus `LIBRARY` folder:
   * `C:\ProgramData\Labcenter Electronics\Proteus 8 Professional\LIBRARY` *(Note: `ProgramData` is a hidden folder)*
   * Or `C:\Program Files (x86)\Labcenter Electronics\Proteus 8 Professional\DATA\LIBRARY`
3. **Restart Proteus:** Close and reopen Proteus 8 so the newly installed components load properly.

### 2. Link the `.HEX` Files
1. Open `Simulation/industerial Control.pdsprj` in Proteus 8.
2. Double-click each of the following components on the schematic and set their **Program File** field to the corresponding file inside `Simulation/HEX/`:

| Component in Proteus | `.HEX` File to Assign (`Simulation/HEX/`) |
| :--- | :--- |
| **Arduino Uno** | `water_treatment.ino.hex` |
| **pH Sensor** | `pHMeterLibraryTEP.HEX` |
| **Ultrasonic Sensor** | `UltraSonicTEP.HEX` |
| **Turbidity & Flow Sensors** | Follow the setup instructions inside their extracted library folders in `Simulation/libraries/` if prompted |

3. Click **Play** at the bottom-left of the Proteus window to start the simulation, then press the **START** button on the schematic to begin normal operation.

---

## Modifying the Firmware

If you edit `firmware/water_treatment/water_treatment.ino`:
1. Open the sketch in the **Arduino IDE** and select **Sketch -> Export Compiled Binary** (or compile via VS Code / Arduino CLI).
2. Copy the newly generated `water_treatment.ino.hex` file into `Simulation/HEX/` to update the simulation firmware.