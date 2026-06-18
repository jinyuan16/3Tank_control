# Simulink Files — DTS200 Three-Tank System (ESP32 Interface)

## Overview

This directory contains the MATLAB/Simulink models for real-time interaction with the **DTS200 Three-Tank System** via a custom ESP32 hardware interface. The models cover three use cases:

1. **Sensor calibration** — mapping raw voltage readings to physical tank heights
2. **Pump calibration** — characterising pump flow for each actuator channel
3. **Communication illustration** — demonstrating the serial link to the ESP32 (reference only)

All models communicate with the ESP32 through a shared subsystem (`sub_com2esp32.slx`) and run at a fixed **100 Hz** sample rate over USB serial.

> **Note:** ESP32 firmware documentation is maintained in a separate file. This README covers only the Simulink-side logic.

---

## File Overview

| File | Purpose |
|---|---|
| `sub_com2esp32.slx` | Shared communication subsystem — referenced by all other models |
| `calibration_sensors.slx` | Two-point sensor calibration (voltage → height in cm) |
| `calibration_pumps.slx` | Pump flow calibration using a timed fill experiment |
| `control_esp32_DTS.slx` | Illustrative control model (not required for lab use) |

---

## `sub_com2esp32.slx` — Shared Communication Subsystem

This is a **referenced subsystem**, meaning it is not a standalone model but a reusable building block that is embedded by reference into every other `.slx` file in this directory. Any change made to this file is automatically reflected in all models that use it.

**What it does:**
- Receives binary sensor packets from the ESP32 and decodes them into three voltage signals (one per tank sensor)
- Accepts two control output values and encodes them into binary command packets sent back to the ESP32

**Why it is shared:**  
Centralising the communication logic here ensures that all models — calibration and control alike — use an identical, consistent serial interface. If the protocol ever needs to change, only this one file needs to be updated.

---

## `calibration_sensors.slx` — Sensor Calibration

### Purpose

The raw voltage signal from each tank sensor varies linearly with the water level, but the exact relationship (gain and offset) differs between sensors due to manufacturing tolerances and installation differences. This model determines the correct **linear mapping** from voltage to centimetres for all three sensors simultaneously using a **two-point calibration** method.

### Calibration Procedure

> Perform this calibration before running any control experiment.

**Step 1 — Low reference point**
1. Fill all three tanks to approximately **20 cm**.
2. Open and run `calibration_sensors.slx`.
3. Measure the actual water level in each tank with a ruler and **type the measured values** into the corresponding input fields in the Dashboard.
4. Click **"Confirm"** for each tank (the button label will have changed to indicate it is ready for input).
5. **Wait until the lamp turns green** — this confirms the low-point measurement has been accepted and stored.

**Step 2 — High reference point**
1. Continue filling all three tanks to approximately **50 cm**.
2. Measure the actual water level in each tank and **type the measured values** into the Dashboard input fields.
3. Click **"Confirm"** for each tank.
4. **Wait until the lamp turns green** again.

**Step 3 — Stop**
Once **both lamps are green** (one per calibration point), the two-point calibration is complete. You can now stop the simulation. The calibration parameters are saved automatically and will be loaded by the other models at startup.

### What the model calculates

From the two measured (voltage, height) pairs per sensor, the model computes:

```
height [cm] = gain × voltage [V] + offset
```

These parameters are stored and reused by `calibration_pumps.slx` and any control model.

---

## `calibration_pumps.slx` — Pump Calibration

### Purpose

The pump actuators receive a voltage command but the resulting flow into each tank depends on the individual pump characteristics. This model determines the **flow rate per unit voltage** for each pump by measuring how fast the tank level rises under controlled conditions.

### Calibration Procedure

> Perform sensor calibration first. Pump calibration relies on the sensor calibration file being present.

1. **Fill Tank 1 and Tank 3** to approximately **10 cm**.
2. **Close all valves** — in particular, ensure the outlet valves of **Tank 1 and Tank 3** are fully closed so that all pump flow accumulates in the tank.
3. Open `calibration_pumps.slx` and **start the model immediately** — do not wait.
4. As soon as the model is running, **switch to Automatic mode** via the Dashboard toggle.
5. The model will drive each pump through a calibration sequence and measure the resulting level rise.
6. **Wait until the calibration lamp turns green** — this indicates the calibration sequence has completed successfully and the parameters have been saved.

### What the model calculates

By measuring the rate of level rise (cm/s) at a known pump command voltage, the model computes:

```
flow [cm³/s] = pump_gain × voltage [V]
```

The resulting `pump_gain` values are saved to the calibration file for use in control experiments.

---

## `control_esp32_DTS.slx` — Communication Illustration (Reference Only)

This model is **not required** for the standard lab workflow. It serves as a reference implementation showing how the ESP32 communication subsystem (`sub_com2esp32.slx`) fits into a complete closed-loop Simulink model.

It demonstrates:
- How sensor signals flow from the ESP32 into a Simulink control block
- How computed control outputs are sent back to the ESP32
- The correct placement and wiring of `sub_com2esp32` within a model

Students building their own control models can use this file as a structural template.

---

## Typical Workflow

```
1. Connect ESP32 via USB and verify the serial port
2. Run calibration_sensors.slx  →  two-point fill procedure  →  save
3. Run calibration_pumps.slx    →  fill to 10 cm, close valves,
                                    start + switch to Auto, wait for green lamp
4. Build or open your control model (using sub_com2esp32 as the interface block)
5. Run the control experiment
```

---

## Requirements

- MATLAB R2025b with Instrument Control Toolbox
- AZ-Delivery ESP32 Dev Module flashed with the matching firmware (see firmware documentation)
- DTS200 Three-Tank System
- Data-capable USB cable
- Serial port configured at **115200 baud**