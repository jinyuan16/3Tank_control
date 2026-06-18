# `calibration_sensors.slx` — Sensor Calibration

## Overview

Each tank sensor produces a raw voltage that varies linearly with water height. Because manufacturing tolerances and installation differences cause each sensor to have a slightly different offset and gain, the voltage-to-height relationship must be determined experimentally before any control experiment can be run.

This model performs a **two-point linear calibration** for all three sensors simultaneously. The operator configures the tank valves so that water distributes freely and reaches the same level in all three tanks, fills to two known reference heights, reads the height from the measuring tape on the tanks, and enters the values into the Dashboard. When both points are confirmed, the model computes the gain coefficient for each sensor and saves the result automatically when the simulation is stopped.

> **Prerequisite:** None. This is the first calibration step in the lab workflow.

> **Output:** `sensor_calibration.mat` — loaded automatically by `calibration_pumps.slx` and control models at startup.

---

## Part 1 — Operator Instructions

### Tank setup — DTS200 Three-Tank System

The DTS200 consists of three tanks with the following connections:

| Tank | Inlet | Outlet | Connecting valves |
|---|---|---|---|
| T1 | ✓ (pump) | ✓ | Valve to T2 |
| T2 | — | ✓ | Valve to T1, valve to T3 |
| T3 | ✓ (pump) | ✓ (×2) | Valve to T2 |

Each tank has a **measuring tape** fixed to the outside for direct height readings. Water height is read in centimetres directly from this tape — no separate ruler is needed.

---

### Physical setup before calibration

Before starting the simulation, configure the physical system as follows:

1. **Close all outlet valves** on T1, T2, and T3. No water should be able to drain during the calibration.
2. **Open all connecting valves** between tanks (T1↔T2 and T2↔T3). With outlets closed and connecting valves open, water distributes freely between all three tanks and reaches the same level everywhere. This is what allows a single height measurement to represent all three sensors.

> If any outlet is left open or any connecting valve is left closed, the water levels will differ between tanks and the calibration will be invalid.

---

### Step 1 — Low reference point (~20 cm)

1. With the valves configured as above, fill the tanks (via T1 or T3 inlet) to approximately **20 cm**. Read the exact level from the measuring tape on any tank — because all connecting valves are open, all tanks are at the same height.
2. Open `calibration_sensors.slx` in Simulink and click **Run**. The Dashboard opens automatically.
3. Type the exact measured height (in **cm**) into the **upper edit field** on the Dashboard.
4. Click **Measure** (the upper push button).
5. **Wait until the upper lamp turns green.** This confirms the low-point voltage snapshot has been captured. Do not proceed until the lamp is green.

---

### Step 2 — High reference point (~50 cm)

1. Continue filling the tanks to approximately **50 cm**. Read the exact level from the measuring tape.
2. Type the exact measured height (in **cm**) into the **lower edit field** on the Dashboard.
3. Click **Measure** (the lower push button).
4. **Wait until the lower lamp turns green.**

---

### Step 3 — Stop the simulation

Once **both lamps are green**, the calibration is complete. Click **Stop** in Simulink to end the simulation.

The calibration parameters are saved automatically to `sensor_calibration.mat` when the simulation stops. No manual saving is required.

---

### Dashboard reference

| Widget | Purpose |
|---|---|
| Upper edit field | Enter the measured height at the **low** reference point (cm) |
| **Measure** button (upper) | Capture sensor voltages as the low-point reference |
| Upper lamp | Green = low-point confirmed and stored |
| Lower edit field | Enter the measured height at the **high** reference point (cm) |
| **Measure** button (lower) | Capture sensor voltages as the high-point reference |
| Lower lamp | Green = high-point confirmed and stored |
| Scope | Real-time display of the three raw ADC voltage signals |

---

### What to do if something goes wrong

**Wrong height typed before clicking Measure:** Correct the value in the edit field before clicking — the value is not captured until the button is pressed.

**Wrong height confirmed by mistake:** Stop the simulation (do not worry about saving — if both lamps are not yet green, the incomplete calibration will not produce valid coefficients). Restart the simulation from Step 1. Persistent variables are cleared on every new simulation start.

**Lamp does not turn green after clicking Measure:** Verify the simulation is running (status bar should show "Running"). If the ESP32 serial link is not active, ADC values will be zero — check that the ESP32 is connected and the correct COM port is set in `sub_com2esp32`.

---

## Part 2 — Code Logic

This section describes how the model is implemented, intended for developers who need to maintain, extend, or debug the calibration logic.

---

### Model structure

The top-level model contains four functional components wired together:

```
[Constant 255] ──────────────────────────────────────────┐
[Constant 255] ──────────────────────────────────────────┤
                                                          ↓
                                               [sub_com2esp32]  ──→  adc1, adc2, adc3, valid
                                                                               ↓
[confirm low]  ──────────────────────────────────────────┐
[actual low]   ──────────────────────────────────────────┤
[confirm high] ──────────────────────────────────────────┤
[actual high]  ──────────────────────────────────────────┤
[adc1]         ──────────────────────────────────────────┤
[adc2]         ──────────────────────────────────────────┤
[adc3]         ──────────────────────────────────────────┴→  [MATLAB Function]
                                                                        ↓
                                          vl1 (low lamp), vl2 (high lamp),
                                          sh (sensor_high), c (coefficients), sl (sensor_low)
                                                   ↓             ↓              ↓
                                           [To Workspace] [To Workspace1] [To Workspace2]
```

The two `Constant` blocks send `255` (= 3.3 V DAC command) to the ESP32 during calibration, keeping the pumps off. The `valid` output from `sub_com2esp32` is terminated and not used in this model.

---

### The MATLAB Function block

The entire calibration logic lives in a single MATLAB Function block. It runs at every simulation step (100 Hz) and uses **persistent variables** to retain state between steps.

```matlab
function [vl1, vl2, sh, c, sl] = fcn(conflow, aclow, confhigh, achigh, adc1, adc2, adc3)

persistent sensor_low sensor_high actual_low actual_high val_low val_high
if isempty(sensor_low) && isempty(sensor_high)
    sensor_low  = zeros(3,1);
    sensor_high = zeros(3,1);
    actual_high = 0;
    actual_low  = 0;
    val_low     = 0;
    val_high    = 0;
end

vl2 = val_high;
vl1 = val_low;

if conflow == 1
    sensor_low = [adc1; adc2; adc3];
    actual_low = aclow;
    val_low    = 1;
end

if confhigh == 1
    sensor_high = [adc1; adc2; adc3];
    actual_high = achigh;
    val_high    = 1;
end

if val_low && val_high
    c  = (actual_high - actual_low) ./ (sensor_high - sensor_low);
    sl = sensor_low;
    sh = sensor_high;
else
    c  = zeros(3,1);
    sh = zeros(3,1);
    sl = zeros(3,1);
end
```

#### Persistent variable initialisation

On the first execution step after the simulation starts, all persistent variables are `isempty` (MATLAB initialises them as empty matrices). The `isempty` guard runs the initialisation block exactly once, clearing all state from any previous run. This means restarting the simulation always gives a clean calibration state — there is no risk of stale values persisting from a previous run.

#### Capture logic

`conflow` and `confhigh` are the outputs of the two Dashboard push buttons. A push button in a Simulink Dashboard outputs `1` while it is held down and `0` otherwise. The function checks for `== 1` at each time step; as long as the button remains pressed, the snapshot is continuously overwritten with the latest ADC reading. Because the ADC signal at a stable water level is essentially constant, the last captured value before the button is released is the value that persists.

There is no explicit rising-edge detector in the MATLAB code — the momentary nature of the push button widget provides equivalent behaviour in practice.

When the **Measure (low)** button is pressed (`conflow == 1`):
- `sensor_low` is set to the three current ADC voltages as a 3×1 column vector.
- `actual_low` is set to the scalar height entered in the Dashboard edit field.
- `val_low` is latched to `1` (it stays `1` even after the button is released).

The same logic applies symmetrically for the **Measure (high)** button.

#### Coefficient computation

Once both `val_low` and `val_high` are `1`, the gain for each sensor is computed element-wise:

```
c = (actual_high - actual_low) ./ (sensor_high - sensor_low)
```

This is the slope of the linear relationship between voltage and height for each sensor individually. The result is a 3×1 vector — one gain per sensor.

The outputs `sl` and `sh` (sensor voltages at low and high) are also passed out of the function for saving.

Until both points are confirmed, all three outputs (`c`, `sl`, `sh`) remain zero vectors.

#### Lamp outputs

`vl1` and `vl2` mirror `val_low` and `val_high` directly. Because `val_low` latches to `1` the moment Confirm Low is pressed and never resets during a run, the green lamp stays on permanently once confirmed.

---

### To Workspace blocks

Three `To Workspace` blocks capture the function outputs with `MaxDataPoints = 1`, meaning only the final value at simulation stop is retained:

| Workspace variable | Connected output | Content |
|---|---|---|
| `coefficients` | `c` | 3×1 gain vector (one value per sensor) |
| `sensor_high` | `sh` | 3×1 voltage readings at the high reference point |
| `sensor_low` | `sl` | 3×1 voltage readings at the low reference point |

---

### StopFcn callback

When the simulation is stopped, MATLAB executes the following callback:

```matlab
coefficients = out.coefficients;
save('C:\Users\user\Documents\MATLAB\sensor_calibration.mat', 'coefficients');
```

The `out.coefficients` value is the last-captured gain vector from the `To Workspace` block. The `.mat` file is written to the specified path.

> ⚠️ **Known issue — hardcoded save path:** The file path `C:\Users\user\Documents\MATLAB\` is hardcoded to a specific Windows user account. This will fail on any other machine. The path must be updated to match the actual MATLAB working directory before use. The recommended fix is to replace the path with a relative reference:
> ```matlab
> coefficients = out.coefficients;
> save(fullfile(pwd, 'sensor_calibration.mat'), 'coefficients');
> ```

> ⚠️ **Known limitation — offset not saved:** The StopFcn saves only the gain vector (`coefficients`). A full linear calibration also requires an offset per sensor, computed as:
> ```matlab
> offset = actual_low - coefficients .* sensor_low
> ```
> where `sensor_low` and `actual_low` are available in the workspace at the time the StopFcn runs. If the downstream experiment model requires offset values, the StopFcn should be extended to save `sensor_low` and `actual_low` (or the pre-computed offsets) alongside `coefficients`.

---

### Signal and data types

| Signal | Type | Range | Notes |
|---|---|---|---|
| `adc1`, `adc2`, `adc3` | `double` | 0 – 3.3 V | Converted from raw 12-bit ADC in `sub_com2esp32` |
| `conflow`, `confhigh` | `double` | 0 or 1 | Dashboard push button output |
| `aclow`, `achigh` | `double` | 0 – 100 cm | Operator-entered measured height (scalar) |
| `sensor_low`, `sensor_high` | `double` | 0 – 3.3 V | 3×1 voltage snapshot at calibration point |
| `coefficients` (`c`) | `double` | ~cm/V | 3×1 gain vector, one per sensor |

---

### How the saved calibration is used downstream

The `sensor_calibration.mat` file is loaded by downstream models (e.g. `calibration_pumps.slx`, control models) via an `InitFcn` callback that runs before the simulation starts. The experiment model applies the saved coefficients to convert live ADC voltages to heights in centimetres:

```matlab
height [cm] = coefficients .* voltage [V] + offset
```

where `offset` must also be available (see the limitation note above).
