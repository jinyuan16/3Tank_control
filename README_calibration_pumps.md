# `calibration_pumps.slx` — Pump Calibration

## Overview

Each pump delivers a flow rate that depends on the voltage applied to it, but the exact relationship differs between pumps due to manufacturing variation and wear. This model determines the **flow rate characteristic curve** for both pumps by applying a sequence of eight voltage steps and measuring how fast the water level rises in the corresponding tank at each step.

The result is a set of flow rate values (in cm³/s) mapped to DAC command levels — one set per pump. These values are saved and used by control models to compute the voltage command needed to achieve a desired flow rate.

> **Prerequisite:** Sensor calibration must be completed first. This model loads `sensor_calibration.mat` at startup and will fail to initialise if the file is not present.

> **Output:** `pump_calibration.mat` — loaded automatically by control models at startup.

---

## Part 1 — Operator Instructions

### Tank setup

The pump calibration measures level rise in Tank 1 and Tank 3 in isolation. Tank 2 has no pump and is not used. The setup goal is to ensure that all water pumped into T1 and T3 stays in those tanks for the duration of the experiment.

Configure the physical system as follows:

1. **Close all outlet valves** on T1, T2, and T3.
2. **Close all connecting valves** between tanks (T1↔T2 and T2↔T3). Unlike sensor calibration, the tanks must be **isolated** from each other so that flow into T1 is only measured in T1, and flow into T3 only in T3.
3. Fill **Tank 1** and **Tank 3** to approximately **10 cm**. This starting level gives enough headroom for 8 fill steps without risking overflow.

> T2 does not need to be at any particular level. It has no inlet and its outlet is closed, so it plays no role in this calibration.

---

### Procedure

Timing matters for this calibration. Start the simulation immediately after the physical setup is complete.

1. Open `calibration_pumps.slx` in Simulink and click **Run** immediately — do not wait.
2. As soon as the simulation is running, **switch the DTS200 controller to Automatic mode**. This enables computer control of the pumps via the ESP32 DAC outputs. If this step is delayed, the first calibration step will be missed.
3. The model runs the calibration sequence fully automatically. Do not interact with the simulation.
4. **Monitor the Dashboard lamp.** When the lamp turns green, the eight-step calibration sequence is complete and the flow rate data has been computed.
5. **Stop the simulation.** The calibration parameters are saved automatically to `pump_calibration.mat` when the simulation stops.

---

### Calibration sequence timing

The model executes 8 calibration steps. Each step takes 12 seconds, giving a total calibration time of approximately **96 seconds** (about 1 minute 40 seconds). No interaction is required during this time.

| Step | DAC command (approx.) | Pump voltage | Duration |
|---|---|---|---|
| 1 | 223 / 255 | ~2.9 V | 12 s |
| 2 | 191 / 255 | ~2.5 V | 12 s |
| 3 | 159 / 255 | ~2.1 V | 12 s |
| 4 | 128 / 255 | ~1.65 V | 12 s |
| 5 | 96 / 255 | ~1.24 V | 12 s |
| 6 | 64 / 255 | ~0.83 V | 12 s |
| 7 | 32 / 255 | ~0.41 V | 12 s |
| 8 | 0 / 255 | 0 V (off) | 12 s |

The sequence starts from near-maximum pump speed and steps down to zero in equal increments of 12.5%.

---

### Dashboard reference

| Widget | Purpose |
|---|---|
| Scope (Tank 1) | Real-time voltage signal from Tank 1 sensor |
| Scope (Tank 3) | Real-time voltage signal from Tank 3 sensor |
| Lamp | Green = calibration sequence complete, data saved |

---

### What to do if something goes wrong

**Model started before DTS200 is in Automatic mode:** The first one or two steps may have zero or incorrect flow readings. Stop the simulation, drain tanks back to ~10 cm, and repeat from the beginning.

**Lamp never turns green:** Check that the simulation is running and that the ESP32 is connected and the serial link is active. If ADC values are flat (zero or constant), the serial link is down — verify the COM port in `sub_com2esp32`. Also confirm `sensor_calibration.mat` is present in the MATLAB path — if the InitFcn fails to load it, the model will not initialise correctly.

**Tank overflows before sequence ends:** The starting level was too high, or an outlet valve was accidentally left open. Drain, reset valves, reduce the starting level to ~10 cm, and repeat.

**`sensor_calibration.mat` not found on startup:** Run `calibration_sensors.slx` first and complete the two-point sensor calibration. The pump model will not run without this file.

---

## Part 2 — Code Logic

This section describes the internal implementation for developers maintaining or extending the calibration model.

---

### InitFcn callback

Before the simulation starts, MATLAB executes:

```matlab
load('C:\Users\user\Documents\MATLAB\sensor_calibration.mat');
coefficients = [0, coefficients];
```

This does two things:

1. **Loads sensor gains** from the sensor calibration into the workspace variable `coefficients`.
2. **Prepends a zero time column.** The pump model feeds `coefficients` into a `From Workspace` block. Simulink's `From Workspace` block interprets the first column of its input as a time vector. Without the prepended zero, the 3-element gain vector would be misinterpreted as timestamped data and produce a dimension mismatch inside the downstream MATLAB Function block. The `[0, coefficients]` transform makes the data appear as a single sample at t=0, which `From Workspace` then holds constant for the duration of the simulation.

> ⚠️ **Known issue — hardcoded path:** The file path is hardcoded to a specific Windows user account, as in the sensor calibration model. Replace with `fullfile(pwd, 'sensor_calibration.mat')` for portability.

---

### Model structure

```
[Pulse Generator] ──────────────────────────────────→ [MATLAB Function1 (step generator)]
                                                              │
                                          y (DAC step) ──────┼──────────────────────→ dac1
                                                             │                  ↗
                                                             └──────────────────────→ dac2
                                                             │
                                          y ──────────────────────────────────────→ [MATLAB Function (main)] input 2
                                                                                              ↑
[From Workspace (coefficients)] ──────────────────────────────────────────────────→ input 1
[sub_com2esp32] ──→ adc1 (Tank 1) ────────────────────────────────────────────────→ input 3
[sub_com2esp32] ──→ adc3 (Tank 3) ────────────────────────────────────────────────→ input 4
                                                                                              │
                                                                          Q_out ──────→ [To Workspace]
                                                                          finished ──→ [Terminator]
```

Both pumps receive the **same DAC command** simultaneously. The model measures the independent level response in T1 and T3 separately, allowing per-pump characterisation even though both run at the same voltage level at each step.

Only `adc1` (Tank 1) and `adc3` (Tank 3) are used. `adc2` (Tank 2) is not connected — Tank 2 has no pump.

---

### MATLAB Function1 — Step Generator

This function produces the DAC step sequence that drives both pumps. It takes the pulse generator output as its only input and uses a rising-edge detector with a persistent counter to step through eight DAC levels.

```matlab
function [y, running] = fcn(u)
persistent counter2 prev_u

if isempty(counter2) && isempty(prev_u)
    counter2 = 0;
    prev_u   = 0;
end
running = 1;

if u == 1 && prev_u == 0   % rising edge detected
    counter2 = counter2 + 1;
end

if u == 0   % LOW phase: output the current step value
    s = 0.125 * counter2;
    if s <= 1
        y       = 255 * (1 - s);
        running = 0;
    else
        y = 255;
    end
else         % HIGH phase: always output 255
    y = 255;
end

prev_u = u;
```

**Timing and output values:**

The Pulse Generator has a 12-second period with a 20% pulse width (HIGH for 2.4 s, LOW for 9.6 s). On each rising edge (LOW→HIGH), `counter2` increments and the step fraction `s` increases by 0.125.

During the LOW phase (9.6 s), the function outputs `y = 255 × (1 − s)`, stepping from ~223 down to 0 over the eight cycles. During the HIGH phase (2.4 s), it always outputs 255, regardless of step number.

The `running` output is `0` during the LOW measurement phase and `1` during HIGH. This signal is currently routed to a Terminator and not used by any Dashboard display.

---

### MATLAB Function — Main Calibration

This is the core measurement function. It captures the tank level at each HIGH→LOW transition, computes the level rise over the preceding LOW phase, and accumulates the results into a flow rate matrix.

```matlab
function [Q_out, finished] = fcn(cal_val, state, adc1, adc3)
persistent sensor_low sensor_high prev_state Q counter cal

if isempty(sensor_low) && isempty(sensor_high)
    sensor_low  = zeros(2,1);
    sensor_high = zeros(2,1);
    counter     = 0;
    Q           = zeros(2,8);
    prev_state  = 255;
    cal         = [cal_val(1); cal_val(3)];
end

% Parameters
A  = 154;       % tank cross-section area [cm²]
dt = 12 * 0.8;  % measurement window [s] = 9.6 s

% HIGH→LOW transition: compute Q from previous step and start new sensor_low
if state ~= 255 && prev_state == 255 && 0 < counter && counter < 8
    sensor_high  = [adc1; adc3] .* cal * A;
    Q(:, counter) = (sensor_high - sensor_low) / dt;
end

if state ~= 255 && prev_state == 255
    sensor_low = [adc1; adc3] .* cal * A;
    counter    = counter + 1;
end

% Final step: back to HIGH after step 8 (pump off step)
if counter == 8 && state == 255
    sensor_high  = [adc1; adc3] .* cal * A;
    Q(:, counter) = (sensor_high - sensor_low) / dt;
    finished     = 1;
    counter      = counter + 1;
elseif counter > 8
    finished = 1;
else
    finished = 0;
end

prev_state = state;
Q_out      = Q;
```

#### Sensor-to-volume conversion

The raw ADC voltages are converted to volume (cm³) before the level rise is computed:

```
volume = voltage × cal × A
```

where `cal` is the gain coefficient from sensor calibration (cm/V) and `A = 154 cm²` is the tank cross-section. Multiplying height by area gives the water volume in cm³. Because the pump calibration uses differences (sensor_high − sensor_low), any constant offset in the sensor calibration cancels out — only the gain affects the Q values.

#### Transition detection

The function detects HIGH→LOW transitions by comparing `state` (current step generator output) and `prev_state` (previous step). When `state ~= 255 AND prev_state == 255`, a new LOW phase is beginning.

At each such transition:
1. If a prior LOW phase has been measured (counter ≥ 1), `sensor_high` is captured at the current moment and Q for that step is computed using `sensor_low` from the start of the previous LOW phase.
2. `sensor_low` is immediately updated to the current reading (start of the new LOW phase), and `counter` increments.

The final step (counter = 8, DAC = 0, pumps off) is handled separately: it fires when the pulse returns to HIGH after the zero-flow step, capturing the level at the end of the pumps-off phase.

#### Flow rate output

`Q` is a **2 × 8 matrix**:
- Row 1: flow rate at each DAC step for Pump 1 (Tank 1), in cm³/s
- Row 2: flow rate at each DAC step for Pump 3 (Tank 3), in cm³/s
- Columns 1–8 correspond to DAC steps from ~223 down to 0

`Q_out` is broadcast at every simulation step, but `MaxDataPoints = 1` in the connected `To Workspace` block means only the final value is retained.

---

### StopFcn callback

When the simulation is stopped:

```matlab
Q = out.Q;
save('C:\Users\user\Documents\MATLAB\pump_calibration.mat', 'Q');
```

The variable `Q` (the 2 × 8 flow rate matrix) is extracted from the simulation output object and written to disk.

> ⚠️ **Known issue — hardcoded path:** Same issue as in the sensor calibration model. Replace with:
> ```matlab
> Q = out.Q;
> save(fullfile(pwd, 'pump_calibration.mat'), 'Q');
> ```

---

### Signal and data types

| Signal | Type | Notes |
|---|---|---|
| `adc1`, `adc3` | `double` (0–3.3 V) | Tank 1 and Tank 3 voltage from `sub_com2esp32` |
| `state` (= `y`) | `double` (0–255) | Step generator DAC output; 255 = HIGH phase |
| `cal_val` | `double` (3×1) | Full sensor gain vector; only indices 1 and 3 are used |
| `Q` | `double` (2×8) | Flow rate matrix, cm³/s |
| `finished` | `double` (0 or 1) | Completion flag (currently unused — routed to Terminator) |

---

### How the saved calibration is used downstream

The `pump_calibration.mat` file stores the 2 × 8 flow rate matrix `Q`. A control model loading this file can linearly interpolate between the 8 measured (DAC, Q) pairs to find the command voltage needed to deliver a target flow rate for each pump.
