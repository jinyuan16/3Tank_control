# `calibration_valves.slx` — Valve (Outflow Coefficient) Calibration

## Overview

The connecting valves between tanks and the tank outlet behave as hydraulic orifices: the flow rate through them depends on the water head difference across the opening and on an effective discharge coefficient (`az`) that accounts for the real geometry and flow losses of each valve.

This model estimates the `az` coefficient for each of the three flow paths in the cascade — T1→T3, T3→T2, and the T2 outflow — by observing how the water levels decay naturally under gravity while the pumps are idle. It fits the observed level-change rates to Torricelli's law using a running mean over a 60-second measurement window.

The resulting three `az` values are used in control models to compute the actual flow through each valve given the current water levels.

> **Prerequisite:** Sensor calibration must be completed first. This model loads `sensor_calibration.mat` at startup. Pump calibration is not required.

> **Output:** `az_calibration.mat` — loaded automatically by control models at startup.

> ⚠️ **Known bug — see developer section.** All three sensors currently use the calibration gain of sensor 1. If sensor gains differ significantly between tanks, the computed `az` values will carry a systematic error. Verify and fix before relying on results. See [Sensor gain indexing bug](#sensor-gain-indexing-bug) for the one-line fix.

---

## Part 1 — Operator Instructions

### Physical setup

The model calibrates the valves by measuring a **free gravity drainage** event. The pumps are idle throughout; water flows purely under its own weight through the connecting valves from the highest tank down to the lowest. The setup must create a cascade from T1 through T3 to T2, with T2 draining to the outlet.

Configure the physical system as follows before starting the simulation:

1. **Open the connecting valve between T1 and T3.**
2. **Open the connecting valve between T3 and T2.**
3. **Open the T2 outlet valve.** This is the drain path for the cascade.
4. **Close the T1 outlet valve.**
5. **Close both T3 outlet valves.**

> ⚠️ **Verify this valve layout against the physical hardware.** The model assumes the flow cascade is T1 → T3 → T2 → drain. Confirm that this matches the actual valve positions on your DTS200 unit before running.

6. Fill the tanks to a level that gives a good 60-second gravity drainage signal. Two separate constraints apply:

   - **Hard minimum (code guard):** The Start button is ignored unless all three tanks are above **5 cm** at the moment it is pressed. This is a safety floor, not a target.
   - **Recommended starting level:** Fill significantly higher than the minimum — approximately **T1 ≈ 40–45 cm, T3 ≈ 30 cm, T2 ≈ 20 cm**. Higher head produces stronger, cleaner Torricelli flows, which reduces noise in the finite-difference level derivatives and produces more valid samples in the running mean. Starting near 5 cm gives marginal flow rates that are easily overwhelmed by sensor noise.

   The cascade direction (T1 highest, T2 lowest) must hold throughout the full 60-second measurement — check that no tank empties before the lamp signals completion. If T2 drains too quickly, reduce the outlet valve opening slightly or start at a lower initial level.

---

### Procedure

1. Open `calibration_valves.slx` in Simulink and click **Run**. The Dashboard opens automatically.
2. Confirm on the scope that all three tank levels are above 5 cm and decreasing (water is flowing through the open valves).
3. Click the **Start** button on the Dashboard.
   - If any tank is below 5 cm, the button press is ignored. Top up the tanks and try again.
   - Once accepted, the lamp changes state to indicate measurement is in progress.
4. **Do not interact with the system** for 60 seconds. The model collects level-decay data and continuously refines the `az` estimates.
5. When the **lamp changes to its final state** (measurement complete), stop the simulation by clicking **Stop** in Simulink.

The `az` coefficients are saved automatically to `az_calibration.mat` when the simulation stops.

---

### Dashboard reference

| Widget | Purpose |
|---|---|
| **Start** button | Starts the 60-second measurement window (requires all tanks > 5 cm) |
| Lamp | Off = waiting for Start; active = measurement running; final state = complete (60 s elapsed) |
| Scope | Real-time display of Tank 1 sensor signal |

---

### What to do if something goes wrong

**Button press does nothing:** At least one tank is at or below 5 cm. The guard condition `h1 > 5 && h2 > 5 && h3 > 5` must be satisfied at the moment the button is pressed. Check the scope and top up any tank that is too low, then try again.

**Level not decreasing after setup:** A connecting valve may still be closed, or the T2 outlet may be closed. Check each valve in the cascade path.

**Level decreasing too fast (< 60 s until a tank empties):** The outlet valve may be too far open, or the starting levels in the lower tanks are too low relative to the flow rate. Partially close the T2 outlet valve or reduce the initial level difference between tanks and repeat.

**`sensor_calibration.mat` not found on startup:** Run `calibration_sensors.slx` first.

---

## Part 2 — Code Logic

This section is intended for developers maintaining or extending the calibration model.

---

### InitFcn callback

```matlab
load('C:\Users\user\Documents\MATLAB\sensor_calibration.mat');
coefficients = [0, coefficients];
```

Identical pattern to the pump calibration model: loads the sensor gain vector and prepends a zero time column so the `From Workspace` block serves it as a time-invariant constant. See the pump calibration README for a full explanation of this pattern.

> ⚠️ **Known issue — hardcoded path.** Replace with:
> ```matlab
> load(fullfile(pwd, 'sensor_calibration.mat'));
> coefficients = [0, coefficients];
> ```

---

### Model structure

```
[Constant 255]    ──────────────────────────────────→ dac1 (pump 1, idle)
[Constant 255]    ──────────────────────────────────→ dac2 (pump 2, idle)

[Constant1  = 0]  ──────────────────────────────────→ button input (default; Dashboard push button overrides)
[From Workspace]  ──────────────────────────────────→ cal_val input (sensor gains)
[Clock]           ──────────────────────────────────→ time input

[sub_com2esp32] → adc1 (Tank 1) ──────────────────→  ┐
[sub_com2esp32] → adc2 (Tank 3) ──────────────────→  ├→ [MATLAB Function: calibrate_az]
[sub_com2esp32] → adc3 (Tank 2) ──────────────────→  ┘
                                                              │
                                                    az ──────→ [To Workspace]   → saved on stop
                                                    lamp ────→ [Terminator]     → Dashboard binding
                                                    n_out ──→  (unconnected)
```

**Both pump DAC outputs are held at 255.** In the DTS200/ESP32 system, DAC value 255 corresponds to 3.3 V, which is the idle state for the pump controllers (no pump flow). This ensures the calibration measures pure gravity drainage without any pump contribution.

**`Constant1 = 0`** drives the `button` input to zero by default. The Dashboard push button widget overrides this to 1 when pressed, triggering the measurement start.

---

### MATLAB Function — `calibrate_az`

The full function implements a physics-based estimator using Torricelli's law for orifice flow:

```matlab
function [az, lamp, n_out] = calibrate_az(button, adc1, adc2, adc3, cal_val, time)

persistent running t_start h1p h2p h3p tp m1 m2 m3 n az_out

az    = zeros(3,1);
lamp  = 0.0;
n_out = 0.0;

if isempty(running)
    running = 0.0;  t_start = 0.0;
    h1p = 0.0; h2p = 0.0; h3p = 0.0;
    tp  = 0.0;
    m1  = 0.0; m2  = 0.0; m3  = 0.0;
    n   = 0.0;
    az_out = zeros(3,1);
end

% Parameters
A  = 154.0;   % tank cross-section area [cm²]
Sn = 0.5;     % nominal orifice area [cm²]
g  = 981.0;   % gravitational acceleration [cm/s²]

% Convert voltages to heights — NOTE: see sensor gain indexing bug below
h1 = adc1(1) * cal_val(1);   % Tank 1 height [cm]
h2 = adc3(1) * cal_val(1);   % Tank 2 height [cm]  ← bug: should use cal_val(2)
h3 = adc2(1) * cal_val(1);   % Tank 3 height [cm]  ← bug: should use cal_val(3)
t  = time(1);
b  = button(1);

% Only start if button pressed AND all tanks sufficiently filled
if b == 1 && running == 0 && h1 > 5 && h2 > 5 && h3 > 5
    running = 1.0;
    t_start = t;
    h1p = h1; h2p = h2; h3p = h3;
    tp  = t;
    m1 = 0.0; m2 = 0.0; m3 = 0.0;
    n  = 0.0;
end

if running == 1
    lamp = 1.0;
    dt   = t - tp;

    if dt > 0
        % Finite-difference level derivatives [cm/s]
        dh1 = (h1 - h1p) / dt;
        dh2 = (h2 - h2p) / dt;
        dh3 = (h3 - h3p) / dt;

        % Midpoint head levels [cm]
        h1m = (h1 + h1p) / 2;
        h2m = (h2 + h2p) / 2;
        h3m = (h3 + h3p) / 2;

        % Torricelli nominal flows (unit-az) [cm³/s]
        Q13 = Sn * sqrt(2*g * max(h1m - h3m, 0));
        Q32 = Sn * sqrt(2*g * max(h3m - h2m, 0));
        Q20 = Sn * sqrt(2*g * max(h2m,       0));

        % Only use samples where all three flows are physically active
        if Q13 > 0.001 && Q32 > 0.001 && Q20 > 0.001
            n = n + 1;

            % Sequential mass-balance solve for az coefficients
            az1 = (-dh1 * A) / Q13;
            az3 = (az1 * Q13 - dh3 * A) / Q32;
            az2 = (az3 * Q32 - dh2 * A) / Q20;

            % Running mean update
            m1 = (m1*(n-1) + az1) / n;
            m3 = (m3*(n-1) + az3) / n;
            m2 = (m2*(n-1) + az2) / n;
        end

        az_out = [m1; m2; m3];
        h1p = h1; h2p = h2; h3p = h3;
        tp  = t;
    end

    if (t - t_start) >= 60
        running = 2.0;
    end
end

if running == 2
    lamp = 2.0;
end

az    = az_out;
n_out = n;
```

#### Physical model

The three flow paths follow Torricelli's law. The actual volumetric flow through each path is:

```
Q_actual = az × Sn × sqrt(2 × g × Δh)
```

where `az` is the dimensionless discharge coefficient being estimated, `Sn = 0.5 cm²` is the nominal orifice cross-section, and `Δh` is the head difference driving the flow.

The cascade modelled:

| Variable | Flow path | Head driving it |
|---|---|---|
| `az1` | T1 → T3 (connecting valve) | h1 − h3 |
| `az3` | T3 → T2 (connecting valve) | h3 − h2 |
| `az2` | T2 → drain (outlet valve) | h2 |

#### Sequential mass-balance solver

At each time step, the three `az` values are solved sequentially from the mass balance on each tank. Starting from T1 (which has no inflow while pumps are off):

```
T1 mass balance:  A × dh1/dt = −az1 × Q13_nominal
  → az1 = (−dh1 × A) / Q13_nominal

T3 mass balance:  A × dh3/dt = az1×Q13 − az3×Q32_nominal
  → az3 = (az1×Q13 − dh3×A) / Q32_nominal

T2 mass balance:  A × dh2/dt = az3×Q32 − az2×Q20_nominal
  → az2 = (az3×Q32 − dh2×A) / Q20_nominal
```

Note that T3 and T2 each substitute the previously solved `az` value, making this a sequential (not simultaneous) solve. Noise in `dh/dt` at any tank propagates forward into later estimates; T2's `az2` is therefore noisier than T1's `az1`.

#### Running mean

Each valid sample is incorporated into a running mean rather than being stored as a time series:

```
m = (m × (n−1) + new_value) / n
```

This accumulates all valid estimates over the 60-second window without storing a growing array. Samples where any of the three nominal flows is near zero (`< 0.001`) are excluded — these correspond to nearly equal head levels where the Torricelli denominator is unreliable.

#### Lamp states

| `lamp` value | Meaning |
|---|---|
| `0` | Idle — waiting for Start button press |
| `1` | Measurement running — 60-second window active |
| `2` | Complete — 60 seconds elapsed, final `az` values frozen |

The lamp signal is routed to a `Terminator` in the block diagram. The Dashboard lamp widget binds to this signal via Simulink's Dashboard binding mechanism (not through a direct wire connection).

#### `n_out`

The third output `n_out` carries the count of valid measurement samples used in the running mean. It is currently **unconnected** in the block diagram (no downstream block or To Workspace). It can be wired to a `To Workspace` block for diagnostics if needed — a low `n` value at the end of the run indicates that most samples were filtered out (possibly due to near-equal tank levels or noisy sensor signals).

---

### To Workspace and StopFcn

```matlab
% StopFcn
az = out.az;
save('C:\Users\user\Documents\MATLAB\az_calibration.mat', 'az');
```

`az` is a 3×1 vector saved with `MaxDataPoints = 1` (last value only). The three elements correspond to:
- `az(1)` = discharge coefficient for the T1→T3 valve
- `az(2)` = discharge coefficient for the T2 outlet
- `az(3)` = discharge coefficient for the T3→T2 valve

> ⚠️ **Known issue — hardcoded path.** Replace with:
> ```matlab
> az = out.az;
> save(fullfile(pwd, 'az_calibration.mat'), 'az');
> ```

---

### Sensor gain indexing bug

In the height conversion lines, all three sensors use `cal_val(1)` — the gain from sensor 1:

```matlab
h1 = adc1(1) * cal_val(1);   % correct: sensor 1 gain
h2 = adc3(1) * cal_val(1);   % bug: should be cal_val(2) for Tank 2
h3 = adc2(1) * cal_val(1);   % bug: should be cal_val(3) for Tank 3
```

`cal_val` is the 3-element sensor gain vector `[c1, c2, c3]` delivered by the `From Workspace` block. Using the wrong gain for T2 and T3 means the heights `h2` and `h3` are miscalibrated by the ratio of sensor 1's gain to the actual sensor gain. Since `az` is estimated from ratios of level derivatives, a constant multiplicative error in height partially cancels in the derivatives — but not completely, because the Torricelli terms (`sqrt(Δh)`) are non-linear. The resulting `az` values will carry a systematic offset.

**Fix:**

```matlab
h1 = adc1(1) * cal_val(1);
h2 = adc3(1) * cal_val(2);   % use Tank 2 gain
h3 = adc2(1) * cal_val(3);   % use Tank 3 gain
```

Note that `cal_val` indexing here matches the calibration output order from `calibration_sensors.slx`, where the three gains are stored as `coefficients = [c1, c2, c3]` corresponding to adc1, adc2, adc3 respectively. The signal mapping (adc2↔Tank3, adc3↔Tank2) means the index must follow the calibration ordering, not the tank number.

---

### Signal and data types

| Signal | Source | Type | Notes |
|---|---|---|---|
| `adc1` | `sub_com2esp32` out:1 | `double` (0–3.3 V) | Tank 1 sensor voltage |
| `adc2` | `sub_com2esp32` out:2 | `double` (0–3.3 V) | Tank 3 sensor voltage (labeled "Tank 3" in subsystem) |
| `adc3` | `sub_com2esp32` out:3 | `double` (0–3.3 V) | Tank 2 sensor voltage (labeled "Tank 2" in subsystem) |
| `button` | Constant1 (0) / Dashboard | `double` (0 or 1) | Start trigger |
| `cal_val` | From Workspace | `double` (3×1) | Sensor gain vector from sensor calibration |
| `time` | Clock | `double` (s) | Simulation wall time |
| `az` | MATLAB Function out:1 | `double` (3×1) | Discharge coefficients (dimensionless) |
| `lamp` | MATLAB Function out:2 | `double` (0, 1, or 2) | Measurement state |
| `n_out` | MATLAB Function out:3 | `double` | Valid sample count — unconnected, for debug use |
