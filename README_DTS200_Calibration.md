# DTS200 Three-Tank-System — Outflow Coefficient Calibration

## System Overview

The DTS200 is a laboratory three-tank system by amira GmbH consisting of three
plexiglas cylinders T1, T2, T3 connected in series via fixed cylindrical orifices.
Two pumps supply flow Q1 and Q2 into T1 and T2 respectively.

### Physical Parameters (from manual, Chapter 3)

| Parameter | Value | Unit |
|-----------|-------|------|
| Tank cross-section A | 154 | cm² |
| Orifice cross-section Sn | 0.5 | cm² |
| Max liquid level Hmax | 62 | cm |
| Gravitational acceleration g | 981 | cm/s² |

### Flow Topology

**T1 → T3 → T2 → Reservoir → Pumps → T1/T2**

Flow between tanks follows the generalised Torricelli rule (Eq. 3.5):

```
q = az * Sn * sgn(Δh) * sqrt(2g|Δh|)
```

Where `az` is the dimensionless outflow coefficient (0 < az < 1) for each orifice.

### ADC Mapping (hardware-specific)

| Software variable | Physical tank | ADC channel |
|-------------------|---------------|-------------|
| h1 | T1 | adc1 |
| h2 | T2 | adc3 |
| h3 | T3 | adc2 |

---

## Mathematical Model

With pumps off (Q1 = Q2 = 0), the three ODEs are:

```
A * dh1/dt = -az1 * Q13_hat
A * dh3/dt =  az1 * Q13_hat - az3 * Q32_hat
A * dh2/dt =  az3 * Q32_hat - az2 * Q20_hat
```

Where the potential flows (without az) are:

```
Q13_hat = Sn * sqrt(2g * max(h1 - h3, 0))
Q32_hat = Sn * sqrt(2g * max(h3 - h2, 0))
Q20_hat = Sn * sqrt(2g * max(h2,       0))
```

Solving sequentially for each az:

```
az1 = (-A * dh1) / Q13_hat
az3 = (az1 * Q13_hat - A * dh3) / Q32_hat
az2 = (az3 * Q32_hat - A * dh2) / Q20_hat
```

---

## Calibration Procedure

### Prerequisites

- All pumps must be **OFF** during calibration
- All three tanks must have **different water levels**: T1 high, T3 medium, T2 low
- Minimum level of **5 cm** in each tank required before starting
- Recommended starting conditions: T1 ~30cm, T3 ~20cm, T2 ~10cm

### Procedure

1. Fill tanks to appropriate levels with pumps
2. Switch pumps off
3. Press the calibration button in the Simulink dashboard
4. Wait 60 seconds — lamp turns green (=1) during measurement
5. Lamp turns to 2 when finished — az values are locked in
6. Save results via File → Save System Parameters (*.CAL file)

### What happens internally

Each timestep during the 60-second window:
- Finite differences compute dh1/dt, dh2/dt, dh3/dt
- Midpoint levels are used for Q calculations
- az1, az2, az3 are computed from the ODEs
- A running mean is updated:  `mean_n = (mean_(n-1) * (n-1) + az_n) / n`
- az_out is updated every timestep so convergence is visible on scope

---

## Simulink Implementation

### MATLAB Function Block: `calibrate_az`

**Inputs:**

| Port | Signal | Description |
|------|--------|-------------|
| button | scalar | Push button, 1 on press |
| adc1 | scalar | Sensor voltage T1 [V] |
| adc2 | scalar | Sensor voltage T3 [V] |
| adc3 | scalar | Sensor voltage T2 [V] |
| cal_val | scalar | Calibration factor [cm/V] |
| time | scalar | Simulation time from Clock block [s] |

**Outputs:**

| Port | Signal | Description |
|------|--------|-------------|
| az | 3x1 vector | [az1, az2, az3] outflow coefficients |
| lamp | scalar | 0=idle, 1=running, 2=finished |
| n_out | scalar | Number of valid samples used |

### Model Callbacks

In **Modeling → Model Settings → Callbacks → InitFcn**:

```matlab
clear calibrate_az
```

This clears persistent variables at every simulation start to ensure
a fresh calibration each run.

---

## Complete MATLAB Function Code

```matlab
function [az, lamp, n_out] = calibrate_az(button, adc1, adc2, adc3, cal_val, time)

persistent running t_start h1p h2p h3p tp m1 m2 m3 n az_out

az    = zeros(3,1);
lamp  = 0.0;
n_out = 0.0;

if isempty(running)
    running = 0.0;
    t_start = 0.0;
    h1p    = 0.0; h2p = 0.0; h3p = 0.0;
    tp     = 0.0;
    m1     = 0.0; m2  = 0.0; m3  = 0.0;
    n      = 0.0;
    az_out = zeros(3,1);
end

% Parameters all in cm
A  = 154.0;
Sn = 0.5;
g  = 981.0;

% T1=adc1, T2=adc3, T3=adc2
h1 = adc1(1) * cal_val(1);
h2 = adc3(1) * cal_val(1);
h3 = adc2(1) * cal_val(1);
t  = time(1);
b  = button(1);

% Only start if all tanks sufficiently filled
if b == 1 && running == 0 && h1 > 5 && h2 > 5 && h3 > 5
    running = 1.0;
    t_start = t;
    h1p = h1; h2p = h2; h3p = h3;
    tp  = t;
    m1  = 0.0; m2 = 0.0; m3 = 0.0;
    n   = 0.0;
end

if running == 1
    lamp = 1.0;
    dt   = t - tp;

    if dt > 0
        dh1 = (h1 - h1p) / dt;
        dh2 = (h2 - h2p) / dt;
        dh3 = (h3 - h3p) / dt;

        h1m = (h1 + h1p) / 2;
        h2m = (h2 + h2p) / 2;
        h3m = (h3 + h3p) / 2;

        % Potential flows (without az)
        Q13 = Sn * sqrt(2*g * max(h1m - h3m, 0));
        Q32 = Sn * sqrt(2*g * max(h3m - h2m, 0));
        Q20 = Sn * sqrt(2*g * max(h2m,       0));

        if Q13 > 0.001 && Q32 > 0.001 && Q20 > 0.001
            n   = n + 1;

            az1 = (-dh1*A) / Q13;
            az3 = (az1*Q13 - dh3*A) / Q32;
            az2 = (az3*Q32 - dh2*A) / Q20;

            m1 = (m1*(n-1) + az1) / n;
            m3 = (m3*(n-1) + az3) / n;
            m2 = (m2*(n-1) + az2) / n;
        end

        az_out(1) = m1;
        az_out(2) = m2;
        az_out(3) = m3;

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

---

## Known Issues & Outstanding Questions

- az values currently converging above 1.0 — physically should be 0.6-0.8
- Suspected cause: cal_val units or sensor scaling not verified end-to-end
- Recommended next step: verify that `adc * cal_val` actually gives cm by
  comparing to a manual ruler measurement of tank level

---

## References

- amira GmbH, *DTS200 Laboratory Setup Three-Tank-System*, 2002
  - Chapter 3, p.3-1 to 3-4: System description and mathematical model
  - Chapter 3, Figure 3.3: Flow topology and variable definitions
- amira GmbH, *DTS200 Windows Software V1.3*, 2002
  - Section 1.5.3, p.1-51 to 1-54: OFC class documentation
  - Chapter 2, p.2-10 to 2-11: OfcEichStart/Stop driver functions
