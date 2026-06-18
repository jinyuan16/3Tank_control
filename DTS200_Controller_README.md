# DTS200 Three-Tank Controller README

This document summarizes the controller implementations used in the Simulink controller subsystem, from the default/no-control case through PI, input-output linearization, and backstepping controllers.

The tank connection assumed here is:

```text
Pump q1 -> Tank 1 -> Tank 3 -> Tank 2 <- Pump q2
                         |
                      outlet
```

The controlled tank levels are:

```text
h1 = Tank 1 level
h2 = Tank 2 level
h3 = Tank 3 level
```

The reference levels are:

```text
r1 = Tank 1 reference
r2 = Tank 2 reference
r3 = Tank 3 reference
```

The pump commands are:

```text
q1 = Pump 1 command
q2 = Pump 2 command
```

---

## 1. Physical Model

The three-tank model is based on mass balance:

\[
A \dot h_1 = q_1 - q_{13}
\]

\[
A \dot h_3 = q_{13} - q_{32}
\]

\[
A \dot h_2 = q_2 + q_{32} - q_{20}
\]

where:

```text
A  = tank cross-sectional area
Sn = valve/orifice area
g  = gravity
```

The nonlinear flows are modelled by Torricelli's law:

\[
q_{13}=az_1 S_n \sqrt{2g(h_1-h_3)}
\]

\[
q_{32}=az_3 S_n \sqrt{2g(h_3-h_2)}
\]

\[
q_{20}=az_2 S_n \sqrt{2gh_2}
\]

In implementation, `max(...,0)` or `abs(...)` is used to avoid invalid square roots.

The calibrated valve coefficients are:

```text
az(1) = coefficient for Tank1 -> Tank3
az(2) = coefficient for Tank2 -> outlet
az(3) = coefficient for Tank3 -> Tank2
```

Typical constants used in the model:

```matlab
A  = 154.0;   % cm^2
Sn = 0.5;     % cm^2
g  = 981.0;   % cm/s^2
Ts = 0.01;    % 100 Hz ESP32 sampling rate
```

---

# 2. Default / None Controller

The default controller applies no pump action:

```matlab
q1 = 0;
q2 = 0;
```

This is useful as a safe fallback mode.

---

# 3. Open Loop Controller

The open-loop controller sends the command directly to the pumps:

```matlab
q1 = Out1;
q2 = Out2;
```

There is no feedback from tank levels.

---

# 4. PI SISO Controller

## 4.1 Purpose

PI SISO controls Tank 2 only:

```text
controlled output: h2
input used: q2
q1 = 0
```

The error is:

\[
e_2(k)=r_2(k)-h_2(k)
\]

The controller output is:

\[
q_2(k)=PI(e_2)
\]

## 4.2 Continuous-Time PI

The continuous PI controller is:

\[
u(t)=K_p e(t)+K_i\int e(t)\,dt
\]

The C++ implementation uses a Tustin-discretized incremental PI.

## 4.3 Tustin Discretization

Using the trapezoidal approximation:

\[
\int_{k-1}^{k}e(t)\,dt
\approx
\frac{T_s}{2}\left(e(k)+e(k-1)\right)
\]

The incremental PI law becomes:

\[
u(k)=u(k-1)+\left(K_p+\frac{K_iT_s}{2}\right)e(k)+\left(\frac{K_iT_s}{2}-K_p\right)e(k-1)
\]

Define:

\[
z_{k0}=K_p+\frac{K_iT_s}{2}
\]

\[
z_{k1}=\frac{K_iT_s}{2}-K_p
\]

Then:

\[
u(k)=u(k-1)+z_{k0}e(k)+z_{k1}e(k-1)
\]

## 4.4 MATLAB Function

```matlab
function [q1,q2] = PI_SISO(h2,r2,Kp,Ki)
%#codegen

persistent u_old e_old

if isempty(u_old)
    u_old = 0;
    e_old = 0;
end

Ts = 0.01;

e = r2 - h2;

k = Ki * Ts / 2;
zk0 = Kp + k;
zk1 = k - Kp;

u = u_old + zk0 * e + zk1 * e_old;
u = min(max(u,-100),100);

q1 = 0;
q2 = u;

u_old = u;
e_old = e;

end
```

---

# 5. PI MIMO Controller

## 5.1 Purpose

The original C++ PI MIMO mode is named `PICONTROLLER_T1T3`.

It controls:

```text
Tank 1 using Pump q1
Tank 3 indirectly using Pump q2
```

The structure is:

\[
e_1=r_1-h_1
\]

\[
e_3=r_3-h_3
\]

\[
q_1=PI_1(e_1)
\]

\[
q_2=PI_3(e_3)
\]

This is not a fully coupled MIMO controller. It is better understood as two independent SISO PI controllers acting on a coupled MIMO plant.

## 5.2 Why q2 Controls Tank 3

Pump 2 physically acts on Tank 2, but Tank 2 changes the flow from Tank 3 to Tank 2:

\[
q_{32}=az_3S_n\sqrt{2g(h_3-h_2)}
\]

So changing `q2` changes `h2`, which changes `q32`, which affects `h3`.

The control path is:

```text
q2 -> h2 -> q32 -> h3
```

## 5.3 MATLAB Function

```matlab
function [q1, q2] = PI_MIMO(h1,h3,r1,r3,Kp1,Ki1,Kp3,Ki3)
%#codegen

persistent q1_old e1_old
persistent q2_old e3_old

if isempty(q1_old)
    q1_old = 0;
    e1_old = 0;
    q2_old = 0;
    e3_old = 0;
end

Ts = 0.01;

% PI1 controls Tank 1
e1 = r1 - h1;

k1 = Ki1 * Ts / 2;
zk10 = Kp1 + k1;
zk11 = k1 - Kp1;

q1_new = q1_old + zk10 * e1 + zk11 * e1_old;
q1_new = min(max(q1_new, -100), 100);

% PI3 controls Tank 3 through Pump 2
e3 = r3 - h3;

k3 = Ki3 * Ts / 2;
zk30 = Kp3 + k3;
zk31 = k3 - Kp3;

q2_new = q2_old + zk30 * e3 + zk31 * e3_old;
q2_new = min(max(q2_new, -100), 100);

q1 = q1_new;
q2 = q2_new;

q1_old = q1_new;
e1_old = e1;

q2_old = q2_new;
e3_old = e3;

end
```

---

# 6. Input-Output Linearization SISO

## 6.1 Purpose

IO Linearization SISO controls Tank 2 only:

```text
controlled output: h2
input: q2
q1 = 0
```

Tank 2 dynamics:

\[
A\dot h_2=q_2-q_{20}
\]

where:

\[
q_{20}=az_2S_n\sqrt{2gh_2}
\]

Thus:

\[
A\dot h_2=q_2-az_2S_n\sqrt{2gh_2}
\]

This is nonlinear because of \(\sqrt{h_2}\).

## 6.2 Linearizing Control Law

Choose:

\[
q_2=A v+az_2S_n\sqrt{2gh_2}
\]

Substitute into the plant:

\[
A\dot h_2=
\left(A v+az_2S_n\sqrt{2gh_2}\right)
-
az_2S_n\sqrt{2gh_2}
\]

The nonlinear terms cancel:

\[
A\dot h_2=Av
\]

\[
\dot h_2=v
\]

Now choose:

\[
v=K_p(r_2-h_2)
\]

Then the closed-loop system is:

\[
\dot h_2=K_p(r_2-h_2)
\]

This is a linear first-order error system.

## 6.3 MATLAB Function

```matlab
function [q1,q2] = IO_SISO(h2,r2,Kp,az2)
%#codegen

A  = 154.0;
Sn = 0.5;
g  = 981.0;

e = r2 - h2;

q1 = 0;
q2 = A*Kp*e + az2*Sn*sqrt(2*g*max(h2,0));

q2 = min(max(q2,-100),100);

end
```

---

# 7. Input-Output Linearization MIMO

## 7.1 Purpose

IO MIMO controls:

```text
h1 using q1
h3 using q2 indirectly
```

The outputs are:

\[
y_1=h_1
\]

\[
y_2=h_3
\]

The inputs are:

\[
u_1=q_1,\quad u_2=q_2
\]

Tank 1 has relative degree 1 because `q1` appears in \(\dot h_1\).

Tank 3 has relative degree 2 because `q2` does not appear in \(\dot h_3\), but does appear in \(\ddot h_3\).

## 7.2 Tank 1 Channel

Tank 1 dynamics:

\[
A\dot h_1=q_1-q_{13}
\]

Choose:

\[
q_1=A K_1(r_1-h_1)+q_{13}
\]

Then:

\[
\dot h_1=K_1(r_1-h_1)
\]

The original C++ used a modified coupling compensation:

\[
q_1=A K_1(r_1-h_1)+2q_{13}-q_{32}
\]

For a theoretical clean version, use `+q13`. For closer C++ behavior, use `+2*q13-q32`.

## 7.3 Tank 3 Channel Derivation

Tank 3 dynamics:

\[
A\dot h_3=q_{13}-q_{32}
\]

Since \(q_2\) does not appear, differentiate:

\[
A\ddot h_3=\dot q_{13}-\dot q_{32}
\]

For:

\[
q_{13}=az_1S_n\sqrt{2g(h_1-h_3)}
\]

\[
\dot q_{13}=k_{13}(\dot h_1-\dot h_3)
\]

where:

\[
k_{13}=\frac{az_1S_n(2g)}{2\sqrt{2g(h_1-h_3)}}
\]

Using:

\[
\dot h_1=\frac{q_1-q_{13}}{A}
\]

\[
\dot h_3=\frac{q_{13}-q_{32}}{A}
\]

gives:

\[
\dot h_1-\dot h_3=
\frac{q_1-2q_{13}+q_{32}}{A}
\]

Therefore:

\[
\dot q_{13}=\frac{k_{13}}{A}(q_1-2q_{13}+q_{32})
\]

Similarly, for:

\[
q_{32}=az_3S_n\sqrt{2g(h_3-h_2)}
\]

\[
\dot q_{32}=k_{32}(\dot h_3-\dot h_2)
\]

where:

\[
k_{32}=\frac{az_3S_n(2g)}{2\sqrt{2g(h_3-h_2)}}
\]

Using:

\[
\dot h_2=\frac{q_2+q_{32}-q_{20}}{A}
\]

gives:

\[
\dot h_3-\dot h_2=
\frac{q_{13}-2q_{32}+q_{20}-q_2}{A}
\]

So:

\[
\dot q_{32}=\frac{k_{32}}{A}(q_{13}-2q_{32}+q_{20}-q_2)
\]

Substitute into:

\[
A\ddot h_3=\dot q_{13}-\dot q_{32}
\]

Then solve for \(q_2\). This produces the IO linearizing input.

## 7.4 Linear Error Dynamics for h3

After linearization, choose:

\[
\ddot h_3=v_3
\]

For constant reference \(r_3\), use PD-type dynamics:

\[
v_3=K_3(r_3-h_3)+D_3\frac{d}{dt}(r_3-h_3)
\]

This gives a second-order linear closed-loop response.

## 7.5 MATLAB Function

```matlab
function [q1,q2] = IO_MIMO(h1,h2,h3,r1,r3,K1,K3,D3,az)
%#codegen

A  = 154.0;
Sn = 0.5;
g  = 981.0;
TWOG = 2*g;
epsv = 1e-6;

az1 = az(1);
az2 = az(2);
az3 = az(3);

q13 = az1 * Sn * sqrt(TWOG * max(h1-h3,0));
q32 = az3 * Sn * sqrt(TWOG * max(h3-h2,0));
q20 = az2 * Sn * sqrt(TWOG * max(h2,0));

% h1 channel
e1 = r1 - h1;
q1 = A*K1*e1 + q13;
q1 = min(max(q1,-100),100);

% h3 channel
persistent e3_old

if isempty(e3_old)
    e3_old = 0;
end

Ts = 0.01;

e3 = r3 - h3;
de3 = (e3 - e3_old)/Ts;

v3 = K3*e3 + D3*de3;

dh13 = max(h1-h3,epsv);
dh32 = max(h3-h2,epsv);

k13 = (az1 * Sn * TWOG) / (2 * sqrt(TWOG * dh13));
k32 = (az3 * Sn * TWOG) / (2 * sqrt(TWOG * dh32));

k1 = k13 / k32;

q2 = -k1*(q1 - 2*q13 + q32) ...
     + (q13 - 2*q32 + q20) ...
     + (A*A/k32)*v3;

q2 = min(max(q2,-100),100);

e3_old = e3;

end
```

---

# 8. Backstepping SISO

## 8.1 Purpose

Backstepping SISO controls Tank 2 only.

Tank 2 dynamics:

\[
A\dot h_2=q_2-q_{20}
\]

Define the tracking error:

\[
e_2=h_2-r_2
\]

Choose desired exponential error dynamics:

\[
\dot e_2=-K e_2
\]

Since \(r_2\) is constant:

\[
\dot e_2=\dot h_2
\]

Thus:

\[
\frac{q_2-q_{20}}{A}=-K e_2
\]

Solve for \(q_2\):

\[
q_2=q_{20}-AKe_2
\]

Since \(e_2=h_2-r_2\):

\[
q_2=q_{20}+AK(r_2-h_2)
\]

The original C++ used \(K=1\).

## 8.2 MATLAB Function

```matlab
function [q1,q2] = Backstepping_SISO(h2,r2,az2)
%#codegen

A  = 154.0;
Sn = 0.5;
g  = 981.0;

q20 = az2 * Sn * sqrt(2*g*max(h2,0));

q1 = 0;
q2 = q20 + A*(r2 - h2);

q2 = min(max(q2,-100),100);

end
```

Optional tunable gain version:

```matlab
function [q1,q2] = Backstepping_SISO(h2,r2,K,az2)
%#codegen

A  = 154.0;
Sn = 0.5;
g  = 981.0;

q20 = az2 * Sn * sqrt(2*g*max(h2,0));

q1 = 0;
q2 = q20 + A*K*(r2 - h2);

q2 = min(max(q2,-100),100);

end
```

---

# 9. Backstepping MIMO

## 9.1 Purpose

The original C++ did not include Backstepping MIMO. This controller is a new theoretical design based on the same physical model.

It controls:

```text
h1 with q1
h3 with q2 through q32
```

## 9.2 Tank 1 Step

Define:

\[
e_1=h_1-r_1
\]

Choose:

\[
\dot e_1=-K_1e_1
\]

Since:

\[
\dot e_1=\dot h_1=\frac{q_1-q_{13}}{A}
\]

set:

\[
\frac{q_1-q_{13}}{A}=-K_1e_1
\]

so:

\[
q_1=q_{13}-AK_1e_1
\]

or:

\[
q_1=q_{13}+AK_1(r_1-h_1)
\]

## 9.3 Tank 3 Virtual Control

Define:

\[
e_3=h_3-r_3
\]

Tank 3 dynamics:

\[
A\dot h_3=q_{13}-q_{32}
\]

The actual input \(q_2\) does not appear directly. Treat \(q_{32}\) as a virtual control.

Choose desired dynamics:

\[
\dot e_3=-K_3e_3
\]

Since:

\[
\dot e_3=\dot h_3
\]

require:

\[
\frac{q_{13}-q_{32,d}}{A}=-K_3e_3
\]

Solve for the desired virtual flow:

\[
q_{32,d}=q_{13}+AK_3e_3
\]

If \(q_{32}=q_{32,d}\), then \(h_3\) tracking error decays exponentially.

## 9.4 Virtual Tracking Error

Define:

\[
z=q_{32}-q_{32,d}
\]

The second backstepping step is to design \(q_2\) so that:

\[
z\rightarrow 0
\]

Choose desired dynamics:

\[
\dot z=-K_z z
\]

Since:

\[
\dot z=\dot q_{32}-\dot q_{32,d}
\]

we require:

\[
\dot q_{32}-\dot q_{32,d}=-K_z z
\]

## 9.5 Deriving q2

Using:

\[
\dot q_{32}=\frac{k_{32}}{A}(q_{13}-2q_{32}+q_{20}-q_2)
\]

set:

\[
\frac{k_{32}}{A}(q_{13}-2q_{32}+q_{20}-q_2)-\dot q_{32,d}=-K_z z
\]

Solve for \(q_2\):

\[
q_2=q_{13}-2q_{32}+q_{20}-\frac{A}{k_{32}}\dot q_{32,d}+\frac{A}{k_{32}}K_z z
\]

This is the actual pump command for the second pump.

## 9.6 q32d_dot

Because:

\[
q_{32,d}=q_{13}+AK_3e_3
\]

then:

\[
\dot q_{32,d}=\dot q_{13}+AK_3\dot e_3
\]

For constant reference:

\[
\dot e_3=\dot h_3
\]

So:

\[
\dot q_{32,d}=\dot q_{13}+AK_3\dot h_3
\]

where:

\[
\dot q_{13}=k_{13}(\dot h_1-\dot h_3)
\]

## 9.7 MATLAB Function

```matlab
function [q1,q2] = Backstepping_MIMO(h1,h2,h3,r1,r3,K1,K3,Kz,az)
%#codegen

A  = 154.0;
Sn = 0.5;
g  = 981.0;
TWOG = 2*g;
epsv = 1e-6;

az1 = az(1);
az2 = az(2);
az3 = az(3);

q13 = az1 * Sn * sqrt(TWOG * max(h1-h3,0));
q32 = az3 * Sn * sqrt(TWOG * max(h3-h2,0));
q20 = az2 * Sn * sqrt(TWOG * max(h2,0));

% Step 1: h1 backstepping
e1 = h1 - r1;

q1 = q13 - A*K1*e1;
q1 = min(max(q1,-100),100);

% Step 2: virtual control for h3
e3 = h3 - r3;

q32_des = q13 + A*K3*e3;

z = q32 - q32_des;

% Derivatives needed for q32_des_dot
dh13 = max(h1-h3,epsv);
dh32 = max(h3-h2,epsv);

k13 = (az1 * Sn * TWOG) / (2 * sqrt(TWOG * dh13));
k32 = (az3 * Sn * TWOG) / (2 * sqrt(TWOG * dh32));

h1_dot = (q1 - q13) / A;
h3_dot = (q13 - q32) / A;

q13_dot = k13 * (h1_dot - h3_dot);

q32_des_dot = q13_dot + A*K3*h3_dot;

% Step 3: actual q2
q2 = q13 - 2*q32 + q20 ...
     - (A/k32)*q32_des_dot ...
     + (A/k32)*Kz*z;

q2 = min(max(q2,-100),100);

end
```

---

# 10. Summary of Controller Differences

| Controller | Controlled outputs | Inputs used | Model knowledge | Memory |
|---|---:|---:|---:|---:|
| Default | none | none | no | no |
| Open Loop | none | q1, q2 direct | no | no |
| PI SISO | h2 | q2 | no | yes |
| PI MIMO | h1, h3 | q1, q2 | no | yes |
| IO SISO | h2 | q2 | yes | no |
| IO MIMO | h1, h3 | q1, q2 | yes | yes for derivative |
| Backstepping SISO | h2 | q2 | yes | no |
| Backstepping MIMO | h1, h3 | q1, q2 | yes | no |

PI controllers rely on accumulated error. IO linearization cancels nonlinear terms directly. Backstepping starts by imposing desired error dynamics and recursively derives virtual and real controls.

---

# 11. Recommended Mask Organization

Use one main mask with collapsible groups:

```text
Controller Mode
Reference Levels
PI SISO Parameters
PI MIMO Parameters
IO SISO Parameters
IO MIMO Parameters
Backstepping SISO Parameters
Backstepping MIMO Parameters
```

Recommended parameters:

```text
Reference Levels:
r1, r2, r3

PI SISO:
Kp, Ki

PI MIMO:
Kp1, Ki1, Kp3, Ki3

IO SISO:
Kp_IO

IO MIMO:
K1_IO, K3_IO, D3_IO

Backstepping SISO:
K_BS or fixed K = 1

Backstepping MIMO:
K1_BS, K3_BS, Kz_BS
```

Use `az` from `az_calibration.mat`:

```matlab
load(fullfile(pwd,'az_calibration.mat'));
```

Then either pass:

```matlab
az
```

to MIMO controllers, or pass:

```matlab
az(2)
```

to SISO controllers that only need the Tank 2 outlet coefficient.
