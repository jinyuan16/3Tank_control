function xNext = tankEnvironment(x,u,az)

%% ========================================
% Three-tank nonlinear model
% One simulation step: Ts = 0.05 s
% =========================================

A    = 154.0;
Sn   = 0.5;
g    = 981.0;
Ts   = 0.05;
Qmax = 85.0;

%% Current water levels

h1 = max(x(1),0);
h2 = max(x(2),0);
h3 = max(x(3),0);

%% Pump inputs

q1 = min(max(u(1),0),Qmax);
q2 = min(max(u(2),0),Qmax);

%% Flow coefficients

C1 = az(1)*Sn;
C2 = az(3)*Sn;
C3 = az(2)*Sn;

%% Water-level differences

d13 = h1 - h3;
d32 = h3 - h2;

%% Internal flows

q13 = C1 * sign(d13) * sqrt(2*g*abs(d13));
q32 = C2 * sign(d32) * sqrt(2*g*abs(d32));
q20 = C3 * sqrt(2*g*h2);

%% Tank dynamics

dh1 = (q1 - q13)/A;

dh2 = (q2 + q32 - q20)/A;

dh3 = (q13 - q32)/A;

%% Euler integration

xNext = x + Ts*[dh1;dh2;dh3];

%% Physical lower bound

xNext = max(xNext,0);

end