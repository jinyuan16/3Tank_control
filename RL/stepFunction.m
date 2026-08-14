function [NextObservation, Reward, IsDone, LoggedSignals] = ...
    stepFunction(Action, LoggedSignals)

%% ========================================
% Real calibrated parameters
% =========================================

az = [0.4191 0.7860 0.4639];

Qmax = 85.0;

%% ========================================
% Timing
% =========================================

% Physical tank simulation time
TsPlant = 0.05;

% RL controller update time
TsAgent = 0.5;

% Number of plant steps per RL action
Nsub = round(TsAgent/TsPlant);

%% ========================================
% Current state and reference
% =========================================

x = LoggedSignals.State;

ref = LoggedSignals.Reference;

%% ========================================
% 1. SAC normalized action
%
% a1,a2 in [-1,1]
% =========================================

a = double(Action(:));

a = min(max(a,-1),1);

%% ========================================
% 2. Convert normalized action
%    to real pump flow
%
% a = -1 -> Q = 0
% a =  0 -> Q = 42.5
% a = +1 -> Q = 85
% =========================================

u = (Qmax/2)*(a + 1);

%% ========================================
% 3. Simulate tank for 0.5 seconds
% =========================================

xNext = x;

for i = 1:Nsub

    xNext = tankEnvironment( ...
        xNext, ...
        u, ...
        az);

end

%% ========================================
% 4. Tracking error
% =========================================

e = ref - xNext;

%% ========================================
% 5. Tracking cost
% =========================================

eNorm = e/20;

trackingCost = ...
      eNorm(1)^2 ...
    + eNorm(2)^2 ...
    + eNorm(3)^2;

%% ========================================
% 6. Smoothness cost
%
% We DO NOT penalize large pump values.
% We only penalize sudden changes.
% =========================================

if isempty(LoggedSignals.PreviousAction)

    % Do not penalize the first action
    smoothCost = 0;

else

    uPrev = LoggedSignals.PreviousAction;

    du = u - uPrev;

    duNorm = du/Qmax;

    % Smoothness weight
    lambdaDu = 0.2;

    smoothCost = ...
        lambdaDu * sum(duNorm.^2);

end

%% ========================================
% 7. Total reward
% =========================================

Reward = -( ...
    trackingCost ...
    + smoothCost );

%% ========================================
% 8. Save current pump input
% =========================================

LoggedSignals.PreviousAction = u;

%% ========================================
% 9. Next observation
% =========================================

NextObservation = [
    xNext/60;
    e/30
];

%% ========================================
% 10. Save new state
% =========================================

LoggedSignals.State = xNext;

%% ========================================
% 11. Safety termination
% =========================================

IsDone = ...
       any(xNext > 60) ...
    || any(~isfinite(xNext));

end