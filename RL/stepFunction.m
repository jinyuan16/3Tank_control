function [NextObservation, Reward, IsDone, LoggedSignals] = ...
    stepFunction(Action, LoggedSignals)

%% ========================================
% Parameters
% =========================================

az = [0.5 0.5 0.5];

Qmax = 85;

% Tank model sampling time
TsPlant = 0.05;

% RL controller sampling time
TsAgent = 0.5;

% Number of plant simulation steps
Nsub = round(TsAgent/TsPlant);

%% ========================================
% Current state and reference
% =========================================

x = LoggedSignals.State;

ref = LoggedSignals.Reference;

%% ========================================
% 1. SAC normalized action
%
% Action range:
% [-1,1]
% =========================================

a = double(Action(:));

a = min(max(a,-1),1);

%% ========================================
% 2. Convert normalized action
%    to physical pump flow
%
% -1 -> 0
%  0 -> 42.5
% +1 -> 85
% =========================================

u = (Qmax/2)*(a + 1);

%% ========================================
% 3. Run tank model for 0.5 seconds
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
% 5. Reward
%
% Tank 3 receives larger weight
% =========================================

eNorm = e/20;

Reward = -( ...
      eNorm(1)^2 ...
    + eNorm(2)^2 ...
    + 2*eNorm(3)^2 );

%% ========================================
% 6. Next observation
% =========================================

NextObservation = [
    xNext/60;
    e/30
];

%% ========================================
% 7. Save state
% =========================================

LoggedSignals.State = xNext;

%% ========================================
% 8. Safety termination
% =========================================

IsDone = any(xNext > 60);

end