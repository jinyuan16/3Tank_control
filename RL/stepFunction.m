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

TsPlant = 0.05;

% RL action held for 2 seconds
TsAgent = 2.0;

Nsub = round(TsAgent/TsPlant);

%% ========================================
% Current state and reference
% =========================================

x = LoggedSignals.State;

ref = LoggedSignals.Reference;

%% ========================================
% Current tracking error BEFORE action
% =========================================

eOld = ref - x;

%% ========================================
% 1. SAC normalized action
% =========================================

a = double(Action(:));

a = min(max(a,-1),1);

%% ========================================
% 2. Convert normalized action to pump flow
% =========================================

u = (Qmax/2)*(a + 1);

%% ========================================
% 3. Simulate tank for one RL step
% =========================================

xNext = x;

for i = 1:Nsub

    xNext = tankEnvironment( ...
        xNext, ...
        u, ...
        az);

end

%% ========================================
% 4. New tracking error
% =========================================

eNew = ref - xNext;

%% ========================================
% 5. Normalized errors
% =========================================

% Error scale
errorScale = 20;

eOldNorm = eOld/errorScale;
eNewNorm = eNew/errorScale;

%% ========================================
% 6. Absolute tracking cost
%
% Small near reference
% Large far from reference
% =========================================

trackingCost = sum(eNewNorm.^2);

%% ========================================
% 7. Progress reward
%
% Positive if error becomes smaller
% Negative if error becomes larger
% =========================================

oldCost = sum(eOldNorm.^2);
newCost = sum(eNewNorm.^2);

progressReward = oldCost - newCost;

%% ========================================
% 8. Near-reference bonus
%% =========================================

absError = abs(eNew);

% Bonus if ALL tank errors are within 1 cm
if all(absError < 1.0)

    targetBonus = 2.0;

else

    targetBonus = 0;

end

% Larger bonus if extremely close
if all(absError < 0.25)

    targetBonus = targetBonus + 5.0;

end

%% ========================================
% 9. Total reward
%
% NO pump penalty
% NO delta-u penalty
%% =========================================

lambdaTracking = 10;
lambdaProgress = 50;

Reward = ...
    -lambdaTracking * trackingCost ...
    +lambdaProgress * progressReward ...
    +targetBonus;

%% ========================================
% 10. Next observation
%% =========================================

NextObservation = [
    xNext/60;
    eNew/30
];

%% ========================================
% 11. Save new state
%% =========================================

LoggedSignals.State = xNext;

%% PreviousAction is not needed anymore,
% but keeping it does no harm.
LoggedSignals.PreviousAction = u;

%% ========================================
% 12. Safety termination
%% =========================================

IsDone = ...
       any(xNext > 60) ...
    || any(xNext < 0) ...
    || any(~isfinite(xNext));

end