function [InitialObservation, LoggedSignals] = resetFunction()

%% ========================================
% Random initial state
%
% Requirements:
%   10 <= h2 < h3 < h1 <= 50
%   h1 - h2 <= 10
%% ========================================

% Lowest level h2
h2 = 10 + 30*rand;          % 10 ~ 40

% Total spread between h1 and h2
gapH = 2 + 8*rand;          % 2 ~ 10

% h3 lies between h2 and h1
fractionH = 0.2 + 0.6*rand;

h3 = h2 + fractionH*gapH;
h1 = h2 + gapH;

x0 = [
    h1;
    h2;
    h3
];

%% ========================================
% Random reference
%
% Same requirements:
%   10 <= r2 < r3 < r1 <= 50
%   r1 - r2 <= 10
%% ========================================

% Lowest reference r2
r2 = 10 + 30*rand;          % 10 ~ 40

% Total spread
gapR = 2 + 8*rand;          % 2 ~ 10

% r3 lies between r2 and r1
fractionR = 0.2 + 0.6*rand;

r3 = r2 + fractionR*gapR;
r1 = r2 + gapR;

ref = [
    r1;
    r2;
    r3
];

%% ========================================
% Store
%% ========================================

LoggedSignals.State = x0;
LoggedSignals.Reference = ref;

%% ========================================
% Initial tracking error
%% ========================================

e0 = ref - x0;

%% ========================================
% Observation
%% ========================================

InitialObservation = [
    x0/60;
    e0/30
];

end