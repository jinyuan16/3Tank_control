function [InitialObservation, LoggedSignals] = resetFunction()

%% ========================================
% Initial water levels
% =========================================

x0 = [10;10;10];

%% Desired reference

ref = [30;10;20];

%% Save current state

LoggedSignals.State = x0;

LoggedSignals.Reference = ref;

%% Previous physical pump command
% Empty means:
% do not penalize the very first control action

LoggedSignals.PreviousAction = [];

%% Initial tracking error

e0 = ref - x0;

%% ========================================
% Normalized observation
%
% [ h1/60
%   h2/60
%   h3/60
%   e1/30
%   e2/30
%   e3/30 ]
% =========================================

InitialObservation = [
    x0/60;
    e0/30
];

end