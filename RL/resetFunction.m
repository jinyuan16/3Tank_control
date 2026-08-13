function [InitialObservation, LoggedSignals] = resetFunction()

%% ========================================
% Initial condition
% =========================================

x0 = [10;10;10];

%% Desired water levels

ref = [30;10;20];

%% Save environment information

LoggedSignals.State = x0;
LoggedSignals.Reference = ref;

%% Initial tracking error

e0 = ref - x0;

%% ========================================
% Normalized observation
%
% Observation =
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