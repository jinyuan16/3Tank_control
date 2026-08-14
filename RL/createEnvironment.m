clear;
clc;

%% ========================================
% Observation specification
%
% [ h1/60
%   h2/60
%   h3/60
%   e1/30
%   e2/30
%   e3/30 ]
% =========================================

obsInfo = rlNumericSpec([6 1]);

obsInfo.Name = ...
    "Normalized tank states and tracking errors";

%% ========================================
% Action specification
%
% SAC outputs normalized pump commands
% in [-1,1]
% =========================================

actInfo = rlNumericSpec( ...
    [2 1], ...
    LowerLimit=[-1;-1], ...
    UpperLimit=[1;1]);

actInfo.Name = ...
    "Normalized pump commands";

%% ========================================
% Create environment
% =========================================

env = rlFunctionEnv( ...
    obsInfo, ...
    actInfo, ...
    @stepFunction, ...
    @resetFunction);

disp("RL environment created.");

%% ========================================
% Validate
% =========================================

validateEnvironment(env);

disp("Environment validation successful.");