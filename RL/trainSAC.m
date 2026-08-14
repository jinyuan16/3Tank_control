clear;
clc;
close all;

%% ========================================
% Observation specification
% =========================================

obsInfo = rlNumericSpec([6 1]);

obsInfo.Name = ...
    "Normalized tank states and tracking errors";

%% ========================================
% Action specification
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
% Create a NEW SAC agent
% =========================================

agent = rlSACAgent( ...
    obsInfo, ...
    actInfo);

disp("New SAC agent created.");

%% ========================================
% Timing
% =========================================

TsAgent = 0.5;

Tf = 200;

maxSteps = round(Tf/TsAgent);

fprintf( ...
    "Episode duration: %.1f s\n", ...
    Tf);

fprintf( ...
    "RL sample time: %.2f s\n", ...
    TsAgent);

fprintf( ...
    "Steps per episode: %d\n", ...
    maxSteps);

%% ========================================
% Training options
% =========================================

trainOpts = rlTrainingOptions( ...
    MaxEpisodes=500, ...
    MaxStepsPerEpisode=maxSteps, ...
    ScoreAveragingWindowLength=20, ...
    Verbose=true, ...
    Plots="training-progress");

%% ========================================
% Train
% =========================================

trainingStats = train( ...
    agent, ...
    env, ...
    trainOpts);

%% ========================================
% Save controller
% =========================================

save( ...
    "trainedSAC_realSmooth.mat", ...
    "agent", ...
    "trainingStats");

disp("Training finished.");

disp( ...
    "Saved as trainedSAC_realSmooth.mat");