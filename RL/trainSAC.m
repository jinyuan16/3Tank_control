clear;
clc;
close all;

%% ========================================
% 1. Observation specification
% =========================================

obsInfo = rlNumericSpec([6 1]);

obsInfo.Name = ...
    "Normalized tank states and errors";

%% ========================================
% 2. Action specification
% =========================================

actInfo = rlNumericSpec( ...
    [2 1], ...
    LowerLimit=[-1;-1], ...
    UpperLimit=[1;1]);

actInfo.Name = ...
    "Normalized pump commands";

%% ========================================
% 3. Create RL environment
% =========================================

env = rlFunctionEnv( ...
    obsInfo, ...
    actInfo, ...
    @stepFunction, ...
    @resetFunction);

disp("RL environment created.");

%% ========================================
% 4. Create SAC agent
% =========================================

agent = rlSACAgent( ...
    obsInfo, ...
    actInfo);

disp("SAC agent created.");

%% ========================================
% 5. Timing
% =========================================

% RL controller decides every 0.5 s
TsAgent = 0.5;

% One episode represents 200 seconds
Tf = 200;

maxSteps = round(Tf/TsAgent);

fprintf( ...
    "Steps per episode: %d\n", ...
    maxSteps);

%% ========================================
% 6. Training options
% =========================================

trainOpts = rlTrainingOptions( ...
    MaxEpisodes=500, ...
    MaxStepsPerEpisode=maxSteps, ...
    ScoreAveragingWindowLength=20, ...
    Verbose=true, ...
    Plots="training-progress");

%% ========================================
% 7. Start training
% =========================================

trainingStats = train( ...
    agent, ...
    env, ...
    trainOpts);

%% ========================================
% 8. Save trained controller
% =========================================

save( ...
    "trainedSAC_v2.mat", ...
    "agent", ...
    "trainingStats");

disp("Training finished.");

disp( ...
    "Saved as trainedSAC_v2.mat");