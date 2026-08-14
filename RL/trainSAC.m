clear;
clc;
close all;

%% ========================================
% 1. Observation specification
% =========================================

obsInfo = rlNumericSpec([6 1]);

obsInfo.Name = ...
    "Normalized tank states and tracking errors";

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
% 3. Create environment
% =========================================

env = rlFunctionEnv( ...
    obsInfo, ...
    actInfo, ...
    @stepFunction, ...
    @resetFunction);

disp("RL environment created.");

%% ========================================
% 4. Create NEW SAC agent
% =========================================

agent = rlSACAgent( ...
    obsInfo, ...
    actInfo);

disp("New SAC agent created.");

%% ========================================
% 5. Timing
% =========================================

TsAgent = 0.5;

% Keep 200 s first
Tf = 200;

maxSteps = round(Tf/TsAgent);

fprintf('\n');
fprintf('Episode duration   = %.1f s\n',Tf);
fprintf('RL sample time     = %.2f s\n',TsAgent);
fprintf('Steps per episode  = %d\n',maxSteps);
fprintf('\n');

%% ========================================
% 6. Folder for candidate agents
% =========================================

saveFolder = fullfile(pwd,"SavedAgents");

if ~exist(saveFolder,"dir")
    mkdir(saveFolder);
end

fprintf('Candidate agents will be saved in:\n');
fprintf('%s\n\n',saveFolder);

%% ========================================
% 7. Training options
% =========================================

trainOpts = rlTrainingOptions( ...
    MaxEpisodes=500, ...
    MaxStepsPerEpisode=maxSteps, ...
    ScoreAveragingWindowLength=20, ...
    Verbose=true, ...
    Plots="training-progress", ...
    ...
    SaveAgentCriteria="AverageReward", ...
    SaveAgentValue=-70, ...
    SaveAgentDirectory=saveFolder);

%% ========================================
% 8. Start training
% =========================================

trainingStats = train( ...
    agent, ...
    env, ...
    trainOpts);

%% ========================================
% 9. Save FINAL agent separately
% =========================================

save( ...
    "trainedSAC_final.mat", ...
    "agent", ...
    "trainingStats");

disp("Training finished.");
disp("Final agent saved as trainedSAC_final.mat");
disp("Candidate agents are inside SavedAgents/");