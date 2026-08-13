clear;
clc;
close all;

%% ========================================
% Load trained controller
% =========================================

load( ...
    "trainedSAC_v2.mat", ...
    "agent");

%% ========================================
% Parameters
% =========================================

TsPlant = 0.05;

TsAgent = 0.5;

Tf = 200;

Qmax = 85;

az = [0.5 0.5 0.5];

N = round(Tf/TsPlant);

Nhold = round(TsAgent/TsPlant);

%% ========================================
% Initial state
% =========================================

x = [10;10;10];

%% Desired water levels

ref = [30;10;20];

%% ========================================
% Storage
% =========================================

X = zeros(3,N);

U = zeros(2,N);

%% Initial control

u = [0;0];

%% ========================================
% Simulation
% =========================================

for k = 1:N

    %% Save state

    X(:,k) = x;

    %% ====================================
    % Update controller every 0.5 seconds
    % =====================================

    if mod(k-1,Nhold) == 0

        e = ref - x;

        obs = [
            x/60;
            e/30
        ];

        %% Ask SAC actor

        action = getAction( ...
            agent, ...
            {obs});

        %% Extract action

        if iscell(action)

            a = action{1};

        else

            a = action;

        end

        a = double(a(:));

        %% Safety clipping

        a = min(max(a,-1),1);

        %% Convert to pump flow

        u = (Qmax/2)*(a + 1);

    end

    %% Save pump command

    U(:,k) = u;

    %% ====================================
    % Simulate one physical step
    % =====================================

    x = tankEnvironment( ...
        x, ...
        u, ...
        az);

end

%% ========================================
% Time vector
% =========================================

t = (0:N-1)*TsPlant;

%% ========================================
% Plot water levels
% =========================================

figure;

plot( ...
    t, ...
    X(1,:), ...
    'LineWidth',1.2);

hold on;

plot( ...
    t, ...
    X(2,:), ...
    'LineWidth',1.2);

plot( ...
    t, ...
    X(3,:), ...
    'LineWidth',1.2);

yline( ...
    ref(1), ...
    '--');

yline( ...
    ref(2), ...
    '--');

yline( ...
    ref(3), ...
    '--');

xlabel('Time [s]');

ylabel('Water level [cm]');

title( ...
    'SAC Water Level Control');

legend( ...
    'h1', ...
    'h2', ...
    'h3', ...
    'r1', ...
    'r2', ...
    'r3');

grid on;

%% ========================================
% Plot pump commands
% =========================================

figure;

plot( ...
    t, ...
    U(1,:), ...
    'LineWidth',1.2);

hold on;

plot( ...
    t, ...
    U(2,:), ...
    'LineWidth',1.2);

xlabel('Time [s]');

ylabel('Pump flow [ml/s]');

title( ...
    'SAC Control Inputs');

legend( ...
    'Q1', ...
    'Q2');

ylim([0 90]);

grid on;