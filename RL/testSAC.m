clear;
clc;
close all;

%% ========================================
% Load trained SAC
% =========================================

load( ...
    "trainedSAC_realSmooth.mat", ...
    "agent");

%% ========================================
% Real calibrated parameters
% =========================================

az = [0.4191 0.7860 0.4639];

Qmax = 85.0;

%% ========================================
% Timing
% =========================================

TsPlant = 0.05;

TsAgent = 2;

Tf = 250;

N = round(Tf/TsPlant);

Nhold = round(TsAgent/TsPlant);

%% ========================================
% Initial condition
% =========================================

x = [30;10;15];

%% ========================================
% Reference
% =========================================

ref = [20;16;18];

%% ========================================
% Storage
% =========================================

X = zeros(3,N);

U = zeros(2,N);

E = zeros(3,N);

%% Initial pump command

u = [0;0];

%% ========================================
% Simulation
% =========================================

for k = 1:N

    %% Save current state

    X(:,k) = x;

    %% Tracking error

    e = ref - x;

    E(:,k) = e;

    %% ====================================
    % SAC update every 0.5 s
    % =====================================

    if mod(k-1,Nhold) == 0

        %% Normalized observation

        obs = [
            x/60;
            e/30
        ];

        %% Get action

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

        %% Clamp normalized action

        a = min(max(a,-1),1);

        %% Convert to physical pump flow

        u = (Qmax/2)*(a + 1);

    end

    %% Save control

    U(:,k) = u;

    %% ====================================
    % Plant simulation
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
% Plot 1: Water levels
% =========================================

figure;

plot( ...
    t, ...
    X(1,:), ...
    'LineWidth',1.3);

hold on;

plot( ...
    t, ...
    X(2,:), ...
    'LineWidth',1.3);

plot( ...
    t, ...
    X(3,:), ...
    'LineWidth',1.3);

yline(ref(1),'--','r1');
yline(ref(2),'--','r2');
yline(ref(3),'--','r3');

xlabel('Time [s]');

ylabel('Water level [cm]');

title('SAC Water Level Control');

legend( ...
    'h1', ...
    'h2', ...
    'h3', ...
    'Location','best');

grid on;

%% ========================================
% Plot 2: Pump commands
% =========================================

figure;

plot( ...
    t, ...
    U(1,:), ...
    'LineWidth',1.3);

hold on;

plot( ...
    t, ...
    U(2,:), ...
    'LineWidth',1.3);

xlabel('Time [s]');

ylabel('Pump flow [ml/s]');

title('SAC Pump Commands');

legend('Q1','Q2');

ylim([0 90]);

grid on;

%% ========================================
% Plot 3: Pump-rate changes
% =========================================

dU = diff(U,1,2);

tdU = t(2:end);

figure;

plot( ...
    tdU, ...
    dU(1,:), ...
    'LineWidth',1.2);

hold on;

plot( ...
    tdU, ...
    dU(2,:), ...
    'LineWidth',1.2);

xlabel('Time [s]');

ylabel('\Delta Q [ml/s]');

title('Pump Command Changes');

legend('\Delta Q1','\Delta Q2');

grid on;

%% ========================================
% Final values
% =========================================

fprintf('\n');
fprintf('===== Final result =====\n');

fprintf( ...
    'Final h1 = %.3f cm, reference = %.3f cm\n', ...
    X(1,end), ...
    ref(1));

fprintf( ...
    'Final h2 = %.3f cm, reference = %.3f cm\n', ...
    X(2,end), ...
    ref(2));

fprintf( ...
    'Final h3 = %.3f cm, reference = %.3f cm\n', ...
    X(3,end), ...
    ref(3));

fprintf('\n');

fprintf( ...
    'Final Q1 = %.3f ml/s\n', ...
    U(1,end));

fprintf( ...
    'Final Q2 = %.3f ml/s\n', ...
    U(2,end));

fprintf('\n');

fprintf( ...
    'Maximum |Delta Q1| = %.3f ml/s\n', ...
    max(abs(dU(1,:))));

fprintf( ...
    'Maximum |Delta Q2| = %.3f ml/s\n', ...
    max(abs(dU(2,:))));