clear;
clc;
close all;

%% =========================================================
% Load trained SAC agent
%% =========================================================

load("trainedSAC_realSmooth.mat","agent");

%% Get the two SAC critics
critics = getCritic(agent);

critic1 = critics(1);
critic2 = critics(2);

%% =========================================================
% Choose the physical tank state you want to investigate
%
% Example 1:
% h = [10;10;10];
%
% Example 2:
% h = [30;10;20];   % exactly at reference
%
% Example 3:
% h = [40;40;40];   % above reference
%% =========================================================

h = [30;10;20];

ref = [30;10;20];

%% Tracking error
e = ref - h;

%% Same observation normalization used during training
obs = [
    h/60;
    e/30
];

fprintf("\n========================================\n");
fprintf("State being investigated:\n");
fprintf("h1 = %.2f cm\n",h(1));
fprintf("h2 = %.2f cm\n",h(2));
fprintf("h3 = %.2f cm\n",h(3));

fprintf("\nReference:\n");
fprintf("r1 = %.2f cm\n",ref(1));
fprintf("r2 = %.2f cm\n",ref(2));
fprintf("r3 = %.2f cm\n",ref(3));

fprintf("\nNormalized observation:\n");
disp(obs);

%% =========================================================
% Pump range
%% =========================================================

Qmax = 85;

% Resolution of pump sweep
% 1 means:
% 0,1,2,...,85 ml/s
%
% 5 means:
% 0,5,10,...,85 ml/s
%
% Start with 5 for fast calculation.
pumpStep = 5;

pumpValues = 0:pumpStep:Qmax;

N = length(pumpValues);

%% Storage
Qcritic1 = zeros(N,N);
Qcritic2 = zeros(N,N);
Qmin     = zeros(N,N);

%% =========================================================
% Scan ALL pump combinations
%% =========================================================

fprintf("\nScanning critic Q-values...\n");

for i = 1:N

    Q1physical = pumpValues(i);

    for j = 1:N

        Q2physical = pumpValues(j);

        %% Physical pump command
        uPhysical = [
            Q1physical;
            Q2physical
        ];

        %% Convert physical pump command to SAC normalized action
        %
        % physical:
        %
        % u = Qmax/2 * (a + 1)
        %
        % therefore:
        %
        % a = 2*u/Qmax - 1

        action = 2*uPhysical/Qmax - 1;

        %% Evaluate critic 1
        q1 = getValue(critic1,{obs},{action});

        %% Evaluate critic 2
        q2 = getValue(critic2,{obs},{action});

        %% Convert dlarray / numeric object to double
        q1 = double(extractdata(q1));
        q2 = double(extractdata(q2));

        %% Store values
        Qcritic1(j,i) = q1;
        Qcritic2(j,i) = q2;

        %% SAC conservative critic estimate
        Qmin(j,i) = min(q1,q2);

    end
end

fprintf("Critic scan finished.\n");

%% =========================================================
% Find critic-best pump combination
%% =========================================================

[maxQ,index] = max(Qmin(:));

[row,col] = ind2sub(size(Qmin),index);

bestPump1 = pumpValues(col);
bestPump2 = pumpValues(row);

fprintf("\n========================================\n");
fprintf("CRITIC BEST ACTION\n");
fprintf("========================================\n");

fprintf("Pump 1 = %.2f ml/s\n",bestPump1);
fprintf("Pump 2 = %.2f ml/s\n",bestPump2);
fprintf("Maximum min(Q1,Q2) = %.6f\n",maxQ);

fprintf("\nCritic 1 at best action = %.6f\n", ...
    Qcritic1(row,col));

fprintf("Critic 2 at best action = %.6f\n", ...
    Qcritic2(row,col));

%% =========================================================
% Ask ACTOR what it would actually do at this state
%% =========================================================

actorAction = getAction(agent,{obs});

%% Extract numeric action
actorAction = actorAction{1};
actorAction = double(extractdata(actorAction));

%% Convert normalized action [-1,1]
% back to physical pump flow [0,85]

actorPump = Qmax/2*(actorAction + 1);

fprintf("\n========================================\n");
fprintf("ACTOR ACTION\n");
fprintf("========================================\n");

fprintf("Normalized action:\n");
disp(actorAction);

fprintf("Physical pump command:\n");
fprintf("Pump 1 = %.4f ml/s\n",actorPump(1));
fprintf("Pump 2 = %.4f ml/s\n",actorPump(2));

%% =========================================================
% Evaluate critic at actor's chosen action
%% =========================================================

q1Actor = getValue(critic1,{obs},{actorAction});
q2Actor = getValue(critic2,{obs},{actorAction});

q1Actor = double(extractdata(q1Actor));
q2Actor = double(extractdata(q2Actor));

qActorMin = min(q1Actor,q2Actor);

fprintf("\nCritic evaluation of actor action:\n");

fprintf("Critic 1 = %.6f\n",q1Actor);
fprintf("Critic 2 = %.6f\n",q2Actor);
fprintf("min(Q1,Q2) = %.6f\n",qActorMin);

fprintf("\nDifference from critic grid maximum:\n");
fprintf("Best critic Q - actor Q = %.6f\n", ...
    maxQ-qActorMin);

%% =========================================================
% Display some useful specific actions
%% =========================================================

fprintf("\n========================================\n");
fprintf("SELECTED ACTION COMPARISON\n");
fprintf("========================================\n");

testActions = [
     0   0
    10  10
    20  20
    40  40
    60  60
    85  85
];

fprintf("\n");
fprintf(" Pump1     Pump2       Critic1       Critic2       min(Q)\n");
fprintf("-------------------------------------------------------------\n");

for k = 1:size(testActions,1)

    u = testActions(k,:)';

    a = 2*u/Qmax - 1;

    q1 = getValue(critic1,{obs},{a});
    q2 = getValue(critic2,{obs},{a});

    q1 = double(extractdata(q1));
    q2 = double(extractdata(q2));

    fprintf("%6.1f    %6.1f     %10.4f     %10.4f     %10.4f\n", ...
        u(1),u(2),q1,q2,min(q1,q2));

end

%% =========================================================
% Plot 1: Q heatmap
%% =========================================================

figure;

imagesc(pumpValues,pumpValues,Qmin);

axis xy;

colorbar;

xlabel("Pump Q1 [ml/s]");
ylabel("Pump Q2 [ml/s]");

title(sprintf( ...
    "SAC Critic min(Q_1,Q_2), h = [%.1f %.1f %.1f] cm", ...
    h(1),h(2),h(3)));

hold on;

%% Critic-best action
plot(bestPump1,bestPump2, ...
    "kx", ...
    "MarkerSize",16, ...
    "LineWidth",3);

%% Actor action
plot(actorPump(1),actorPump(2), ...
    "ko", ...
    "MarkerSize",10, ...
    "LineWidth",2);

legend( ...
    "Critic maximum", ...
    "Actor action", ...
    "Location","best");

%% =========================================================
% Plot 2: 3D Q surface
%% =========================================================

[Pump1Grid,Pump2Grid] = meshgrid( ...
    pumpValues,pumpValues);

figure;

surf( ...
    Pump1Grid, ...
    Pump2Grid, ...
    Qmin);

xlabel("Pump Q1 [ml/s]");
ylabel("Pump Q2 [ml/s]");
zlabel("min(Q_1,Q_2)");

title(sprintf( ...
    "Critic Q Surface at h = [%.1f %.1f %.1f] cm", ...
    h(1),h(2),h(3)));

grid on;

hold on;

%% Mark critic maximum
plot3( ...
    bestPump1, ...
    bestPump2, ...
    maxQ, ...
    "kx", ...
    "MarkerSize",16, ...
    "LineWidth",3);

%% Mark actor action
plot3( ...
    actorPump(1), ...
    actorPump(2), ...
    qActorMin, ...
    "ko", ...
    "MarkerSize",10, ...
    "LineWidth",2);

legend( ...
    "Critic Q surface", ...
    "Critic maximum", ...
    "Actor action", ...
    "Location","best");

%% =========================================================
% Plot 3: Critic disagreement
%
% Shows where critic 1 and critic 2 disagree
%% =========================================================

criticDifference = abs(Qcritic1-Qcritic2);

figure;

imagesc( ...
    pumpValues, ...
    pumpValues, ...
    criticDifference);

axis xy;

colorbar;

xlabel("Pump Q1 [ml/s]");
ylabel("Pump Q2 [ml/s]");

title(sprintf( ...
    "|Q_1 - Q_2| at h = [%.1f %.1f %.1f] cm", ...
    h(1),h(2),h(3)));

%% =========================================================
% Save result
%% =========================================================

results.h = h;
results.ref = ref;
results.obs = obs;

results.pumpValues = pumpValues;

results.Qcritic1 = Qcritic1;
results.Qcritic2 = Qcritic2;
results.Qmin = Qmin;

results.bestPump = [
    bestPump1;
    bestPump2
];

results.maxQ = maxQ;

results.actorActionNormalized = actorAction;
results.actorPump = actorPump;

results.actorQ1 = q1Actor;
results.actorQ2 = q2Actor;
results.actorQmin = qActorMin;

save("criticDiagnostic.mat","results");

fprintf("\nResults saved to criticDiagnostic.mat\n");
