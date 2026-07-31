clear;
clc;
close all;

load(fullfile('plotData','mpcResult.mat'));
load(fullfile('plotData','LearningmpcResult.mat'));

yMPC  = squeeze(mpcResult.Data);
yLMPC = squeeze(learningmpcResult.Data);

if size(yMPC,1) ~= 8 && size(yMPC,2) == 8
    yMPC = yMPC.';
end

if size(yLMPC,1) ~= 8 && size(yLMPC,2) == 8
    yLMPC = yLMPC.';
end

Ntarget = 62000;

N = min([Ntarget,size(yMPC,2),size(yLMPC,2)]);

yMPC  = yMPC(:,1:N);
yLMPC = yLMPC(:,1:N);

sampleIndex = 0:(N-1);

figure('Name','Liquid Level Comparison', ...
       'NumberTitle','off', ...
       'Position',[100 80 1000 760]);

subplot(3,1,1);

plot(sampleIndex,yMPC(1,:),'LineWidth',1.3);
hold on;
plot(sampleIndex,yLMPC(1,:),'--','LineWidth',1.3);
plot(sampleIndex,yMPC(4,:),':','LineWidth',1.5);

grid on;
xlim([0 N-1]);
ylabel('h_1');
title('Tank 1 Level');
legend('MPC','Learning MPC','Reference','Location','best');

subplot(3,1,2);

plot(sampleIndex,yMPC(2,:),'LineWidth',1.3);
hold on;
plot(sampleIndex,yLMPC(2,:),'--','LineWidth',1.3);
plot(sampleIndex,yMPC(5,:),':','LineWidth',1.5);

grid on;
xlim([0 N-1]);
ylabel('h_2');
title('Tank 2 Level');
legend('MPC','Learning MPC','Reference','Location','best');

subplot(3,1,3);

plot(sampleIndex,yMPC(3,:),'LineWidth',1.3);
hold on;
plot(sampleIndex,yLMPC(3,:),'--','LineWidth',1.3);
plot(sampleIndex,yMPC(6,:),':','LineWidth',1.5);

grid on;
xlim([0 N-1]);
xlabel('Sample');
ylabel('h_3');
title('Tank 3 Level');
legend('MPC','Learning MPC','Reference','Location','best');

figure('Name','Control Input Comparison', ...
       'NumberTitle','off', ...
       'Position',[150 120 1000 600]);

subplot(2,1,1);

plot(sampleIndex,yMPC(7,:),'LineWidth',1.3);
hold on;
plot(sampleIndex,yLMPC(7,:),'--','LineWidth',1.3);

grid on;
xlim([0 N-1]);
ylabel('q_1');
title('Pump 1 Flow');
legend('MPC','Learning MPC','Location','best');

subplot(2,1,2);

plot(sampleIndex,yMPC(8,:),'LineWidth',1.3);
hold on;
plot(sampleIndex,yLMPC(8,:),'--','LineWidth',1.3);

grid on;
xlim([0 N-1]);
xlabel('Sample');
ylabel('q_2');
title('Pump 2 Flow');
legend('MPC','Learning MPC','Location','best');

if ~exist('plotData','dir')
    mkdir('plotData');
end

exportgraphics(figure(1), ...
    fullfile('plotData','LevelComparison.png'), ...
    'Resolution',300);

exportgraphics(figure(2), ...
    fullfile('plotData','ControlComparison.png'), ...
    'Resolution',300);

fprintf('MPC data size: %d samples\n',size(yMPC,2));
fprintf('Learning MPC data size: %d samples\n',size(yLMPC,2));
fprintf('Compared samples: 0 to %d\n',N-1);
fprintf('Images saved in plotData folder.\n');