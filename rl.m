r1 = 30;
r2 = 10;
r3 = 20;

h1 = out.h1_data;
h2 = out.h2_data;
h3 = out.h3_data;
q1 = out.q1_data;
q2 = out.q2_data;

t1 = h1.Time;
t2 = h2.Time;
t3 = h3.Time;
tq1 = q1.Time;
tq2 = q2.Time;

y1 = squeeze(h1.Data);
y2 = squeeze(h2.Data);
y3 = squeeze(h3.Data);
u1 = squeeze(q1.Data);
u2 = squeeze(q2.Data);

Tend = max([t1(end),t2(end),t3(end),tq1(end),tq2(end)]);

fig1 = figure('Color','w','Position',[100 50 1400 1000]);

tl1 = tiledlayout(3,1,'TileSpacing','loose','Padding','compact');
title(tl1,'RL - Three-Tank Water Level Tracking', ...
    'FontSize',18,'FontWeight','bold');

nexttile;
plot(t1,y1,'b','LineWidth',1.5);
hold on;
yline(r1,'r--','LineWidth',1.5);
title('Tank 1 - Water Level Tracking');
ylabel('Water Level (cm)');
legend('Actual Level h_1','Reference r_1','Location','northeastoutside');
grid on;
xlim([0 Tend]);

nexttile;
plot(t2,y2,'b','LineWidth',1.5);
hold on;
yline(r2,'r--','LineWidth',1.5);
title('Tank 2 - Water Level Tracking');
ylabel('Water Level (cm)');
legend('Actual Level h_2','Reference r_2','Location','northeastoutside');
grid on;
xlim([0 Tend]);

nexttile;
plot(t3,y3,'b','LineWidth',1.5);
hold on;
yline(r3,'r--','LineWidth',1.5);
title('Tank 3 - Water Level Tracking');
ylabel('Water Level (cm)');
xlabel('Time (s)');
legend('Actual Level h_3','Reference r_3','Location','northeastoutside');
grid on;
xlim([0 Tend]);

exportgraphics(fig1,'RL_Water_Level_Tracking.png','Resolution',300);
savefig(fig1,'RL_Water_Level_Tracking.fig');


fig2 = figure('Color','w','Position',[150 100 1400 850]);

tl2 = tiledlayout(2,1,'TileSpacing','loose','Padding','compact');
title(tl2,'RL - Pump Control Signals', ...
    'FontSize',18,'FontWeight','bold');

nexttile;
plot(tq1,u1,'k','LineWidth',1.2);
title('Pump 1 - Control Input');
ylabel('Control Input q_1 (ml/s)');
ylim([0 90]);
xlim([0 Tend]);
grid on;

nexttile;
plot(tq2,u2,'k','LineWidth',1.2);
title('Pump 2 - Control Input');
ylabel('Control Input q_2 (ml/s)');
xlabel('Time (s)');
ylim([0 90]);
xlim([0 Tend]);
grid on;

exportgraphics(fig2,'RL_Pump_Control_Signals.png','Resolution',300);
savefig(fig2,'RL_Pump_Control_Signals.fig');


RL_Result = out;
save('RL_Result.mat','RL_Result');

disp('RL figures and experiment data saved successfully.');