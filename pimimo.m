
%% PI MIMO T1T3 - Experimental Results
% Two separate figures:
% Figure 1: Tank 1 and Tank 3 water levels
% Figure 2: Pump 1 and Pump 2 control inputs

%% 1. Load experimental data

t1 = out.h1_data.Time(:);
h1 = squeeze(out.h1_data.Data);
h1 = h1(:);

t3 = out.h3_data.Time(:);
h3 = squeeze(out.h3_data.Data);
h3 = h3(:);

tq1 = out.q1_data.Time(:);
q1 = squeeze(out.q1_data.Data);
q1 = q1(:);

tq2 = out.q2_data.Time(:);
q2 = squeeze(out.q2_data.Data);
q2 = q2(:);

% Reference levels
r1 = 35;
r3 = 25;

% Use the same horizontal axis for both figures
tEnd = max([t1(end),t3(end),tq1(end),tq2(end)]);

%% 2. Figure 1 - Water Level Tracking

fig1 = figure('Color','w', ...
    'Position',[100 100 1100 750]);

tl1 = tiledlayout(2,1, ...
    'TileSpacing','loose', ...
    'Padding','loose');

title(tl1,'PI MIMO T1T3 - Water Level Tracking', ...
    'FontSize',16,'FontWeight','bold');

% Tank 1
ax1 = nexttile;

plot(t1,h1,'b','LineWidth',1.5);
hold on;
yline(r1,'r--','LineWidth',1.6);

title('Tank 1 - Water Level Tracking', ...
    'FontSize',13);

ylabel('Water Level (cm)','FontSize',12);
xlabel('Time (s)','FontSize',11);

legend('Actual Level h_1','Reference r_1', ...
    'Location','northeastoutside', ...
    'FontSize',10);

xlim([0 tEnd]);
ylim([18 40]);
grid on;
box on;
set(ax1,'FontSize',11);

% Tank 3
ax2 = nexttile;

plot(t3,h3,'b','LineWidth',1.5);
hold on;
yline(r3,'r--','LineWidth',1.6);

title('Tank 3 - Water Level Tracking', ...
    'FontSize',13);

ylabel('Water Level (cm)','FontSize',12);
xlabel('Time (s)','FontSize',11);

legend('Actual Level h_3','Reference r_3', ...
    'Location','northeastoutside', ...
    'FontSize',10);

xlim([0 tEnd]);
ylim([12 30]);
grid on;
box on;
set(ax2,'FontSize',11);

% Export Figure 1
exportgraphics(fig1,'PI_MIMO_T1T3_WaterLevels.png', ...
    'Resolution',300);



%% 3. Figure 2 - Pump Control Signals

fig2 = figure('Color','w', ...
    'Position',[150 100 1100 750]);

tl2 = tiledlayout(2,1, ...
    'TileSpacing','loose', ...
    'Padding','loose');

title(tl2,'PI MIMO T1T3 - Pump Control Signals', ...
    'FontSize',16,'FontWeight','bold');

% Pump 1
ax3 = nexttile;

plot(tq1,q1,'k','LineWidth',1.5);

title('Pump 1 - Control Signal', ...
    'FontSize',13);

ylabel('Control Input q_1','FontSize',12);
xlabel('Time (s)','FontSize',11);

xlim([0 tEnd]);
ylim([0 90]);
grid on;
box on;
set(ax3,'FontSize',11);

% Pump 2
ax4 = nexttile;

plot(tq2,q2,'k','LineWidth',1.5);

title('Pump 2 - Control Signal', ...
    'FontSize',13);

ylabel('Control Input q_2','FontSize',12);
xlabel('Time (s)','FontSize',11);

xlim([0 tEnd]);
ylim([0 90]);
grid on;
box on;
set(ax4,'FontSize',11);

% Export Figure 2
exportgraphics(fig2,'PI_MIMO_T1T3_PumpSignals.png', ...
    'Resolution',300);


