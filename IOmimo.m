
%% IO MIMO - Experimental Results
% Tank 1: Initial 20 cm -> Reference 35 cm
% Tank 3: Initial 20 cm -> Reference 25 cm
%
% Figure 1: Water Level Tracking (h1 and h3)
% Figure 2: Pump Control Signals (q1 and q2)

%% 1. Load experimental data

t1 = out.h1_data.Time(:);
h1 = out.h1_data.Data(:);

t3 = out.h3_data.Time(:);
h3 = out.h3_data.Data(:);

tq1 = out.q1_data.Time(:);
q1 = out.q1_data.Data(:);

tq2 = out.q2_data.Time(:);
q2 = out.q2_data.Data(:);

% Reference water levels
r1 = 35;
r3 = 25;

% Common time range
tEnd = max([t1(end),t3(end),tq1(end),tq2(end)]);


%% 2. Figure 1 - Water Level Tracking

fig1 = figure('Color','w', ...
    'Position',[100 100 1100 750]);

tl1 = tiledlayout(2,1, ...
    'TileSpacing','loose', ...
    'Padding','loose');

title(tl1,'IO MIMO - Water Level Tracking', ...
    'FontSize',16,'FontWeight','bold');

% Tank 1
ax1 = nexttile;

plot(t1,h1,'b-','LineWidth',1.5);
hold on;
yline(r1,'r--','LineWidth',1.8);

title('Tank 1 - Water Level Tracking', ...
    'FontSize',13);

ylabel('Water Level (cm)','FontSize',12);
xlabel('Time (s)','FontSize',11);

legend('Actual Level h_1','Reference r_1', ...
    'Location','northeastoutside', ...
    'FontSize',10);

xlim([0 tEnd]);
ylim([15 40]);

grid on;
box on;
set(ax1,'FontSize',11);


% Tank 3
ax2 = nexttile;

plot(t3,h3,'b-','LineWidth',1.5);
hold on;
yline(r3,'r--','LineWidth',1.8);

title('Tank 3 - Water Level Tracking', ...
    'FontSize',13);

ylabel('Water Level (cm)','FontSize',12);
xlabel('Time (s)','FontSize',11);

legend('Actual Level h_3','Reference r_3', ...
    'Location','northeastoutside', ...
    'FontSize',10);

xlim([0 tEnd]);
ylim([15 30]);

grid on;
box on;
set(ax2,'FontSize',11);

% Save water level figure
exportgraphics(fig1,'IO_MIMO_WaterLevels.png', ...
    'Resolution',300);

savefig(fig1,'IO_MIMO_WaterLevels.fig');


%% 3. Figure 2 - Pump Control Signals

fig2 = figure('Color','w', ...
    'Position',[150 100 1100 750]);

tl2 = tiledlayout(2,1, ...
    'TileSpacing','loose', ...
    'Padding','loose');

title(tl2,'IO MIMO - Pump Control Signals', ...
    'FontSize',16,'FontWeight','bold');


% Pump 1
ax3 = nexttile;

plot(tq1,q1,'k-','LineWidth',1.5);

title('Pump 1 - Control Input', ...
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

plot(tq2,q2,'k-','LineWidth',1.5);

title('Pump 2 - Control Input', ...
    'FontSize',13);

ylabel('Control Input q_2','FontSize',12);
xlabel('Time (s)','FontSize',11);

xlim([0 tEnd]);
ylim([0 90]);

grid on;
box on;
set(ax4,'FontSize',11);

% Save pump control figure
exportgraphics(fig2,'IO_MIMO_PumpSignals.png', ...
    'Resolution',300);

savefig(fig2,'IO_MIMO_PumpSignals.fig');


%% 4. Save experimental data

IO_MIMO_Result = out;

save('IO_MIMO_Data.mat','IO_MIMO_Result');

disp('IO MIMO results saved successfully.');
