
%% MPC - Three-Tank Experimental Results
% Reference levels:
% Tank 1 = 30 cm
% Tank 2 = 10 cm
% Tank 3 = 20 cm

%% 1. Load experimental data

t1 = out.h1_data.Time(:);
h1 = out.h1_data.Data(:);

t2 = out.h2_data.Time(:);
h2 = out.h2_data.Data(:);

t3 = out.h3_data.Time(:);
h3 = out.h3_data.Data(:);

tq1 = out.q1_data.Time(:);
q1 = out.q1_data.Data(:);

tq2 = out.q2_data.Time(:);
q2 = out.q2_data.Data(:);

% Reference levels
r1 = 30;
r2 = 10;
r3 = 20;

% Common time range
tEnd = max([t1(end),t2(end),t3(end), ...
            tq1(end),tq2(end)]);


%% 2. Figure 1 - Three-Tank Water Level Tracking

fig1 = figure('Color','w', ...
    'Position',[100 50 1150 900]);

tl1 = tiledlayout(3,1, ...
    'TileSpacing','loose', ...
    'Padding','loose');

title(tl1,'MPC - Three-Tank Water Level Tracking', ...
    'FontSize',16,'FontWeight','bold');


% Tank 1
ax1 = nexttile;

plot(t1,h1,'b-','LineWidth',1.5);
hold on;
yline(r1,'r--','LineWidth',1.8);

title('Tank 1 - Water Level Tracking','FontSize',13);
ylabel('Water Level (cm)','FontSize',12);

legend('Actual Level h_1','Reference r_1', ...
    'Location','northeastoutside');

xlim([0 tEnd]);
ylim([5 40]);
grid on;
box on;
set(ax1,'FontSize',11);


% Tank 2
ax2 = nexttile;

plot(t2,h2,'b-','LineWidth',1.5);
hold on;
yline(r2,'r--','LineWidth',1.8);

title('Tank 2 - Water Level Tracking','FontSize',13);
ylabel('Water Level (cm)','FontSize',12);

legend('Actual Level h_2','Reference r_2', ...
    'Location','northeastoutside');

xlim([0 tEnd]);
ylim([5 30]);
grid on;
box on;
set(ax2,'FontSize',11);


% Tank 3
ax3 = nexttile;

plot(t3,h3,'b-','LineWidth',1.5);
hold on;
yline(r3,'r--','LineWidth',1.8);

title('Tank 3 - Water Level Tracking','FontSize',13);
ylabel('Water Level (cm)','FontSize',12);
xlabel('Time (s)','FontSize',12);

legend('Actual Level h_3','Reference r_3', ...
    'Location','northeastoutside');

xlim([0 tEnd]);
ylim([5 30]);
grid on;
box on;
set(ax3,'FontSize',11);

% Save water level figure
exportgraphics(fig1,'MPC_ThreeTank_WaterLevels.png', ...
    'Resolution',300);

savefig(fig1,'MPC_ThreeTank_WaterLevels.fig');


%% 3. Figure 2 - Pump Control Signals

fig2 = figure('Color','w', ...
    'Position',[150 100 1100 750]);

tl2 = tiledlayout(2,1, ...
    'TileSpacing','loose', ...
    'Padding','loose');

title(tl2,'MPC - Pump Control Signals', ...
    'FontSize',16,'FontWeight','bold');


% Pump 1
ax4 = nexttile;

plot(tq1,q1,'k-','LineWidth',1.5);

title('Pump 1 - Control Input','FontSize',13);
ylabel('Control Input q_1','FontSize',12);

xlim([0 tEnd]);
ylim([0 90]);
grid on;
box on;
set(ax4,'FontSize',11);


% Pump 2
ax5 = nexttile;

plot(tq2,q2,'k-','LineWidth',1.5);

title('Pump 2 - Control Input','FontSize',13);
ylabel('Control Input q_2','FontSize',12);
xlabel('Time (s)','FontSize',12);

xlim([0 tEnd]);
ylim([0 90]);
grid on;
box on;
set(ax5,'FontSize',11);

% Save pump control figure
exportgraphics(fig2,'MPC_PumpSignals.png', ...
    'Resolution',300);

savefig(fig2,'MPC_PumpSignals.fig');


%% 4. Save experimental data

MPC_Result = out;

save('MPC_ThreeTank_Data.mat','MPC_Result');

disp('MPC results saved successfully.');
