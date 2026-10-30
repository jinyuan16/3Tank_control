
%% Backstepping SISO - Experimental Results
% Figure 1: Water Level Tracking
% Figure 2: Pump Control Input

%% 1. Load experimental data

t = out.h2_data.Time(:);
h2 = out.h2_data.Data(:);

tq = out.q2_data.Time(:);
q2 = out.q2_data.Data(:);

% Reference
r2 = 25;


%% 2. Figure 1 - Water Level Tracking

fig1 = figure('Color','w', ...
    'Position',[100 100 1100 450]);

plot(t,h2,'b-','LineWidth',1.5);
hold on;

yline(r2,'r--','LineWidth',1.8);

xlabel('Time (s)','FontSize',12);
ylabel('Water Level (cm)','FontSize',12);

title('Backstepping SISO - Water Level Tracking', ...
    'FontSize',15,'FontWeight','bold');

legend('Actual Level h_2','Reference r_2', ...
    'Location','northeastoutside', ...
    'FontSize',11);

xlim([t(1) t(end)]);
ylim([15 30]);

grid on;
box on;
set(gca,'FontSize',11);

exportgraphics(fig1,'BS_SISO_WaterLevel.png', ...
    'Resolution',300);



%% 3. Figure 2 - Pump Control Input

fig2 = figure('Color','w', ...
    'Position',[150 150 1100 450]);

plot(tq,q2,'k-','LineWidth',1.5);

xlabel('Time (s)','FontSize',12);
ylabel('Control Input q_2','FontSize',12);

title('Backstepping SISO - Control Input', ...
    'FontSize',15,'FontWeight','bold');

xlim([tq(1) tq(end)]);
ylim([0 90]);

grid on;
box on;
set(gca,'FontSize',11);

exportgraphics(fig2,'BS_SISO_ControlInput.png', ...
    'Resolution',300);


