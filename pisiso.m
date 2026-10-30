%% PI SISO - Water Level Tracking Only

% Experimental data
t = out.h2_data.Time(:);
h2 = out.h2_data.Data(:);

% Reference water level
r2 = 25;

% Create figure
figure('Color','w','Position',[100 100 1100 450]);

% Actual water level
plot(t,h2,'b-','LineWidth',1.5);
hold on;

% Reference water level
yline(r2,'r--','LineWidth',1.8);

% Labels and title
xlabel('Time (s)','FontSize',12);
ylabel('Water Level (cm)','FontSize',12);

title('PI SISO - Water Level Tracking', ...
    'FontSize',15,'FontWeight','bold');

% Legend outside to avoid covering the curve
legend('Actual Level h_2','Reference r_2', ...
    'Location','northeastoutside', ...
    'FontSize',11);

% Axis settings
ylim([15 30]);
xlim([t(1) t(end)]);

grid on;
box on;
set(gca,'FontSize',11);

% Save high-resolution image
exportgraphics(gcf,'PI_SISO_WaterLevel.png', ...
    'Resolution',300);

% Save editable MATLAB figure
savefig(gcf,'PI_SISO_WaterLevel.fig');