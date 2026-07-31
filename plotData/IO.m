clear;
clc;
close all;

%% =========================================================
% 1. Locate data folder
% ==========================================================

% Folder containing this script
scriptDir = fileparts(mfilename('fullpath'));

% If the script is run section-by-section, mfilename may be empty
if isempty(scriptDir)
    scriptDir = pwd;
end

% Determine whether MAT files are in the current script folder
if isfile(fullfile(scriptDir,'ioResult.mat')) && ...
   isfile(fullfile(scriptDir,'learningIOResult.mat'))

    dataDir = scriptDir;

elseif isfile(fullfile(scriptDir,'plotData','ioResult.mat')) && ...
       isfile(fullfile(scriptDir,'plotData','learningIOResult.mat'))

    dataDir = fullfile(scriptDir,'plotData');

else
    error(['Cannot find ioResult.mat and learningIOResult.mat. ', ...
           'Check the location of the MAT files.']);
end

%% =========================================================
% 2. Load data
% ==========================================================

S1 = load(fullfile(dataDir,'ioResult.mat'));
S2 = load(fullfile(dataDir,'learningIOResult.mat'));

% Actual variable names stored in the MAT files:
% ioResult.mat          -> IOResult
% learningIOResult.mat  -> learningIOResult

if ~isfield(S1,'IOResult')
    error('Variable IOResult was not found in ioResult.mat.');
end

if ~isfield(S2,'learningIOResult')
    error(['Variable learningIOResult was not found in ', ...
           'learningIOResult.mat.']);
end

if ~isa(S1.IOResult,'timeseries')
    error('IOResult is not a timeseries object.');
end

if ~isa(S2.learningIOResult,'timeseries')
    error('learningIOResult is not a timeseries object.');
end

%% =========================================================
% 3. Extract and reshape data
% ==========================================================

io  = squeeze(S1.IOResult.Data);
lio = squeeze(S2.learningIOResult.Data);

% Convert both arrays to 13-by-N
io  = convertToChannelBySample(io,13,'IOResult');
lio = convertToChannelBySample(lio,13,'learningIOResult');

fprintf('IO data size:          %d x %d\n',size(io,1),size(io,2));
fprintf('Learning IO data size: %d x %d\n',size(lio,1),size(lio,2));

%% =========================================================
% 4. Select samples
% ==========================================================

Ntarget = 58000;

N = min([Ntarget,size(io,2),size(lio,2)]);

if N < 2
    error('There are not enough samples to generate plots.');
end

io  = io(:,1:N);
lio = lio(:,1:N);

k = 0:N-1;

%% =========================================================
% 5. Tank-level comparison
% ==========================================================

figLevel = figure( ...
    'Name','IO Level Comparison', ...
    'NumberTitle','off', ...
    'Position',[100 80 950 750]);

subplot(3,1,1);

plot(k,io(1,:),'b','LineWidth',1.4);
hold on;
plot(k,lio(1,:),'r--','LineWidth',1.4);
plot(k,io(4,:),'k:','LineWidth',1.4);

grid on;
xlim([0 N-1]);
ylabel('h_1');
title('Tank 1');
legend('IO','Learning IO','Reference','Location','best');

subplot(3,1,2);

plot(k,io(2,:),'b','LineWidth',1.4);
hold on;
plot(k,lio(2,:),'r--','LineWidth',1.4);

grid on;
xlim([0 N-1]);
ylabel('h_2');
title('Tank 2');
legend('IO','Learning IO','Location','best');

subplot(3,1,3);

plot(k,io(3,:),'b','LineWidth',1.4);
hold on;
plot(k,lio(3,:),'r--','LineWidth',1.4);
plot(k,io(5,:),'k:','LineWidth',1.4);

grid on;
xlim([0 N-1]);
xlabel('Sample');
ylabel('h_3');
title('Tank 3');
legend('IO','Learning IO','Reference','Location','best');

%% =========================================================
% 6. Pump-input comparison
% ==========================================================

figInput = figure( ...
    'Name','IO Input Comparison', ...
    'NumberTitle','off', ...
    'Position',[120 100 950 600]);

subplot(2,1,1);

plot(k,io(6,:),'b','LineWidth',1.4);
hold on;
plot(k,lio(6,:),'r--','LineWidth',1.4);

grid on;
xlim([0 N-1]);
ylabel('q_1');
title('Pump 1');
legend('IO','Learning IO','Location','best');

subplot(2,1,2);

plot(k,io(7,:),'b','LineWidth',1.4);
hold on;
plot(k,lio(7,:),'r--','LineWidth',1.4);

grid on;
xlim([0 N-1]);
xlabel('Sample');
ylabel('q_2');
title('Pump 2');
legend('IO','Learning IO','Location','best');

%% =========================================================
% 7. Learned residuals
% ==========================================================

figResidual = figure( ...
    'Name','Learning IO Residuals', ...
    'NumberTitle','off', ...
    'Position',[150 120 950 700]);

subplot(3,1,1);

plot(k,lio(8,:),'LineWidth',1.5);

grid on;
xlim([0 N-1]);
ylabel('\Delta q_{13}');
title('Learned Flow Residuals');

subplot(3,1,2);

plot(k,lio(9,:),'LineWidth',1.5);

grid on;
xlim([0 N-1]);
ylabel('\Delta q_{32}');

subplot(3,1,3);

plot(k,lio(10,:),'LineWidth',1.5);

grid on;
xlim([0 N-1]);
xlabel('Sample');
ylabel('\Delta q_{20}');

%% =========================================================
% 8. Tracking-error comparison
% ==========================================================

figError = figure( ...
    'Name','IO Tracking Error Comparison', ...
    'NumberTitle','off', ...
    'Position',[180 120 950 600]);

subplot(2,1,1);

plot(k,io(11,:),'b','LineWidth',1.4);
hold on;
plot(k,lio(11,:),'r--','LineWidth',1.4);

grid on;
xlim([0 N-1]);
ylabel('e_1');
title('Tank 1 Tracking Error');
legend('IO','Learning IO','Location','best');

subplot(2,1,2);

plot(k,io(12,:),'b','LineWidth',1.4);
hold on;
plot(k,lio(12,:),'r--','LineWidth',1.4);

grid on;
xlim([0 N-1]);
xlabel('Sample');
ylabel('e_3');
title('Tank 3 Tracking Error');
legend('IO','Learning IO','Location','best');

%% =========================================================
% 9. RMSE comparison
% ==========================================================

rmse_h1_io  = sqrt(mean(io(11,:).^2,'omitnan'));
rmse_h1_lio = sqrt(mean(lio(11,:).^2,'omitnan'));

rmse_h3_io  = sqrt(mean(io(12,:).^2,'omitnan'));
rmse_h3_lio = sqrt(mean(lio(12,:).^2,'omitnan'));

rmseValues = [
    rmse_h1_io, rmse_h1_lio;
    rmse_h3_io, rmse_h3_lio
];

figRMSE = figure( ...
    'Name','IO RMSE Comparison', ...
    'NumberTitle','off', ...
    'Position',[300 200 550 420]);

bar(rmseValues);

grid on;
set(gca,'XTickLabel',{'h_1','h_3'});
ylabel('RMSE');
title('Tracking Error RMSE');
legend('IO','Learning IO','Location','best');

fprintf('\n========== RMSE results ==========\n');
fprintf('h1 IO RMSE:          %.8f\n',rmse_h1_io);
fprintf('h1 Learning IO RMSE: %.8f\n',rmse_h1_lio);
fprintf('h3 IO RMSE:          %.8f\n',rmse_h3_io);
fprintf('h3 Learning IO RMSE: %.8f\n',rmse_h3_lio);

%% =========================================================
% 10. Save figures
% ==========================================================

exportgraphics( ...
    figLevel, ...
    fullfile(dataDir,'LevelComparison_IO.png'), ...
    'Resolution',300);

exportgraphics( ...
    figInput, ...
    fullfile(dataDir,'InputComparison_IO.png'), ...
    'Resolution',300);

exportgraphics( ...
    figResidual, ...
    fullfile(dataDir,'Residual_IO.png'), ...
    'Resolution',300);

exportgraphics( ...
    figError, ...
    fullfile(dataDir,'ErrorComparison_IO.png'), ...
    'Resolution',300);

exportgraphics( ...
    figRMSE, ...
    fullfile(dataDir,'RMSE_IO.png'), ...
    'Resolution',300);

fprintf('\nFigures saved in:\n%s\n',dataDir);
disp('Finished.');

%% =========================================================
% Local function
% ==========================================================

function data = convertToChannelBySample(data,nChannels,dataName)

    if isempty(data)
        error('%s contains no data.',dataName);
    end

    if ~ismatrix(data)
        error('%s could not be reduced to a two-dimensional array.', ...
              dataName);
    end

    if size(data,1) == nChannels
        % Already channels-by-samples
        return;

    elseif size(data,2) == nChannels
        % Samples-by-channels
        data = data.';

    else
        error(['%s must contain %d channels. ', ...
               'Its current size is %d-by-%d.'], ...
              dataName,nChannels,size(data,1),size(data,2));
    end

end