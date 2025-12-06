function plot_convergence_curves(convergenceResults, params)
%% Plot Convergence Curves
% Creates the convergence comparison figure similar to Figure 8 in the paper

meanConvergence = convergenceResults.meanConvergence;
algorithms = convergenceResults.algorithms;
maxIter = params.maxIter;

% Color scheme for algorithms
colors = [
    0.8500, 0.3250, 0.0980;  % BAS - orange
    0.0000, 0.4470, 0.7410;  % PSO - blue
    0.4660, 0.6740, 0.1880;  % BSO - green
    0.6350, 0.0780, 0.1840;  % IC-SCA-BSO - dark red
];

lineStyles = {'-', '--', '-.', '-'};
markers = {'o', 's', 'd', '^'};

% Main convergence plot
subplot(1, 2, 1);
hold on;
for i = 1:length(algorithms)
    plot(1:maxIter, meanConvergence(:, i), ...
        'Color', colors(i, :), ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.5, ...
        'DisplayName', strrep(algorithms{i}, '_', '-'));
end
hold off;
xlabel('Iteration');
ylabel('ITAE');
title('(a) Complete Convergence Curves');
legend('Location', 'northeast');
grid on;
set(gca, 'FontSize', 10);

% Local zoom (iterations 20-50)
subplot(1, 2, 2);
hold on;
zoomRange = 20:50;
for i = 1:length(algorithms)
    plot(zoomRange, meanConvergence(zoomRange, i), ...
        'Color', colors(i, :), ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.5, ...
        'DisplayName', strrep(algorithms{i}, '_', '-'));
end
hold off;
xlabel('Iteration');
ylabel('ITAE');
title('(b) Local Zoom (Iterations 20-50)');
legend('Location', 'northeast');
grid on;
set(gca, 'FontSize', 10);

% Adjust figure size
set(gcf, 'Position', [100, 100, 1000, 400]);

end

function plot_tracking_results(trackingResult, speedCommand)
%% Plot Tracking Test Results
% Creates the speed response curve similar to Figures 9, 10, 11 in the paper

controllers = {'PID', 'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
displayNames = {'PID', 'BAS-PID', 'PSO-PID', 'BSO-PID', 'IC-SCA-BSO-PID'};

% Color scheme
colors = [
    0.5, 0.5, 0.5;           % PID - gray
    0.8500, 0.3250, 0.0980;  % BAS - orange
    0.0000, 0.4470, 0.7410;  % PSO - blue
    0.4660, 0.6740, 0.1880;  % BSO - green
    0.6350, 0.0780, 0.1840;  % IC-SCA-BSO - dark red
];

lineStyles = {'-', '--', '-.', ':', '-'};

% Full response curve
subplot(1, 2, 1);
hold on;
for i = 1:length(controllers)
    plot(trackingResult.time{i}, trackingResult.speed{i}, ...
        'Color', colors(i, :), ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', displayNames{i});
end
yline(speedCommand, 'k--', 'LineWidth', 1, 'DisplayName', 'Command');
hold off;
xlabel('Time (s)');
ylabel('Speed (r/min)');
title(sprintf('(a) Speed Response - %d r/min', speedCommand));
legend('Location', 'southeast');
grid on;
set(gca, 'FontSize', 10);

% Locally enlarged curve
subplot(1, 2, 2);
hold on;
for i = 1:length(controllers)
    % Find the region around the overshoot
    time = trackingResult.time{i};
    speed = trackingResult.speed{i};
    
    % Zoom to interesting region (1s to 8s)
    zoomIdx = time >= 1 & time <= 8;
    
    plot(time(zoomIdx), speed(zoomIdx), ...
        'Color', colors(i, :), ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', displayNames{i});
end
yline(speedCommand, 'k--', 'LineWidth', 1);
hold off;
xlabel('Time (s)');
ylabel('Speed (r/min)');
title('(b) Locally-enlarged Curve');
legend('Location', 'southeast');
grid on;
set(gca, 'FontSize', 10);

% Adjust figure size
set(gcf, 'Position', [100, 100, 1000, 400]);

end

function plot_interference_results(interferenceResults)
%% Plot Anti-interference Test Results
% Creates the speed response curve similar to Figure 12 in the paper

controllers = interferenceResults.controllers;
displayNames = {'PID', 'BAS-PID', 'PSO-PID', 'BSO-PID', 'IC-SCA-BSO-PID'};

% Color scheme
colors = [
    0.5, 0.5, 0.5;           % PID - gray
    0.8500, 0.3250, 0.0980;  % BAS - orange
    0.0000, 0.4470, 0.7410;  % PSO - blue
    0.4660, 0.6740, 0.1880;  % BSO - green
    0.6350, 0.0780, 0.1840;  % IC-SCA-BSO - dark red
];

lineStyles = {'-', '--', '-.', ':', '-'};

targetSpeed = 8300;  % Base speed

% Full response curve
subplot(1, 2, 1);
hold on;
for i = 1:length(controllers)
    plot(interferenceResults.time{i}, interferenceResults.speed{i}, ...
        'Color', colors(i, :), ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', displayNames{i});
end
% Mark disturbance time
xline(interferenceResults.disturbanceTime, 'k:', 'LineWidth', 1.5, ...
    'DisplayName', 'Disturbance');
yline(targetSpeed, 'k--', 'LineWidth', 1, 'DisplayName', 'Target');
hold off;
xlabel('Time (s)');
ylabel('Speed (r/min)');
title(sprintf('(a) Speed Response with %.0f%% Fuel Disturbance', ...
    interferenceResults.disturbanceLevel * 100));
legend('Location', 'southeast');
grid on;
set(gca, 'FontSize', 10);

% Locally enlarged curve around disturbance
subplot(1, 2, 2);
hold on;
for i = 1:length(controllers)
    time = interferenceResults.time{i};
    speed = interferenceResults.speed{i};
    
    % Zoom to disturbance region
    zoomIdx = time >= 3 & time <= 12;
    
    plot(time(zoomIdx), speed(zoomIdx), ...
        'Color', colors(i, :), ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', 1.2, ...
        'DisplayName', displayNames{i});
end
xline(interferenceResults.disturbanceTime, 'k:', 'LineWidth', 1.5);
yline(targetSpeed, 'k--', 'LineWidth', 1);
hold off;
xlabel('Time (s)');
ylabel('Speed (r/min)');
title('(b) Locally-enlarged Curve');
legend('Location', 'southeast');
grid on;
set(gca, 'FontSize', 10);

% Adjust figure size
set(gcf, 'Position', [100, 100, 1000, 400]);

end
