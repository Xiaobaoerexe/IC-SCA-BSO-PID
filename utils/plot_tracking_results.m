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
    time = trackingResult.time{i};
    speed = trackingResult.speed{i};
    
    % Zoom to interesting region
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

set(gcf, 'Position', [100, 100, 1000, 400]);

end
