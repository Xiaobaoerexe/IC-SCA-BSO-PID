function display_summary_results(convergenceResults, trackingResults, interferenceResults, bestParams, speedCommands)
%% Display Summary of All Results
% Prints comprehensive summary tables for all experiments

controllers = {'PID', 'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
displayNames = {'PID', 'BAS-PID', 'PSO-PID', 'BSO-PID', 'IC-SCA-BSO-PID'};

%% Table 4: Controller Parameters
fprintf('\n========================================\n');
fprintf('Table 4: Controller Parameters\n');
fprintf('========================================\n');
fprintf('%-15s %10s %10s %10s\n', 'Controller', 'Kp', 'Ki', 'Kd');
fprintf('--------------------------------------------\n');

for i = 1:length(controllers)
    ctrlName = controllers{i};
    p = bestParams.(ctrlName);
    fprintf('%-15s %10.4f %10.4f %10.4f\n', displayNames{i}, p.Kp, p.Ki, p.Kd);
end

%% Tracking Test Results Tables
for testIdx = 1:length(speedCommands)
    speedCmd = speedCommands(testIdx);
    result = trackingResults{testIdx};
    
    fprintf('\n========================================\n');
    fprintf('Table %d: Performance at %d r/min\n', 4 + testIdx, speedCmd);
    fprintf('========================================\n');
    fprintf('%-15s %10s %10s %12s %12s\n', 'Controller', 'tr/s', 'ts/s', 'ess', 'sigma_p');
    fprintf('-------------------------------------------------------------\n');
    
    for i = 1:length(controllers)
        ctrlName = controllers{i};
        m = result.metrics.(ctrlName);
        fprintf('%-15s %10.3f %10.3f %12.4f %12.4f\n', ...
            displayNames{i}, m.riseTime, m.settlingTime, ...
            m.steadyStateError, m.overshoot);
    end
end

%% Table 8: Anti-interference Test Results
fprintf('\n========================================\n');
fprintf('Table 8: Anti-interference Test Results\n');
fprintf('========================================\n');
fprintf('%-20s %15s %15s\n', 'Controller', 'ts/s', 'sigma_p');
fprintf('--------------------------------------------------\n');

for i = 1:length(controllers)
    ctrlName = controllers{i};
    m = interferenceResults.metrics.(ctrlName);
    fprintf('%-20s %15.3f %15.5f\n', displayNames{i}, m.settlingTime, m.overshoot);
end

%% Performance Improvement Summary
fprintf('\n========================================\n');
fprintf('Performance Improvement of IC-SCA-BSO-PID\n');
fprintf('========================================\n');

% Compare IC-SCA-BSO-PID with others for the first tracking test
result = trackingResults{1};
ic_metrics = result.metrics.IC_SCA_BSO;

fprintf('\nTracking Test at %d r/min:\n', speedCommands(1));
fprintf('%-20s %20s %20s\n', 'vs Controller', 'ts Reduction (%)', 'sigma Reduction (%)');
fprintf('-------------------------------------------------------------\n');

for i = 1:length(controllers)-1
    ctrlName = controllers{i};
    m = result.metrics.(ctrlName);
    
    ts_reduction = (m.settlingTime - ic_metrics.settlingTime) / m.settlingTime * 100;
    sigma_reduction = (m.overshoot - ic_metrics.overshoot) / m.overshoot * 100;
    
    fprintf('%-20s %20.1f %20.1f\n', displayNames{i}, ts_reduction, sigma_reduction);
end

% Compare for interference test
fprintf('\nAnti-interference Test:\n');
fprintf('%-20s %20s %20s\n', 'vs Controller', 'ts Reduction (%)', 'sigma Reduction (%)');
fprintf('-------------------------------------------------------------\n');

ic_int_metrics = interferenceResults.metrics.IC_SCA_BSO;

for i = 1:length(controllers)-1
    ctrlName = controllers{i};
    m = interferenceResults.metrics.(ctrlName);
    
    ts_reduction = (m.settlingTime - ic_int_metrics.settlingTime) / m.settlingTime * 100;
    sigma_reduction = (m.overshoot - ic_int_metrics.overshoot) / m.overshoot * 100;
    
    fprintf('%-20s %20.1f %20.1f\n', displayNames{i}, ts_reduction, sigma_reduction);
end

%% Algorithm Convergence Statistics
fprintf('\n========================================\n');
fprintf('Algorithm Convergence Statistics\n');
fprintf('========================================\n');
fprintf('%-15s %15s %15s %15s\n', 'Algorithm', 'Mean ITAE', 'Std ITAE', 'Best ITAE');
fprintf('-------------------------------------------------------------\n');

algorithms = convergenceResults.algorithms;
for i = 1:length(algorithms)
    meanFit = mean(convergenceResults.allBestFitness(:, i));
    stdFit = std(convergenceResults.allBestFitness(:, i));
    bestFit = min(convergenceResults.allBestFitness(:, i));
    
    fprintf('%-15s %15.4f %15.4f %15.4f\n', ...
        strrep(algorithms{i}, '_', '-'), meanFit, stdFit, bestFit);
end

fprintf('\nANOVA Results: F = %.4f, p = %.4f\n', ...
    convergenceResults.anovaF, convergenceResults.anovaP);

end
