function [convergenceResults, bestParams] = run_convergence_experiment(params)
%% Run Convergence Experiment
% Compares convergence performance of BAS, PSO, BSO, and IC-SCA-BSO
% algorithms over multiple independent runs
%
% Inputs:
%   params - Experiment parameters
%
% Outputs:
%   convergenceResults - Structure containing convergence data
%   bestParams         - Best PID parameters found by each algorithm

numRuns = params.numRuns;
maxIter = params.maxIter;

% Create fitness function handle
fitFunc = @(x) fitness_ITAE(x, params);

% Initialize result storage
algorithms = {'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
numAlgo = length(algorithms);

allConvergence = zeros(maxIter, numRuns, numAlgo);
allBestFitness = zeros(numRuns, numAlgo);
allBestSolutions = zeros(numRuns, 3, numAlgo);

%% Run experiments for each algorithm
for algoIdx = 1:numAlgo
    algoName = algorithms{algoIdx};
    fprintf('\nRunning %s algorithm...\n', algoName);
    
    for run = 1:numRuns
        fprintf('  Run %d/%d\n', run, numRuns);
        
        % Set random seed for reproducibility within each run
        rng(run);
        
        % Run optimization algorithm
        switch algoName
            case 'BAS'
                [bestSol, bestFit, convCurve] = BAS(fitFunc, params);
            case 'PSO'
                [bestSol, bestFit, convCurve] = PSO(fitFunc, params);
            case 'BSO'
                [bestSol, bestFit, convCurve] = BSO(fitFunc, params);
            case 'IC_SCA_BSO'
                [bestSol, bestFit, convCurve] = IC_SCA_BSO(fitFunc, params);
        end
        
        allConvergence(:, run, algoIdx) = convCurve;
        allBestFitness(run, algoIdx) = bestFit;
        allBestSolutions(run, :, algoIdx) = bestSol;
    end
    
    fprintf('%s completed. Mean ITAE: %.4f\n', algoName, mean(allBestFitness(:, algoIdx)));
end

%% Calculate statistics
meanConvergence = squeeze(mean(allConvergence, 2));
stdConvergence = squeeze(std(allConvergence, 0, 2));

%% Find best parameters for each algorithm
bestParams = struct();
for algoIdx = 1:numAlgo
    algoName = algorithms{algoIdx};
    [~, bestRunIdx] = min(allBestFitness(:, algoIdx));
    
    bestParams.(algoName).Kp = allBestSolutions(bestRunIdx, 1, algoIdx);
    bestParams.(algoName).Ki = allBestSolutions(bestRunIdx, 2, algoIdx);
    bestParams.(algoName).Kd = allBestSolutions(bestRunIdx, 3, algoIdx);
    bestParams.(algoName).fitness = allBestFitness(bestRunIdx, algoIdx);
end

% Add traditional PID parameters (manual tuning based on paper)
bestParams.PID.Kp = 0.0941;
bestParams.PID.Ki = 0.0644;
bestParams.PID.Kd = 0.1672;
bestParams.PID.fitness = fitFunc([bestParams.PID.Kp, bestParams.PID.Ki, bestParams.PID.Kd]);

%% Perform one-way ANOVA
[p, tbl, stats] = anova1(allBestFitness, algorithms, 'off');
F_statistic = tbl{2, 5};

fprintf('\n========================================\n');
fprintf('One-way ANOVA Results:\n');
fprintf('F-statistic: %.4f\n', F_statistic);
fprintf('p-value: %.4f\n', p);
fprintf('========================================\n');

%% Store results
convergenceResults.algorithms = algorithms;
convergenceResults.allConvergence = allConvergence;
convergenceResults.meanConvergence = meanConvergence;
convergenceResults.stdConvergence = stdConvergence;
convergenceResults.allBestFitness = allBestFitness;
convergenceResults.allBestSolutions = allBestSolutions;
convergenceResults.anovaF = F_statistic;
convergenceResults.anovaP = p;

%% Display best parameters
fprintf('\n========================================\n');
fprintf('Best PID Parameters:\n');
fprintf('========================================\n');
fprintf('%-15s %10s %10s %10s %12s\n', 'Controller', 'Kp', 'Ki', 'Kd', 'ITAE');
fprintf('%-15s %10.4f %10.4f %10.4f %12.4f\n', 'PID', ...
    bestParams.PID.Kp, bestParams.PID.Ki, bestParams.PID.Kd, bestParams.PID.fitness);
fprintf('%-15s %10.4f %10.4f %10.4f %12.4f\n', 'BAS-PID', ...
    bestParams.BAS.Kp, bestParams.BAS.Ki, bestParams.BAS.Kd, bestParams.BAS.fitness);
fprintf('%-15s %10.4f %10.4f %10.4f %12.4f\n', 'PSO-PID', ...
    bestParams.PSO.Kp, bestParams.PSO.Ki, bestParams.PSO.Kd, bestParams.PSO.fitness);
fprintf('%-15s %10.4f %10.4f %10.4f %12.4f\n', 'BSO-PID', ...
    bestParams.BSO.Kp, bestParams.BSO.Ki, bestParams.BSO.Kd, bestParams.BSO.fitness);
fprintf('%-15s %10.4f %10.4f %10.4f %12.4f\n', 'IC-SCA-BSO-PID', ...
    bestParams.IC_SCA_BSO.Kp, bestParams.IC_SCA_BSO.Ki, bestParams.IC_SCA_BSO.Kd, ...
    bestParams.IC_SCA_BSO.fitness);

end
