%% Main Experiment Script for Gas Turbine Speed Controller Optimization
% Based on IC-SCA-BSO Algorithm
% This script runs all experiments from the paper

clear; clc; close all;

%% Add paths
addpath('algorithms');
addpath('models');
addpath('utils');
addpath('experiments');

%% Parameters
params.popSize = 100;           % Population size
params.maxIter = 100;           % Maximum iterations
params.numRuns = 20;            % Number of independent runs
params.dim = 3;                 % Dimension (Kp, Ki, Kd)

% PID parameter bounds
params.lb = [0, 0, 0];          % Lower bounds [Kp_min, Ki_min, Kd_min]
params.ub = [2, 1.5, 1];        % Upper bounds [Kp_max, Ki_max, Kd_max]

% IC-SCA-BSO specific parameters
params.circleA = 0.2;           % Circle mapping parameter a
params.circleB = 0.5;           % Circle mapping parameter b
params.weightA = 5;             % Weight optimization parameter a
params.scaA = 2;                % SCA parameter a
params.scaB = 0;                % SCA parameter b
params.switchProb = 0.5;        % Switching probability p

% BSO parameters
params.bso_omega = 0.7;         % Inertia weight
params.bso_c1 = 1.8;            % Individual learning factor
params.bso_c2 = 1.8;            % Global learning factor
params.bso_eta = 0.9;           % Step-size attenuation

% BAS parameters
params.bas_delta0 = 0.5;        % Initial step size
params.bas_eta = 0.95;          % Step-size attenuation

% PSO parameters
params.pso_omega_max = 0.9;     % Initial inertia weight
params.pso_omega_min = 0.4;     % Final inertia weight
params.pso_c1 = 2.0;            % Cognitive coefficient
params.pso_c2 = 2.0;            % Social coefficient

% Gas turbine parameters
params.baseSpeed = 8300;        % Base speed (r/min)
params.simTime = 20;            % Simulation time (s)
params.dt = 0.01;               % Time step (s)

%% Run Experiment 1: Algorithm Convergence Comparison
fprintf('========================================\n');
fprintf('Experiment 1: Algorithm Convergence Comparison\n');
fprintf('========================================\n');

[convergenceResults, bestParams] = run_convergence_experiment(params);

% Save results
save('results/convergence_results.mat', 'convergenceResults', 'bestParams');

% Plot convergence curves
figure('Name', 'Convergence Comparison');
plot_convergence_curves(convergenceResults, params);

%% Run Experiment 2: Tracking Tests
fprintf('\n========================================\n');
fprintf('Experiment 2: Tracking Tests\n');
fprintf('========================================\n');

speedCommands = [8340, 8380, 8420];  % Speed commands for tracking tests

trackingResults = run_tracking_experiments(bestParams, speedCommands, params);

% Save results
save('results/tracking_results.mat', 'trackingResults');

% Plot tracking results
for i = 1:length(speedCommands)
    figure('Name', sprintf('Tracking Test - %d r/min', speedCommands(i)));
    plot_tracking_results(trackingResults{i}, speedCommands(i));
end

%% Run Experiment 3: Anti-interference Test
fprintf('\n========================================\n');
fprintf('Experiment 3: Anti-interference Test\n');
fprintf('========================================\n');

disturbanceLevel = 0.05;  % 5% fuel disturbance

interferenceResults = run_interference_experiment(bestParams, disturbanceLevel, params);

% Save results
save('results/interference_results.mat', 'interferenceResults');

% Plot interference results
figure('Name', 'Anti-interference Test');
plot_interference_results(interferenceResults);

%% Display Summary Results
fprintf('\n========================================\n');
fprintf('Summary of Results\n');
fprintf('========================================\n');

display_summary_results(convergenceResults, trackingResults, interferenceResults, ...
                        bestParams, speedCommands);

fprintf('\nAll experiments completed successfully!\n');
