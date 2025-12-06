%% Standalone Demo Script for IC-SCA-BSO PID Optimization
% This script demonstrates all the algorithms without requiring separate function files
% All functions are embedded in this single file

clear; clc; close all;
fprintf('========================================\n');
fprintf('Gas Turbine Speed Controller Optimization\n');
fprintf('Based on IC-SCA-BSO Algorithm\n');
fprintf('========================================\n\n');

%% Parameters
params.popSize = 100;           % Population size
params.maxIter = 100;           % Maximum iterations
params.numRuns = 5;             % Number of independent runs (reduced for demo)
params.dim = 3;                 % Dimension (Kp, Ki, Kd)

% PID parameter bounds
params.lb = [0, 0, 0];          % Lower bounds
params.ub = [2, 1.5, 1];        % Upper bounds

% Algorithm parameters
params.circleA = 0.2;
params.circleB = 0.5;
params.weightA = 5;
params.scaA = 2;
params.scaB = 0;
params.switchProb = 0.5;
params.bso_omega = 0.7;
params.bso_c1 = 1.8;
params.bso_c2 = 1.8;
params.bso_eta = 0.9;
params.bas_delta0 = 0.5;
params.bas_eta = 0.95;
params.pso_omega_max = 0.9;
params.pso_omega_min = 0.4;
params.pso_c1 = 2.0;
params.pso_c2 = 2.0;

% Simulation parameters
params.baseSpeed = 8300;
params.simTime = 20;
params.dt = 0.01;

%% Create fitness function
fitFunc = @(x) fitness_func(x, params);

%% Run Algorithm Comparison
fprintf('Running algorithm comparison (%d runs each)...\n\n', params.numRuns);

algorithms = {'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
numAlgo = length(algorithms);
allBestFitness = zeros(params.numRuns, numAlgo);
allConvergence = zeros(params.maxIter, params.numRuns, numAlgo);
bestSolutions = zeros(numAlgo, 3);
bestFitnessValues = zeros(numAlgo, 1);

for algoIdx = 1:numAlgo
    algoName = algorithms{algoIdx};
    fprintf('Running %s...\n', strrep(algoName, '_', '-'));
    
    for run = 1:params.numRuns
        rng(run);
        
        switch algoName
            case 'BAS'
                [bestSol, bestFit, convCurve] = run_BAS(fitFunc, params);
            case 'PSO'
                [bestSol, bestFit, convCurve] = run_PSO(fitFunc, params);
            case 'BSO'
                [bestSol, bestFit, convCurve] = run_BSO(fitFunc, params);
            case 'IC_SCA_BSO'
                [bestSol, bestFit, convCurve] = run_IC_SCA_BSO(fitFunc, params);
        end
        
        allConvergence(:, run, algoIdx) = convCurve;
        allBestFitness(run, algoIdx) = bestFit;
        
        if run == 1 || bestFit < bestFitnessValues(algoIdx)
            bestFitnessValues(algoIdx) = bestFit;
            bestSolutions(algoIdx, :) = bestSol;
        end
    end
    
    fprintf('  Best ITAE: %.4f, Mean ITAE: %.4f\n', ...
        min(allBestFitness(:, algoIdx)), mean(allBestFitness(:, algoIdx)));
end

%% Display PID Parameters Table (Table 4)
fprintf('\n========================================\n');
fprintf('Table 4: Controller Parameters\n');
fprintf('========================================\n');
fprintf('%-15s %10s %10s %10s %12s\n', 'Controller', 'Kp', 'Ki', 'Kd', 'ITAE');
fprintf('-----------------------------------------------------\n');

% Traditional PID (from paper)
PID_params = [0.0941, 0.0644, 0.1672];
fprintf('%-15s %10.4f %10.4f %10.4f %12.4f\n', 'PID', ...
    PID_params(1), PID_params(2), PID_params(3), fitFunc(PID_params));

for i = 1:numAlgo
    fprintf('%-15s %10.4f %10.4f %10.4f %12.4f\n', ...
        [strrep(algorithms{i}, '_', '-') '-PID'], ...
        bestSolutions(i, 1), bestSolutions(i, 2), bestSolutions(i, 3), ...
        bestFitnessValues(i));
end

%% Plot Convergence Curves (Figure 8)
meanConvergence = squeeze(mean(allConvergence, 2));

figure('Name', 'Figure 8: Convergence Comparison');
colors = {'#D95319', '#0072BD', '#77AC30', '#A2142F'};
lineStyles = {'-', '--', '-.', '-'};

subplot(1, 2, 1);
hold on;
for i = 1:numAlgo
    plot(1:params.maxIter, meanConvergence(:, i), ...
        'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', 1.5);
end
hold off;
xlabel('Iteration');
ylabel('ITAE');
title('(a) Complete Convergence Curves');
legend('BAS', 'PSO', 'BSO', 'IC-SCA-BSO', 'Location', 'northeast');
grid on;

subplot(1, 2, 2);
hold on;
zoomRange = 20:50;
for i = 1:numAlgo
    plot(zoomRange, meanConvergence(zoomRange, i), ...
        'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', 1.5);
end
hold off;
xlabel('Iteration');
ylabel('ITAE');
title('(b) Local Zoom (Iterations 20-50)');
legend('BAS', 'PSO', 'BSO', 'IC-SCA-BSO', 'Location', 'northeast');
grid on;
set(gcf, 'Position', [100, 100, 1000, 400]);

%% Run Tracking Tests
fprintf('\n========================================\n');
fprintf('Running Tracking Tests...\n');
fprintf('========================================\n');

speedCommands = [8340, 8380, 8420];
controllers = {'PID', 'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
allParams = [PID_params; bestSolutions];

for testIdx = 1:length(speedCommands)
    speedCmd = speedCommands(testIdx);
    fprintf('\nSpeed Command: %d r/min\n', speedCmd);
    fprintf('%-15s %10s %10s %12s %12s\n', 'Controller', 'tr/s', 'ts/s', 'ess', 'sigma_p');
    fprintf('-------------------------------------------------------------\n');
    
    figure('Name', sprintf('Tracking Test - %d r/min', speedCmd));
    
    for ctrlIdx = 1:5
        Kp = allParams(ctrlIdx, 1);
        Ki = allParams(ctrlIdx, 2);
        Kd = allParams(ctrlIdx, 3);
        
        [time, speed] = simulate_gas_turbine(Kp, Ki, Kd, speedCmd, params);
        
        % Calculate metrics
        metrics = calc_metrics(time, speed, speedCmd, params.dt);
        
        fprintf('%-15s %10.3f %10.3f %12.4f %12.4f\n', ...
            [controllers{ctrlIdx} '-PID'], metrics.tr, metrics.ts, metrics.ess, metrics.sigma);
        
        % Plot
        subplot(1, 2, 1);
        hold on;
        plot(time, speed, 'LineWidth', 1.2);
    end
    
    yline(speedCmd, 'k--', 'LineWidth', 1);
    hold off;
    xlabel('Time (s)');
    ylabel('Speed (r/min)');
    title('(a) Speed Response');
    legend('PID', 'BAS-PID', 'PSO-PID', 'BSO-PID', 'IC-SCA-BSO-PID', 'Command', ...
        'Location', 'southeast');
    grid on;
    
    subplot(1, 2, 2);
    % Re-plot zoomed
    for ctrlIdx = 1:5
        [time, speed] = simulate_gas_turbine(allParams(ctrlIdx, 1), allParams(ctrlIdx, 2), ...
            allParams(ctrlIdx, 3), speedCmd, params);
        zoomIdx = time >= 1 & time <= 8;
        hold on;
        plot(time(zoomIdx), speed(zoomIdx), 'LineWidth', 1.2);
    end
    yline(speedCmd, 'k--', 'LineWidth', 1);
    hold off;
    xlabel('Time (s)');
    ylabel('Speed (r/min)');
    title('(b) Zoomed View');
    legend('PID', 'BAS-PID', 'PSO-PID', 'BSO-PID', 'IC-SCA-BSO-PID', ...
        'Location', 'southeast');
    grid on;
    set(gcf, 'Position', [100, 100, 1000, 400]);
end

%% Run Anti-interference Test
fprintf('\n========================================\n');
fprintf('Anti-interference Test (5%% Disturbance)\n');
fprintf('========================================\n');
fprintf('%-15s %10s %12s\n', 'Controller', 'ts/s', 'sigma_p');
fprintf('---------------------------------------------\n');

figure('Name', 'Anti-interference Test');

for ctrlIdx = 1:5
    Kp = allParams(ctrlIdx, 1);
    Ki = allParams(ctrlIdx, 2);
    Kd = allParams(ctrlIdx, 3);
    
    [time, speed] = simulate_with_disturbance(Kp, Ki, Kd, params, 0.05, 4);
    
    % Calculate metrics
    [ts, sigma] = calc_disturbance_metrics(time, speed, params.baseSpeed, 4, params.dt);
    
    fprintf('%-15s %10.3f %12.5f\n', [controllers{ctrlIdx} '-PID'], ts, sigma);
    
    subplot(1, 2, 1);
    hold on;
    plot(time, speed, 'LineWidth', 1.2);
end

xline(4, 'k:', 'LineWidth', 1.5);
yline(params.baseSpeed, 'k--', 'LineWidth', 1);
hold off;
xlabel('Time (s)');
ylabel('Speed (r/min)');
title('(a) Speed Response with 5% Fuel Disturbance');
legend('PID', 'BAS-PID', 'PSO-PID', 'BSO-PID', 'IC-SCA-BSO-PID', 'Disturbance', 'Target', ...
    'Location', 'southeast');
grid on;

subplot(1, 2, 2);
for ctrlIdx = 1:5
    [time, speed] = simulate_with_disturbance(allParams(ctrlIdx, 1), allParams(ctrlIdx, 2), ...
        allParams(ctrlIdx, 3), params, 0.05, 4);
    zoomIdx = time >= 3 & time <= 12;
    hold on;
    plot(time(zoomIdx), speed(zoomIdx), 'LineWidth', 1.2);
end
xline(4, 'k:', 'LineWidth', 1.5);
yline(params.baseSpeed, 'k--', 'LineWidth', 1);
hold off;
xlabel('Time (s)');
ylabel('Speed (r/min)');
title('(b) Zoomed View');
legend('PID', 'BAS-PID', 'PSO-PID', 'BSO-PID', 'IC-SCA-BSO-PID', ...
    'Location', 'southeast');
grid on;
set(gcf, 'Position', [100, 100, 1000, 400]);

%% Plot Weight Comparison (Figure 5)
figure('Name', 'Figure 5: Weight Comparison');
iterations = 1:params.maxIter;
omega1 = zeros(1, params.maxIter);
for k = 1:params.maxIter
    omega1(k) = 0.5 * ((1 - log(1 + (exp(1)-1) * k/params.maxIter)) + exp(-k/params.weightA));
end
omega2 = 0.9 - (0.9 - 0.4) * (iterations - 1) / (params.maxIter - 1);

plot(iterations, omega1, 'b-', 'LineWidth', 2);
hold on;
plot(iterations, omega2, 'r--', 'LineWidth', 2);
hold off;
xlabel('Iteration');
ylabel('Weight \omega');
title('Figure 5: Comparison of Weight \omega');
legend('\omega_1 (Nonlinear)', '\omega_2 (Linear)', 'Location', 'northeast');
grid on;

fprintf('\n========================================\n');
fprintf('All experiments completed!\n');
fprintf('========================================\n');

%% ========== EMBEDDED FUNCTIONS ==========

%% Fitness Function
function ITAE = fitness_func(pidParams, params)
    Kp = pidParams(1);
    Ki = pidParams(2);
    Kd = pidParams(3);
    
    speedCommand = params.baseSpeed + 40;
    [time, speed] = simulate_gas_turbine(Kp, Ki, Kd, speedCommand, params);
    
    error = speedCommand - speed;
    ITAE = sum(time .* abs(error)) * params.dt;
    
    % Penalty for overshoot > 5%
    overshoot = max(0, max(speed) - speedCommand) / speedCommand * 100;
    if overshoot > 5
        ITAE = ITAE + 500 * (overshoot - 5)^2;
    end
end

%% Gas Turbine Simulation
function [time, speed] = simulate_gas_turbine(Kp, Ki, Kd, speedCommand, params)
    numSteps = round(params.simTime / params.dt) + 1;
    time = (0:numSteps-1)' * params.dt;
    speed = zeros(numSteps, 1);
    
    tau1 = 0.5; tau2 = 1.5; K_gt = 100;
    qf_max = 2.5; qf_min = 0.1; dqf_max = 0.5;
    
    x1 = 0; x2 = 0;
    speed(1) = params.baseSpeed;
    qf_prev = 0.8;
    integral_error = 0; prev_error = 0;
    
    for k = 2:numSteps
        error = speedCommand - speed(k-1);
        P = Kp * error;
        integral_error = max(-100, min(100, integral_error + error * params.dt));
        I = Ki * integral_error;
        D = Kd * (error - prev_error) / params.dt;
        prev_error = error;
        
        qf = 0.8 + (P + I + D) * 0.1;
        dqf = (qf - qf_prev) / params.dt;
        if abs(dqf) > dqf_max
            qf = qf_prev + sign(dqf) * dqf_max * params.dt;
        end
        qf = max(qf_min, min(qf_max, qf));
        qf_prev = qf;
        
        x1 = x1 + (-x1 + qf) / tau1 * params.dt;
        x2 = x2 + (-x2 + K_gt * x1) / tau2 * params.dt;
        speed(k) = params.baseSpeed + x2;
    end
end

%% Simulation with Disturbance
function [time, speed] = simulate_with_disturbance(Kp, Ki, Kd, params, distLevel, distTime)
    numSteps = round(params.simTime / params.dt) + 1;
    time = (0:numSteps-1)' * params.dt;
    speed = zeros(numSteps, 1);
    
    tau1 = 0.5; tau2 = 1.5; K_gt = 100;
    qf_max = 2.5; qf_min = 0.1; dqf_max = 0.5;
    
    x1 = 0; x2 = 0;
    speed(1) = params.baseSpeed;
    qf_prev = 0.8;
    integral_error = 0; prev_error = 0;
    
    for k = 2:numSteps
        t = time(k);
        error = params.baseSpeed - speed(k-1);
        P = Kp * error;
        integral_error = max(-50, min(50, integral_error + error * params.dt));
        I = Ki * integral_error;
        D = Kd * (error - prev_error) / params.dt;
        prev_error = error;
        
        qf = 0.8 + (P + I + D) * 0.1;
        dqf = (qf - qf_prev) / params.dt;
        if abs(dqf) > dqf_max
            qf = qf_prev + sign(dqf) * dqf_max * params.dt;
        end
        qf = max(qf_min, min(qf_max, qf));
        qf_prev = qf;
        
        if t >= distTime
            qf_disturbed = qf * (1 + distLevel);
        else
            qf_disturbed = qf;
        end
        
        x1 = x1 + (-x1 + qf_disturbed) / tau1 * params.dt;
        x2 = x2 + (-x2 + K_gt * x1) / tau2 * params.dt;
        speed(k) = params.baseSpeed + x2;
    end
end

%% Calculate Tracking Metrics
function metrics = calc_metrics(time, speed, speedCmd, dt)
    speedChange = speedCmd - speed(1);
    idx10 = find(speed >= speed(1) + 0.1*speedChange, 1);
    idx90 = find(speed >= speed(1) + 0.9*speedChange, 1);
    if ~isempty(idx10) && ~isempty(idx90)
        metrics.tr = time(idx90) - time(idx10);
    else
        metrics.tr = Inf;
    end
    
    tolerance = 0.02 * speedCmd;
    error = abs(speed - speedCmd);
    settledIdx = find(error < tolerance, 1);
    if ~isempty(settledIdx)
        metrics.ts = time(settledIdx);
    else
        metrics.ts = time(end);
    end
    
    numSamples = round(2 / dt);
    metrics.ess = var(speed(end-numSamples:end) - speedCmd);
    
    peakSpeed = max(speed);
    if peakSpeed > speedCmd
        metrics.sigma = (peakSpeed - speedCmd) / (speedCmd - speed(1));
    else
        metrics.sigma = 0;
    end
end

%% Calculate Disturbance Metrics
function [ts, sigma] = calc_disturbance_metrics(time, speed, targetSpeed, distTime, dt)
    distIdx = find(time >= distTime, 1);
    speed_post = speed(distIdx:end);
    time_post = time(distIdx:end) - distTime;
    
    deviation = abs(speed_post - targetSpeed);
    sigma = max(deviation) / targetSpeed;
    
    tolerance = 0.02 * targetSpeed;
    error = abs(speed_post - targetSpeed);
    settledIdx = find(error < tolerance, 1);
    if ~isempty(settledIdx)
        ts = time_post(settledIdx) + distTime;
    else
        ts = time(end);
    end
end

%% IC-SCA-BSO Algorithm
function [bestSol, bestFit, convCurve] = run_IC_SCA_BSO(fitFunc, params)
    popSize = params.popSize;
    maxIter = params.maxIter;
    dim = params.dim;
    lb = params.lb;
    ub = params.ub;
    
    % Circle mapping initialization
    X = zeros(popSize, dim);
    chaos = rand(1, dim);
    for i = 1:popSize
        chaos = chaos + params.circleA - mod((params.circleB/(2*pi)) * sin(2*pi*chaos), 1);
        chaos = mod(chaos, 1);
        X(i, :) = lb + chaos .* (ub - lb);
    end
    
    V = zeros(popSize, dim);
    delta = ones(popSize, 1) * 0.5;
    
    fitness = zeros(popSize, 1);
    for i = 1:popSize
        fitness(i) = fitFunc(X(i, :));
    end
    
    pbest = X;
    pbestFitness = fitness;
    [bestFit, bestIdx] = min(fitness);
    gbest = X(bestIdx, :);
    
    convCurve = zeros(maxIter, 1);
    
    for iter = 1:maxIter
        omega = 0.5 * ((1 - log(1 + (exp(1)-1) * iter/maxIter)) + exp(-iter/params.weightA));
        r1 = params.scaA - (params.scaA - params.scaB) * log(1 + (exp(1)-1) * iter/maxIter);
        
        for i = 1:popSize
            r2 = 2 * pi * rand();
            p = rand();
            
            if p < params.switchProb
                V(i, :) = omega * V(i, :) + r1 * sin(r2) * (pbest(i, :) - X(i, :)) + ...
                          r1 * sin(r2) * (gbest - X(i, :));
            else
                V(i, :) = omega * V(i, :) + r1 * cos(r2) * (pbest(i, :) - X(i, :)) + ...
                          r1 * cos(r2) * (gbest - X(i, :));
            end
            
            d = delta(i) / 1.8;
            direction = V(i, :) / (norm(V(i, :)) + eps);
            X_left = max(min(X(i, :) + direction * d/2, ub), lb);
            X_right = max(min(X(i, :) - direction * d/2, ub), lb);
            
            f_left = fitFunc(X_left);
            f_right = fitFunc(X_right);
            
            xi = delta(i) * direction * sign(f_left - f_right);
            X(i, :) = max(min(X(i, :) + 0.5 * V(i, :) + 0.5 * xi, ub), lb);
            
            delta(i) = params.bso_eta * delta(i);
            fitness(i) = fitFunc(X(i, :));
            
            if fitness(i) < pbestFitness(i)
                pbest(i, :) = X(i, :);
                pbestFitness(i) = fitness(i);
            end
        end
        
        [minFit, minIdx] = min(pbestFitness);
        if minFit < bestFit
            bestFit = minFit;
            gbest = pbest(minIdx, :);
        end
        convCurve(iter) = bestFit;
    end
    bestSol = gbest;
end

%% BSO Algorithm
function [bestSol, bestFit, convCurve] = run_BSO(fitFunc, params)
    popSize = params.popSize;
    maxIter = params.maxIter;
    dim = params.dim;
    lb = params.lb;
    ub = params.ub;
    
    X = zeros(popSize, dim);
    for i = 1:popSize
        X(i, :) = lb + rand(1, dim) .* (ub - lb);
    end
    
    V = zeros(popSize, dim);
    delta = ones(popSize, 1) * 0.5;
    
    fitness = zeros(popSize, 1);
    for i = 1:popSize
        fitness(i) = fitFunc(X(i, :));
    end
    
    pbest = X;
    pbestFitness = fitness;
    [bestFit, bestIdx] = min(fitness);
    gbest = X(bestIdx, :);
    
    convCurve = zeros(maxIter, 1);
    
    for iter = 1:maxIter
        for i = 1:popSize
            r1 = rand(); r2 = rand();
            V(i, :) = params.bso_omega * V(i, :) + ...
                      params.bso_c1 * r1 * (pbest(i, :) - X(i, :)) + ...
                      params.bso_c2 * r2 * (gbest - X(i, :));
            
            d = delta(i) / params.bso_c2;
            direction = V(i, :) / (norm(V(i, :)) + eps);
            X_left = max(min(X(i, :) + direction * d/2, ub), lb);
            X_right = max(min(X(i, :) - direction * d/2, ub), lb);
            
            f_left = fitFunc(X_left);
            f_right = fitFunc(X_right);
            
            xi = delta(i) * direction * sign(f_left - f_right);
            X(i, :) = max(min(X(i, :) + 0.5 * V(i, :) + 0.5 * xi, ub), lb);
            
            delta(i) = params.bso_eta * delta(i);
            fitness(i) = fitFunc(X(i, :));
            
            if fitness(i) < pbestFitness(i)
                pbest(i, :) = X(i, :);
                pbestFitness(i) = fitness(i);
            end
        end
        
        [minFit, minIdx] = min(pbestFitness);
        if minFit < bestFit
            bestFit = minFit;
            gbest = pbest(minIdx, :);
        end
        convCurve(iter) = bestFit;
    end
    bestSol = gbest;
end

%% PSO Algorithm
function [bestSol, bestFit, convCurve] = run_PSO(fitFunc, params)
    popSize = params.popSize;
    maxIter = params.maxIter;
    dim = params.dim;
    lb = params.lb;
    ub = params.ub;
    
    Vmax = 0.2 * (ub - lb);
    Vmin = -Vmax;
    
    X = zeros(popSize, dim);
    V = zeros(popSize, dim);
    for i = 1:popSize
        X(i, :) = lb + rand(1, dim) .* (ub - lb);
        V(i, :) = Vmin + rand(1, dim) .* (Vmax - Vmin);
    end
    
    fitness = zeros(popSize, 1);
    for i = 1:popSize
        fitness(i) = fitFunc(X(i, :));
    end
    
    pbest = X;
    pbestFitness = fitness;
    [bestFit, bestIdx] = min(fitness);
    gbest = X(bestIdx, :);
    
    convCurve = zeros(maxIter, 1);
    
    for iter = 1:maxIter
        omega = params.pso_omega_max - (params.pso_omega_max - params.pso_omega_min) * iter / maxIter;
        
        for i = 1:popSize
            r1 = rand(1, dim); r2 = rand(1, dim);
            V(i, :) = omega * V(i, :) + ...
                      params.pso_c1 * r1 .* (pbest(i, :) - X(i, :)) + ...
                      params.pso_c2 * r2 .* (gbest - X(i, :));
            V(i, :) = max(min(V(i, :), Vmax), Vmin);
            
            X(i, :) = max(min(X(i, :) + V(i, :), ub), lb);
            fitness(i) = fitFunc(X(i, :));
            
            if fitness(i) < pbestFitness(i)
                pbest(i, :) = X(i, :);
                pbestFitness(i) = fitness(i);
            end
        end
        
        [minFit, minIdx] = min(pbestFitness);
        if minFit < bestFit
            bestFit = minFit;
            gbest = pbest(minIdx, :);
        end
        convCurve(iter) = bestFit;
    end
    bestSol = gbest;
end

%% BAS Algorithm
function [bestSol, bestFit, convCurve] = run_BAS(fitFunc, params)
    popSize = params.popSize;
    maxIter = params.maxIter;
    dim = params.dim;
    lb = params.lb;
    ub = params.ub;
    
    X = zeros(popSize, dim);
    for i = 1:popSize
        X(i, :) = lb + rand(1, dim) .* (ub - lb);
    end
    
    delta = ones(popSize, 1) * params.bas_delta0;
    
    fitness = zeros(popSize, 1);
    for i = 1:popSize
        fitness(i) = fitFunc(X(i, :));
    end
    
    [bestFit, bestIdx] = min(fitness);
    bestSol = X(bestIdx, :);
    
    convCurve = zeros(maxIter, 1);
    
    for iter = 1:maxIter
        for i = 1:popSize
            direction = randn(1, dim);
            direction = direction / (norm(direction) + eps);
            
            d = delta(i);
            X_left = max(min(X(i, :) + d/2 * direction, ub), lb);
            X_right = max(min(X(i, :) - d/2 * direction, ub), lb);
            
            f_left = fitFunc(X_left);
            f_right = fitFunc(X_right);
            
            X(i, :) = max(min(X(i, :) + delta(i) * direction * sign(f_right - f_left), ub), lb);
            delta(i) = params.bas_eta * delta(i);
            
            fitness(i) = fitFunc(X(i, :));
            if fitness(i) < bestFit
                bestFit = fitness(i);
                bestSol = X(i, :);
            end
        end
        convCurve(iter) = bestFit;
    end
end
