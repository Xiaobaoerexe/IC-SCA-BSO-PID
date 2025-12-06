%% Benchmark Functions for Algorithm Testing
% This script tests the optimization algorithms on standard benchmark functions
% to verify their performance before applying to PID optimization

clear; clc; close all;
fprintf('========================================\n');
fprintf('Benchmark Function Testing\n');
fprintf('========================================\n\n');

%% Parameters
params.popSize = 50;
params.maxIter = 100;
params.numRuns = 10;

% Algorithm-specific parameters
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

%% Test on Sphere Function (F1)
fprintf('Testing on Sphere Function (F1):\n');
params.dim = 30;
params.lb = -100 * ones(1, params.dim);
params.ub = 100 * ones(1, params.dim);

sphereFunc = @(x) sum(x.^2);
test_algorithms(sphereFunc, params, 'Sphere (F1)');

%% Test on Rosenbrock Function (F2)
fprintf('\nTesting on Rosenbrock Function (F2):\n');
params.dim = 30;
params.lb = -30 * ones(1, params.dim);
params.ub = 30 * ones(1, params.dim);

rosenbrockFunc = @(x) sum(100*(x(2:end) - x(1:end-1).^2).^2 + (x(1:end-1) - 1).^2);
test_algorithms(rosenbrockFunc, params, 'Rosenbrock (F2)');

%% Test on Rastrigin Function (F3)
fprintf('\nTesting on Rastrigin Function (F3):\n');
params.dim = 30;
params.lb = -5.12 * ones(1, params.dim);
params.ub = 5.12 * ones(1, params.dim);

rastriginFunc = @(x) 10*length(x) + sum(x.^2 - 10*cos(2*pi*x));
test_algorithms(rastriginFunc, params, 'Rastrigin (F3)');

%% Test on Ackley Function (F4)
fprintf('\nTesting on Ackley Function (F4):\n');
params.dim = 30;
params.lb = -32 * ones(1, params.dim);
params.ub = 32 * ones(1, params.dim);

ackleyFunc = @(x) -20*exp(-0.2*sqrt(sum(x.^2)/length(x))) - ...
                  exp(sum(cos(2*pi*x))/length(x)) + 20 + exp(1);
test_algorithms(ackleyFunc, params, 'Ackley (F4)');

%% Test on Griewank Function (F5)
fprintf('\nTesting on Griewank Function (F5):\n');
params.dim = 30;
params.lb = -600 * ones(1, params.dim);
params.ub = 600 * ones(1, params.dim);

griewankFunc = @(x) sum(x.^2)/4000 - prod(cos(x./sqrt(1:length(x)))) + 1;
test_algorithms(griewankFunc, params, 'Griewank (F5)');

fprintf('\n========================================\n');
fprintf('Benchmark testing completed!\n');
fprintf('========================================\n');

%% Helper Function
function test_algorithms(fitFunc, params, funcName)
    algorithms = {'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
    numAlgo = length(algorithms);
    
    results = zeros(params.numRuns, numAlgo);
    
    for algoIdx = 1:numAlgo
        algoName = algorithms{algoIdx};
        
        for run = 1:params.numRuns
            rng(run);
            
            switch algoName
                case 'BAS'
                    [~, bestFit, ~] = run_BAS_bench(fitFunc, params);
                case 'PSO'
                    [~, bestFit, ~] = run_PSO_bench(fitFunc, params);
                case 'BSO'
                    [~, bestFit, ~] = run_BSO_bench(fitFunc, params);
                case 'IC_SCA_BSO'
                    [~, bestFit, ~] = run_IC_SCA_BSO_bench(fitFunc, params);
            end
            
            results(run, algoIdx) = bestFit;
        end
    end
    
    fprintf('\n%s Results:\n', funcName);
    fprintf('%-15s %15s %15s %15s\n', 'Algorithm', 'Mean', 'Std', 'Best');
    fprintf('-------------------------------------------------------------\n');
    
    for algoIdx = 1:numAlgo
        fprintf('%-15s %15.4e %15.4e %15.4e\n', ...
            strrep(algorithms{algoIdx}, '_', '-'), ...
            mean(results(:, algoIdx)), ...
            std(results(:, algoIdx)), ...
            min(results(:, algoIdx)));
    end
end

%% IC-SCA-BSO for Benchmarks
function [bestSol, bestFit, convCurve] = run_IC_SCA_BSO_bench(fitFunc, params)
    popSize = params.popSize;
    maxIter = params.maxIter;
    dim = params.dim;
    lb = params.lb;
    ub = params.ub;
    
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

%% BSO for Benchmarks
function [bestSol, bestFit, convCurve] = run_BSO_bench(fitFunc, params)
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

%% PSO for Benchmarks
function [bestSol, bestFit, convCurve] = run_PSO_bench(fitFunc, params)
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

%% BAS for Benchmarks
function [bestSol, bestFit, convCurve] = run_BAS_bench(fitFunc, params)
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
