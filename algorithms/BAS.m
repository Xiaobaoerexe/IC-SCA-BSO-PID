function [bestSolution, bestFitness, convergenceCurve] = BAS(fitFunc, params)
%% BAS: Beetle Antennae Search Algorithm
% Single-solution optimizer that mimics beetle foraging behavior
%
% Inputs:
%   fitFunc - Fitness function handle
%   params  - Algorithm parameters
%
% Outputs:
%   bestSolution    - Best solution found
%   bestFitness     - Best fitness value
%   convergenceCurve - Convergence history

%% Extract parameters
popSize = params.popSize;           % Use multiple starting points for fair comparison
maxIter = params.maxIter;
dim = params.dim;
lb = params.lb;
ub = params.ub;

% BAS specific parameters
delta0 = params.bas_delta0;         % Initial step size = 0.5
eta = params.bas_eta;               % Step-size attenuation = 0.95

%% Initialize multiple beetles (for fair comparison with population-based algorithms)
X = zeros(popSize, dim);
for i = 1:popSize
    X(i, :) = lb + rand(1, dim) .* (ub - lb);
end

%% Initialize step sizes
delta = ones(popSize, 1) * delta0;

%% Evaluate initial population
fitness = zeros(popSize, 1);
for i = 1:popSize
    fitness(i) = fitFunc(X(i, :));
end

%% Find initial best
[bestFitness, bestIdx] = min(fitness);
bestSolution = X(bestIdx, :);

%% Initialize convergence curve
convergenceCurve = zeros(maxIter, 1);

%% Main loop
for iter = 1:maxIter
    for i = 1:popSize
        % Generate random direction (normalized)
        direction = randn(1, dim);
        direction = direction / (norm(direction) + eps);
        
        % Calculate antennae distance
        d = delta(i);
        
        % Calculate antennae positions (Equation 20)
        X_left = X(i, :) + d/2 * direction;
        X_right = X(i, :) - d/2 * direction;
        
        % Bound checking for antennae
        X_left = max(min(X_left, ub), lb);
        X_right = max(min(X_right, ub), lb);
        
        % Calculate fitness for antennae
        f_left = fitFunc(X_left);
        f_right = fitFunc(X_right);
        
        % Update position (Equation 19)
        X(i, :) = X(i, :) + delta(i) * direction * sign(f_right - f_left);
        
        % Bound checking
        X(i, :) = max(min(X(i, :), ub), lb);
        
        % Update step size
        delta(i) = eta * delta(i);
        
        % Evaluate fitness
        fitness(i) = fitFunc(X(i, :));
        
        % Update global best
        if fitness(i) < bestFitness
            bestFitness = fitness(i);
            bestSolution = X(i, :);
        end
    end
    
    % Record convergence
    convergenceCurve(iter) = bestFitness;
end

end
