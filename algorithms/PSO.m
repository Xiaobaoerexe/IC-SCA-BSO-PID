function [bestSolution, bestFitness, convergenceCurve] = PSO(fitFunc, params)
%% PSO: Particle Swarm Optimization Algorithm
% Standard PSO implementation with linearly decreasing inertia weight
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
popSize = params.popSize;
maxIter = params.maxIter;
dim = params.dim;
lb = params.lb;
ub = params.ub;

% PSO specific parameters
omega_max = params.pso_omega_max;   % Initial inertia weight = 0.9
omega_min = params.pso_omega_min;   % Final inertia weight = 0.4
c1 = params.pso_c1;                 % Cognitive coefficient = 2.0
c2 = params.pso_c2;                 % Social coefficient = 2.0

% Velocity limits
Vmax = 0.2 * (ub - lb);
Vmin = -Vmax;

%% Initialize population randomly
X = zeros(popSize, dim);
for i = 1:popSize
    X(i, :) = lb + rand(1, dim) .* (ub - lb);
end

%% Initialize velocity randomly
V = zeros(popSize, dim);
for i = 1:popSize
    V(i, :) = Vmin + rand(1, dim) .* (Vmax - Vmin);
end

%% Evaluate initial population
fitness = zeros(popSize, 1);
for i = 1:popSize
    fitness(i) = fitFunc(X(i, :));
end

%% Initialize personal best and global best
pbest = X;                          % Personal best positions
pbestFitness = fitness;             % Personal best fitness values

[bestFitness, bestIdx] = min(fitness);
gbest = X(bestIdx, :);              % Global best position

%% Initialize convergence curve
convergenceCurve = zeros(maxIter, 1);

%% Main loop
for iter = 1:maxIter
    % Linearly decreasing inertia weight
    omega = omega_max - (omega_max - omega_min) * iter / maxIter;
    
    for i = 1:popSize
        % Random values
        r1 = rand(1, dim);
        r2 = rand(1, dim);
        
        % Update velocity (Equation 17)
        V(i, :) = omega * V(i, :) + ...
                  c1 * r1 .* (pbest(i, :) - X(i, :)) + ...
                  c2 * r2 .* (gbest - X(i, :));
        
        % Velocity clamping
        V(i, :) = max(min(V(i, :), Vmax), Vmin);
        
        % Update position (Equation 18)
        X(i, :) = X(i, :) + V(i, :);
        
        % Bound checking
        X(i, :) = max(min(X(i, :), ub), lb);
        
        % Evaluate fitness
        fitness(i) = fitFunc(X(i, :));
        
        % Update personal best
        if fitness(i) < pbestFitness(i)
            pbest(i, :) = X(i, :);
            pbestFitness(i) = fitness(i);
        end
    end
    
    % Update global best
    [minFitness, minIdx] = min(pbestFitness);
    if minFitness < bestFitness
        bestFitness = minFitness;
        gbest = pbest(minIdx, :);
    end
    
    % Record convergence
    convergenceCurve(iter) = bestFitness;
end

bestSolution = gbest;

end
