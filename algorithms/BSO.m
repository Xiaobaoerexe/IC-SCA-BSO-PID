function [bestSolution, bestFitness, convergenceCurve] = BSO(fitFunc, params)
%% BSO: Beetle Swarm Optimization Algorithm
% Standard BSO implementation based on the paper
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

% BSO specific parameters
omega = params.bso_omega;           % Inertia weight = 0.7
c1 = params.bso_c1;                 % Individual learning factor = 1.8
c2 = params.bso_c2;                 % Global learning factor = 1.8
eta = params.bso_eta;               % Step-size attenuation = 0.9
lambda = 0.5;                       % Step-size proportion factor

%% Initialize population randomly
X = zeros(popSize, dim);
for i = 1:popSize
    X(i, :) = lb + rand(1, dim) .* (ub - lb);
end

%% Initialize velocity
V = zeros(popSize, dim);

%% Initialize step size
delta = ones(popSize, 1) * 0.5;     % Initial step size

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
    for i = 1:popSize
        % Random values
        r1 = rand();
        r2 = rand();
        
        % Update velocity (Equation 12)
        V(i, :) = omega * V(i, :) + ...
                  c1 * r1 * (pbest(i, :) - X(i, :)) + ...
                  c2 * r2 * (gbest - X(i, :));
        
        % Calculate whisker positions (Equation 15)
        d = delta(i) / c2;          % Distance between whiskers (Equation 16)
        direction = V(i, :) / (norm(V(i, :)) + eps);
        
        X_left = X(i, :) + direction * d/2;
        X_right = X(i, :) - direction * d/2;
        
        % Bound checking for whiskers
        X_left = max(min(X_left, ub), lb);
        X_right = max(min(X_right, ub), lb);
        
        % Calculate fitness for whiskers
        f_left = fitFunc(X_left);
        f_right = fitFunc(X_right);
        
        % Calculate position increment (Equation 13)
        xi = delta(i) * direction * sign(f_left - f_right);
        
        % Update position (Equation 11)
        X(i, :) = X(i, :) + lambda * V(i, :) + (1 - lambda) * xi;
        
        % Bound checking
        X(i, :) = max(min(X(i, :), ub), lb);
        
        % Update step size (Equation 14)
        delta(i) = eta * delta(i);
        
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
