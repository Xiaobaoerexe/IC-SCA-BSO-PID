function [bestSolution, bestFitness, convergenceCurve] = IC_SCA_BSO(fitFunc, params)
%% IC-SCA-BSO: Improved Circle-SCA-BSO Algorithm
% Improvements:
%   1. Circle mapping for population initialization
%   2. Nonlinear decreasing inertia weight
%   3. SCA-based learning factors with sine/cosine switching
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

% IC-SCA-BSO specific parameters
circleA = params.circleA;           % Circle mapping a = 0.2
circleB = params.circleB;           % Circle mapping b = 0.5
weightA = params.weightA;           % Weight parameter a = 5
scaA = params.scaA;                 % SCA parameter a = 2
scaB = params.scaB;                 % SCA parameter b = 0
switchProb = params.switchProb;     % Switching probability p = 0.5
lambda = 0.5;                       % Step-size proportion factor
eta = params.bso_eta;               % Step-size attenuation coefficient

%% Initialize population using Circle Mapping
% Circle mapping formula: x(i+1) = x(i) + a - mod((b/2*pi)*sin(2*pi*x(i)), 1)
X = circle_mapping_initialization(popSize, dim, lb, ub, circleA, circleB);

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
    % Calculate nonlinear decreasing inertia weight (Equation 22)
    % omega = 0.5 * [(1 - log(1 + (e-1)*k/kmax)) + exp(-k/a)]
    omega = 0.5 * ((1 - log(1 + (exp(1)-1) * iter/maxIter)) + exp(-iter/weightA));
    
    % Calculate nonlinear decreasing coefficient r1 (Equation 25)
    % r1 = a - (a-b) * log(1 + (e-1)*k/kmax)
    r1 = scaA - (scaA - scaB) * log(1 + (exp(1)-1) * iter/maxIter);
    
    for i = 1:popSize
        % Random values
        r2 = 2 * pi * rand();       % Random in [0, 2*pi]
        p = rand();                 % Switching probability
        
        % Update velocity using SCA-based learning factors
        if p < switchProb
            % Use sine function (Equation 23)
            V(i, :) = omega * V(i, :) + ...
                      r1 * sin(r2) * (pbest(i, :) - X(i, :)) + ...
                      r1 * sin(r2) * (gbest - X(i, :));
        else
            % Use cosine function (Equation 24)
            V(i, :) = omega * V(i, :) + ...
                      r1 * cos(r2) * (pbest(i, :) - X(i, :)) + ...
                      r1 * cos(r2) * (gbest - X(i, :));
        end
        
        % Calculate whisker positions (Equation 15)
        d = delta(i) / 1.8;         % Distance between whiskers
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

%% Circle Mapping Initialization Function
function X = circle_mapping_initialization(popSize, dim, lb, ub, a, b)
% Initialize population using circle mapping
% Formula: x(i+1) = x(i) + a - mod((b/2*pi)*sin(2*pi*x(i)), 1)

X = zeros(popSize, dim);
chaos = rand(1, dim);  % Initial random values in [0, 1]

for i = 1:popSize
    % Apply circle mapping
    chaos = chaos + a - mod((b/(2*pi)) * sin(2*pi*chaos), 1);
    chaos = mod(chaos, 1);  % Keep in [0, 1]
    
    % Map to solution space
    X(i, :) = lb + chaos .* (ub - lb);
end

end
