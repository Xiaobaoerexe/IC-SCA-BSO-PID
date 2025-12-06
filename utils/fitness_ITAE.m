function ITAE = fitness_ITAE(pidParams, params)
%% ITAE Fitness Function for PID Optimization
% Calculates the Integral of Time-weighted Absolute Error (ITAE)
% as the fitness function for optimization algorithms
%
% Inputs:
%   pidParams - [Kp, Ki, Kd] PID parameters
%   params    - Simulation parameters
%
% Output:
%   ITAE      - Fitness value (lower is better)

%% Extract PID parameters
Kp = pidParams(1);
Ki = pidParams(2);
Kd = pidParams(3);

%% Simulation parameters
simTime = params.simTime;
dt = params.dt;
baseSpeed = params.baseSpeed;

% Default speed command for optimization
speedCommand = baseSpeed + 40;  % 8300 + 40 = 8340 r/min

%% Run simulation
[time, speed, ~] = gas_turbine_sim_simple(Kp, Ki, Kd, speedCommand, simTime, dt, baseSpeed);

%% Calculate ITAE (Equation 26)
error = speedCommand - speed;
ITAE = sum(time .* abs(error)) * dt;

%% Add penalty terms for constraint violations

% 1. Overshoot constraint: sigma <= 5%
overshoot = max(0, max(speed) - speedCommand) / speedCommand * 100;
if overshoot > 5
    ITAE = ITAE + 500 * (overshoot - 5)^2;
end

% 2. Settling time constraint: ts <= 10s
% Find settling time (2% criterion)
tolerance = 0.02 * speedCommand;
settlingIdx = find(abs(error(end-round(1/dt):end)) > tolerance, 1, 'last');
if ~isempty(settlingIdx)
    settlingTime = simTime;  % Did not settle
    ITAE = ITAE + 100 * (settlingTime - 10);
else
    % Find when it first stays within tolerance
    withinTolerance = abs(error) < tolerance;
    stayedWithin = true(size(withinTolerance));
    for i = length(withinTolerance):-1:1
        if i < length(withinTolerance)
            stayedWithin(i) = withinTolerance(i) && stayedWithin(i+1);
        else
            stayedWithin(i) = withinTolerance(i);
        end
    end
    settlingIdx = find(stayedWithin, 1, 'first');
    if ~isempty(settlingIdx)
        settlingTime = time(settlingIdx);
        if settlingTime > 10
            ITAE = ITAE + 100 * (settlingTime - 10);
        end
    end
end

% 3. Stability constraint: penalize oscillations
if sum(diff(sign(diff(speed))) ~= 0) > 10  % Too many oscillations
    ITAE = ITAE + 200;
end

% Ensure ITAE is positive
ITAE = max(ITAE, 0);

end

function [time, speed, fuelFlow] = gas_turbine_sim_simple(Kp, Ki, Kd, speedCommand, simTime, dt, baseSpeed)
%% Simplified Gas Turbine Simulation
% Fast simulation for optimization iterations

%% Initialize
numSteps = round(simTime / dt) + 1;
time = (0:numSteps-1)' * dt;
speed = zeros(numSteps, 1);
fuelFlow = zeros(numSteps, 1);

% Gas turbine model parameters
tau1 = 0.5;     % Fuel system time constant
tau2 = 1.5;     % Turbine time constant
K_gt = 100;     % Gas turbine gain

% Fuel flow constraints
qf_max = 2.5;
qf_min = 0.1;
dqf_max = 0.5;

% State variables
x1 = 0;
x2 = 0;

% Initial conditions
speed(1) = baseSpeed;
qf_prev = 0.8;

% PID controller state
integral_error = 0;
prev_error = 0;

%% Simulation loop
for k = 2:numSteps
    currentSpeed = speed(k-1);
    error = speedCommand - currentSpeed;
    
    % PID control
    P = Kp * error;
    integral_error = integral_error + error * dt;
    integral_error = max(-100, min(100, integral_error));
    I = Ki * integral_error;
    D = Kd * (error - prev_error) / dt;
    prev_error = error;
    
    u = P + I + D;
    qf = 0.8 + u * 0.1;
    
    % Rate limiter
    dqf = (qf - qf_prev) / dt;
    if abs(dqf) > dqf_max
        qf = qf_prev + sign(dqf) * dqf_max * dt;
    end
    qf = max(qf_min, min(qf_max, qf));
    
    fuelFlow(k) = qf;
    qf_prev = qf;
    
    % Gas turbine dynamics
    dx1 = (-x1 + qf) / tau1;
    x1 = x1 + dx1 * dt;
    
    dx2 = (-x2 + K_gt * x1) / tau2;
    x2 = x2 + dx2 * dt;
    
    speed(k) = baseSpeed + x2;
end

end
