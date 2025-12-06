function interferenceResults = run_interference_experiment(bestParams, disturbanceLevel, params)
%% Run Anti-interference Test
% Tests controller performance under fuel flow disturbance
%
% Inputs:
%   bestParams      - Structure containing best PID parameters
%   disturbanceLevel- Magnitude of disturbance (fraction, e.g., 0.05 for 5%)
%   params          - Simulation parameters
%
% Outputs:
%   interferenceResults - Structure containing test results

fprintf('\nAnti-interference Test: %.0f%% fuel disturbance\n', disturbanceLevel * 100);
fprintf('========================================\n');

controllers = {'PID', 'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
numControllers = length(controllers);

% Disturbance parameters
disturbance.time = 4.0;     % Apply disturbance at t=4s
disturbance.level = disturbanceLevel;

% Initialize result structure
interferenceResults = struct();
interferenceResults.disturbanceLevel = disturbanceLevel;
interferenceResults.disturbanceTime = disturbance.time;
interferenceResults.time = cell(numControllers, 1);
interferenceResults.speed = cell(numControllers, 1);
interferenceResults.fuelFlow = cell(numControllers, 1);
interferenceResults.metrics = struct();

for ctrlIdx = 1:numControllers
    ctrlName = controllers{ctrlIdx};
    
    % Get PID parameters
    pidParams = bestParams.(ctrlName);
    Kp = pidParams.Kp;
    Ki = pidParams.Ki;
    Kd = pidParams.Kd;
    
    % Run simulation with disturbance
    [time, speed, fuelFlow] = gas_turbine_sim_disturbance(Kp, Ki, Kd, ...
        params.baseSpeed, params.simTime, params.dt, disturbance);
    
    interferenceResults.time{ctrlIdx} = time;
    interferenceResults.speed{ctrlIdx} = speed;
    interferenceResults.fuelFlow{ctrlIdx} = fuelFlow;
    
    % Calculate performance metrics
    metrics = calculate_interference_metrics(time, speed, params.baseSpeed, ...
        disturbance.time, params.dt);
    
    interferenceResults.metrics.(ctrlName) = metrics;
    
    fprintf('%-15s: ts=%.3f s, sigma=%.5f\n', ...
        [ctrlName '-PID'], metrics.settlingTime, metrics.overshoot);
end

interferenceResults.controllers = controllers;

% Print comparison table
print_interference_table(interferenceResults, controllers);

end

function [time, speed, fuelFlow] = gas_turbine_sim_disturbance(Kp, Ki, Kd, baseSpeed, simTime, dt, disturbance)
%% Gas Turbine Simulation with Disturbance
% Simulates steady-state operation with fuel flow disturbance

numSteps = round(simTime / dt) + 1;
time = (0:numSteps-1)' * dt;
speed = zeros(numSteps, 1);
fuelFlow = zeros(numSteps, 1);

% Gas turbine model parameters
tau1 = 0.5;
tau2 = 1.5;
K_gt = 100;

% Fuel constraints
qf_max = 2.5;
qf_min = 0.1;
dqf_max = 0.5;

% Initialize at steady state
x1_ss = 0.8 * tau1;  % Steady state for x1
x2_ss = 0;           % Speed deviation = 0 at base speed

x1 = x1_ss;
x2 = x2_ss;

% Initial conditions
speed(1) = baseSpeed;
qf_prev = 0.8;

% PID controller state
integral_error = 0;
prev_error = 0;

% Simulation loop
for k = 2:numSteps
    t = time(k);
    currentSpeed = speed(k-1);
    error = baseSpeed - currentSpeed;
    
    % PID control
    P = Kp * error;
    integral_error = integral_error + error * dt;
    integral_error = max(-50, min(50, integral_error));
    I = Ki * integral_error;
    
    if k > 2
        D = Kd * (error - prev_error) / dt;
    else
        D = 0;
    end
    prev_error = error;
    
    u = P + I + D;
    qf = 0.8 + u * 0.1;
    
    % Rate limiter
    dqf = (qf - qf_prev) / dt;
    if abs(dqf) > dqf_max
        qf = qf_prev + sign(dqf) * dqf_max * dt;
    end
    qf = max(qf_min, min(qf_max, qf));
    
    % Apply disturbance after specified time
    if t >= disturbance.time
        qf_disturbed = qf * (1 + disturbance.level);
    else
        qf_disturbed = qf;
    end
    
    fuelFlow(k) = qf;
    qf_prev = qf;
    
    % Gas turbine dynamics with disturbed fuel
    dx1 = (-x1 + qf_disturbed) / tau1;
    x1 = x1 + dx1 * dt;
    
    dx2 = (-x2 + K_gt * x1) / tau2;
    x2 = x2 + dx2 * dt;
    
    speed(k) = baseSpeed + x2;
end

end

function metrics = calculate_interference_metrics(time, speed, targetSpeed, disturbanceTime, dt)
%% Calculate Anti-interference Performance Metrics

% Find index where disturbance starts
disturbIdx = find(time >= disturbanceTime, 1, 'first');

% Only analyze after disturbance
time_post = time(disturbIdx:end) - disturbanceTime;
speed_post = speed(disturbIdx:end);

% Calculate overshoot (maximum deviation from target)
deviation = abs(speed_post - targetSpeed);
overshoot = max(deviation) / targetSpeed;

% Settling time (time to return within 2% of target after disturbance)
tolerance = 0.02 * targetSpeed;
error = abs(speed_post - targetSpeed);

% Find when it settles back
settledIdx = find(error < tolerance, 1, 'first');
if ~isempty(settledIdx)
    % Check if it stays within tolerance
    if settledIdx < length(error)
        remainingError = error(settledIdx:end);
        if all(remainingError < tolerance)
            settlingTime = time_post(settledIdx) + disturbanceTime;
        else
            lastOutIdx = find(error > tolerance, 1, 'last');
            if ~isempty(lastOutIdx)
                settlingTime = time_post(lastOutIdx) + disturbanceTime;
            else
                settlingTime = time(end);
            end
        end
    else
        settlingTime = time_post(settledIdx) + disturbanceTime;
    end
else
    settlingTime = time(end);
end

metrics.settlingTime = settlingTime;
metrics.overshoot = overshoot;
metrics.maxDeviation = max(deviation);

end

function print_interference_table(results, controllers)
%% Print Anti-interference Test Results Table

fprintf('\n%-20s %15s %15s\n', 'Controller', 'ts/s', 'sigma_p');
fprintf('--------------------------------------------------\n');

for i = 1:length(controllers)
    ctrlName = controllers{i};
    m = results.metrics.(ctrlName);
    fprintf('%-20s %15.3f %15.5f\n', ...
        [ctrlName '-PID'], m.settlingTime, m.overshoot);
end
fprintf('--------------------------------------------------\n');

end
