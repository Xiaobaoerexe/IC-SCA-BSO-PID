function trackingResults = run_tracking_experiments(bestParams, speedCommands, params)
%% Run Tracking Tests
% Conducts tracking tests at different speed commands for all controllers
%
% Inputs:
%   bestParams    - Structure containing best PID parameters for each algorithm
%   speedCommands - Array of speed commands to test (r/min)
%   params        - Simulation parameters
%
% Outputs:
%   trackingResults - Cell array of results for each speed command

numTests = length(speedCommands);
controllers = {'PID', 'BAS', 'PSO', 'BSO', 'IC_SCA_BSO'};
numControllers = length(controllers);

trackingResults = cell(numTests, 1);

for testIdx = 1:numTests
    speedCommand = speedCommands(testIdx);
    fprintf('\nTracking Test %d: Speed command = %d r/min\n', testIdx, speedCommand);
    fprintf('========================================\n');
    
    % Initialize result structure for this test
    result = struct();
    result.speedCommand = speedCommand;
    result.time = cell(numControllers, 1);
    result.speed = cell(numControllers, 1);
    result.fuelFlow = cell(numControllers, 1);
    result.metrics = struct();
    
    for ctrlIdx = 1:numControllers
        ctrlName = controllers{ctrlIdx};
        
        % Get PID parameters
        pidParams = bestParams.(ctrlName);
        Kp = pidParams.Kp;
        Ki = pidParams.Ki;
        Kd = pidParams.Kd;
        
        % Run simulation
        [time, speed, fuelFlow] = gas_turbine_sim_tracking(Kp, Ki, Kd, ...
            speedCommand, params.simTime, params.dt, params.baseSpeed);
        
        result.time{ctrlIdx} = time;
        result.speed{ctrlIdx} = speed;
        result.fuelFlow{ctrlIdx} = fuelFlow;
        
        % Calculate performance metrics
        metrics = calculate_tracking_metrics(time, speed, speedCommand, params.dt);
        
        result.metrics.(ctrlName) = metrics;
        
        fprintf('%-15s: tr=%.3f s, ts=%.3f s, ess=%.4f, sigma=%.4f\n', ...
            [ctrlName '-PID'], metrics.riseTime, metrics.settlingTime, ...
            metrics.steadyStateError, metrics.overshoot);
    end
    
    trackingResults{testIdx} = result;
    
    % Print comparison table
    print_tracking_table(result, controllers);
end

end

function [time, speed, fuelFlow] = gas_turbine_sim_tracking(Kp, Ki, Kd, speedCommand, simTime, dt, baseSpeed)
%% Gas Turbine Simulation for Tracking Test
% Starts from baseSpeed and tracks to speedCommand

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

% State variables
x1 = 0;
x2 = 0;

% Initial conditions (steady state at base speed)
speed(1) = baseSpeed;
qf_prev = 0.8;

% PID controller state
integral_error = 0;
prev_error = 0;

% Simulation loop
for k = 2:numSteps
    currentSpeed = speed(k-1);
    error = speedCommand - currentSpeed;
    
    % PID control
    P = Kp * error;
    integral_error = integral_error + error * dt;
    integral_error = max(-100, min(100, integral_error));
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

function metrics = calculate_tracking_metrics(time, speed, speedCommand, dt)
%% Calculate Tracking Performance Metrics
% Rise time, settling time, steady-state error, overshoot

% Rise time (10% to 90% of final value)
speedChange = speedCommand - speed(1);
speed10 = speed(1) + 0.1 * speedChange;
speed90 = speed(1) + 0.9 * speedChange;

idx10 = find(speed >= speed10, 1, 'first');
idx90 = find(speed >= speed90, 1, 'first');

if ~isempty(idx10) && ~isempty(idx90)
    riseTime = time(idx90) - time(idx10);
else
    riseTime = Inf;
end

% Settling time (2% criterion)
tolerance = 0.02 * speedCommand;
error = abs(speed - speedCommand);
settledIdx = find(error < tolerance, 1, 'first');

if ~isempty(settledIdx)
    % Check if it stays within tolerance
    staysSettled = all(error(settledIdx:end) < tolerance);
    if staysSettled
        settlingTime = time(settledIdx);
    else
        % Find the last time it's outside tolerance
        lastOutsideIdx = find(error > tolerance, 1, 'last');
        if ~isempty(lastOutsideIdx) && lastOutsideIdx < length(time)
            settlingTime = time(lastOutsideIdx + 1);
        else
            settlingTime = time(end);
        end
    end
else
    settlingTime = time(end);
end

% Steady-state error variance (last 2 seconds)
numSamples = round(2 / dt);
if length(speed) > numSamples
    steadyStateError = var(speed(end-numSamples:end) - speedCommand);
else
    steadyStateError = var(speed - speedCommand);
end

% Overshoot
peakSpeed = max(speed);
if peakSpeed > speedCommand
    overshoot = (peakSpeed - speedCommand) / (speedCommand - speed(1));
else
    overshoot = 0;
end

% Store metrics
metrics.riseTime = riseTime;
metrics.settlingTime = settlingTime;
metrics.steadyStateError = steadyStateError;
metrics.overshoot = overshoot;

end

function print_tracking_table(result, controllers)
%% Print Tracking Test Results Table

fprintf('\n%-15s %10s %10s %12s %12s\n', 'Controller', 'tr/s', 'ts/s', 'ess', 'sigma_p');
fprintf('--------------------------------------------------------------\n');

for i = 1:length(controllers)
    ctrlName = controllers{i};
    m = result.metrics.(ctrlName);
    fprintf('%-15s %10.3f %10.3f %12.4f %12.4f\n', ...
        [ctrlName '-PID'], m.riseTime, m.settlingTime, ...
        m.steadyStateError, m.overshoot);
end
fprintf('--------------------------------------------------------------\n');

end
