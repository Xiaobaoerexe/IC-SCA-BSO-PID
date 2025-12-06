function [time, speed, fuelFlow] = gas_turbine_simulation(Kp, Ki, Kd, speedCommand, params, disturbance)
%% Gas Turbine Speed Control System Simulation
% Simulates the closed-loop gas turbine speed control system
% with PID controller
%
% Inputs:
%   Kp          - Proportional gain
%   Ki          - Integral gain
%   Kd          - Derivative gain
%   speedCommand- Target speed (r/min)
%   params      - Simulation parameters
%   disturbance - Disturbance structure (optional)
%                 disturbance.time - time to apply disturbance
%                 disturbance.level - magnitude of disturbance (fraction)
%
% Outputs:
%   time        - Time vector
%   speed       - Speed response (r/min)
%   fuelFlow    - Fuel flow command

if nargin < 6
    disturbance = [];
end

%% Extract parameters
simTime = params.simTime;
dt = params.dt;
baseSpeed = params.baseSpeed;

%% System parameters (based on typical gas turbine characteristics)
% Time constants and gains for gas turbine speed dynamics
tau1 = 0.5;     % Fuel system time constant (s)
tau2 = 1.5;     % Combustor/turbine time constant (s)
K_gt = 100;     % Gas turbine gain

% Maximum fuel flow rate and constraints
qf_max = 2.5;   % Maximum fuel flow (kg/s)
qf_min = 0.1;   % Minimum fuel flow (kg/s)
dqf_max = 0.5;  % Maximum fuel rate of change (kg/s^2)

%% Initialize simulation
numSteps = round(simTime / dt) + 1;
time = (0:numSteps-1)' * dt;
speed = zeros(numSteps, 1);
fuelFlow = zeros(numSteps, 1);

% State variables for gas turbine model (2nd order system)
x1 = 0;         % Fuel system state
x2 = 0;         % Turbine state

% Initial conditions
speed(1) = baseSpeed;
qf_prev = 0.8;  % Initial fuel flow

% PID controller state
integral_error = 0;
prev_error = 0;

%% Main simulation loop
for k = 2:numSteps
    t = time(k);
    
    % Current speed (from previous step)
    currentSpeed = speed(k-1);
    
    % Error calculation
    error = speedCommand - currentSpeed;
    
    % PID control law
    % Proportional term
    P = Kp * error;
    
    % Integral term (with anti-windup)
    integral_error = integral_error + error * dt;
    integral_error = max(-100, min(100, integral_error));  % Anti-windup
    I = Ki * integral_error;
    
    % Derivative term (with filtering)
    D = Kd * (error - prev_error) / dt;
    prev_error = error;
    
    % Controller output
    u = P + I + D;
    
    % Map to fuel flow (normalized around operating point)
    qf = 0.8 + u * 0.1;  % Base fuel flow + control adjustment
    
    % Apply rate limiter
    dqf = (qf - qf_prev) / dt;
    if abs(dqf) > dqf_max
        qf = qf_prev + sign(dqf) * dqf_max * dt;
    end
    
    % Apply fuel flow limits
    qf = max(qf_min, min(qf_max, qf));
    
    % Apply disturbance if specified
    if ~isempty(disturbance) && t >= disturbance.time
        qf = qf * (1 + disturbance.level);
    end
    
    fuelFlow(k) = qf;
    qf_prev = qf;
    
    % Gas turbine dynamics (2nd order model)
    % State space representation: dx/dt = A*x + B*u, y = C*x
    % Transfer function: G(s) = K_gt / ((tau1*s + 1)*(tau2*s + 1))
    
    % Update state x1 (fuel system dynamics)
    dx1 = (-x1 + qf) / tau1;
    x1 = x1 + dx1 * dt;
    
    % Update state x2 (turbine dynamics)
    dx2 = (-x2 + K_gt * x1) / tau2;
    x2 = x2 + dx2 * dt;
    
    % Output: speed deviation from base
    speedDeviation = x2;
    
    % Add some nonlinearity (characteristic of gas turbines)
    nonlinearFactor = 1 + 0.1 * sin(speedDeviation / 100);
    speedDeviation = speedDeviation * nonlinearFactor;
    
    % Calculate actual speed
    speed(k) = baseSpeed + speedDeviation;
    
    % Add small process noise
    speed(k) = speed(k) + randn() * 0.5;
end

end

function ITAE = calculate_ITAE(Kp, Ki, Kd, speedCommand, params)
%% Calculate ITAE (Integral of Time-weighted Absolute Error)
% Fitness function for optimization algorithms
%
% Inputs:
%   Kp, Ki, Kd  - PID parameters
%   speedCommand- Target speed
%   params      - Simulation parameters
%
% Output:
%   ITAE        - Integral of time-weighted absolute error

[time, speed, ~] = gas_turbine_simulation(Kp, Ki, Kd, speedCommand, params);

% Calculate error
error = speedCommand - speed;

% Calculate ITAE (Equation 26)
dt = params.dt;
ITAE = sum(time .* abs(error)) * dt;

% Add penalty for excessive overshoot
overshoot = max(0, max(speed) - speedCommand) / speedCommand * 100;
if overshoot > 5  % Overshoot constraint: <= 5%
    ITAE = ITAE + 1000 * (overshoot - 5);
end

% Add penalty for excessive settling time
settlingIdx = find(abs(error) < 0.02 * speedCommand, 1, 'first');
if ~isempty(settlingIdx)
    settlingTime = time(settlingIdx);
    if settlingTime > 10  % Settling time constraint: <= 10s
        ITAE = ITAE + 100 * (settlingTime - 10);
    end
end

end
