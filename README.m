%% Gas Turbine Speed Controller Optimization Based on IC-SCA-BSO Algorithm
% ==========================================================================
% MATLAB Implementation of the Paper:
% "Improved Circle-SCA-BSO optimized Gas Turbine Speed PID Controller 
%  for enhanced Speed Tracking and Interference Rejection"
%
% This package contains all the code to reproduce the experiments from the paper.
%
% ==========================================================================
% DIRECTORY STRUCTURE:
% ==========================================================================
%
% matlab_code/
% |
% |-- main_experiment.m          % Main script to run all experiments
% |-- standalone_demo.m          % Self-contained demo (no dependencies)
% |-- test_benchmark_functions.m % Test algorithms on benchmark functions
% |
% |-- algorithms/                % Optimization algorithms
% |   |-- IC_SCA_BSO.m          % Improved Circle-SCA-BSO (proposed)
% |   |-- BSO.m                 % Standard Beetle Swarm Optimization
% |   |-- PSO.m                 % Particle Swarm Optimization
% |   |-- BAS.m                 % Beetle Antennae Search
% |
% |-- models/                    % Gas turbine model
% |   |-- gas_turbine_narx_model.m    % NARX neural network model
% |   |-- gas_turbine_simulation.m    % Control system simulation
% |
% |-- utils/                     % Utility functions
% |   |-- fitness_ITAE.m        % ITAE fitness function
% |   |-- plot_convergence_curves.m
% |   |-- plot_tracking_results.m
% |   |-- plot_interference_results.m
% |   |-- plot_weight_comparison.m
% |   |-- display_summary_results.m
% |
% |-- experiments/               % Experiment scripts
% |   |-- run_convergence_experiment.m
% |   |-- run_tracking_experiments.m
% |   |-- run_interference_experiment.m
% |
% |-- results/                   % Directory for saving results
%
% ==========================================================================
% QUICK START:
% ==========================================================================
%
% Option 1: Run the standalone demo (recommended for first use)
%   >> standalone_demo
%   
%   This runs a complete demonstration including:
%   - Algorithm convergence comparison
%   - PID parameter optimization
%   - Tracking tests at 8340, 8380, 8420 r/min
%   - Anti-interference test with 5% fuel disturbance
%   - All figures from the paper
%
% Option 2: Run the main experiment script
%   >> main_experiment
%   
%   This uses the modular code structure and saves results to files.
%
% Option 3: Test algorithms on benchmark functions
%   >> test_benchmark_functions
%   
%   This tests the algorithms on standard benchmark functions (Sphere,
%   Rosenbrock, Rastrigin, Ackley, Griewank) to verify their performance.
%
% ==========================================================================
% ALGORITHM DESCRIPTIONS:
% ==========================================================================
%
% 1. IC-SCA-BSO (Improved Circle-SCA-BSO) - Proposed Algorithm
%    Three key improvements over standard BSO:
%    (a) Circle mapping for population initialization
%        x(i+1) = x(i) + a - mod((b/2*pi)*sin(2*pi*x(i)), 1)
%        where a=0.2, b=0.5
%    
%    (b) Nonlinear decreasing inertia weight (Equation 22)
%        omega = 0.5 * [(1 - log(1 + (e-1)*k/kmax)) + exp(-k/a)]
%        where a=5
%    
%    (c) SCA-based learning factors with sine/cosine switching
%        - Probability p=0.5 switches between sine and cosine
%        - Nonlinear decreasing coefficient r1 (Equation 25)
%        r1 = a - (a-b) * log(1 + (e-1)*k/kmax), where a=2, b=0
%
% 2. BSO (Beetle Swarm Optimization)
%    - Standard parameters: omega=0.7, c1=c2=1.8, eta=0.9
%
% 3. PSO (Particle Swarm Optimization)
%    - Linear decreasing inertia weight: 0.9 to 0.4
%    - Learning factors: c1=c2=2.0
%
% 4. BAS (Beetle Antennae Search)
%    - Single-solution optimizer with population for fair comparison
%    - Initial step size: 0.5, Attenuation: 0.95
%
% ==========================================================================
% EXPERIMENT SETTINGS (matching the paper):
% ==========================================================================
%
% - Population size: 100
% - Maximum iterations: 100
% - Number of independent runs: 20
% - PID parameter bounds:
%   Kp: [0, 2]
%   Ki: [0, 1.5]
%   Kd: [0, 1]
%
% - Base speed: 8300 r/min
% - Tracking test commands: 8340, 8380, 8420 r/min
% - Disturbance level: 5% fuel flow increase at t=4s
%
% - Fitness function: ITAE = integral(t * |e(t)|) dt
% - Constraints:
%   - Overshoot <= 5%
%   - Settling time <= 10s
%
% ==========================================================================
% EXPECTED OUTPUTS:
% ==========================================================================
%
% 1. Figure 5: Weight comparison (omega1 vs omega2)
% 2. Figure 8: Convergence curves comparison
% 3. Figures 9-11: Tracking test results
% 4. Figure 12: Anti-interference test results
%
% Tables:
% - Table 4: Controller parameters (Kp, Ki, Kd)
% - Table 5: Performance at 8340 r/min
% - Table 6: Performance at 8380 r/min
% - Table 7: Performance at 8420 r/min
% - Table 8: Anti-interference test performance
%
% ==========================================================================
% REFERENCE:
% ==========================================================================
%
% Dong Y., Liu X., Wang Z., Zhang L., Zhang X., "Improved Circle-SCA-BSO 
% optimized Gas Turbine Speed PID Controller for enhanced Speed Tracking 
% and Interference Rejection"
%
% ==========================================================================
