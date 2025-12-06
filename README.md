```markdown
# Gas Turbine Speed Controller Optimization Based on IC-SCA-BSO Algorithm

MATLAB Implementation of the paper: "Improved Circle-SCA-BSO optimized Gas Turbine Speed PID Controller for enhanced Speed Tracking and Interference Rejection"

This package contains all the code to reproduce the experiments from the paper.

## 📁 Directory Structure

```
matlab_code/
├── main_experiment.m          # Main script to run all experiments
├── standalone_demo.m          # Self-contained demo (no dependencies)
├── test_benchmark_functions.m # Test algorithms on benchmark functions
│
├── algorithms/                # Optimization algorithms
│   ├── IC_SCA_BSO.m          # Improved Circle-SCA-BSO (proposed)
│   ├── BSO.m                 # Standard Beetle Swarm Optimization
│   ├── PSO.m                 # Particle Swarm Optimization
│   └── BAS.m                 # Beetle Antennae Search
│
├── models/                    # Gas turbine model
│   ├── gas_turbine_narx_model.m    # NARX neural network model
│   └── gas_turbine_simulation.m    # Control system simulation
│
├── utils/                     # Utility functions
│   ├── fitness_ITAE.m        # ITAE fitness function
│   ├── plot_convergence_curves.m
│   ├── plot_tracking_results.m
│   ├── plot_interference_results.m
│   ├── plot_weight_comparison.m
│   └── display_summary_results.m
│
├── experiments/               # Experiment scripts
│   ├── run_convergence_experiment.m
│   ├── run_tracking_experiments.m
│   └── run_interference_experiment.m
│
└── results/                   # Directory for saving results and experimental data
```

## 🚀 Quick Start

### Option 1: Run Standalone Demo (Recommended for First Use)
```matlab
standalone_demo
```
This runs a complete demonstration including:
- Algorithm convergence comparison
- PID parameter optimization
- Tracking tests at 8340, 8380, 8420 r/min
- Anti-interference test with 5% fuel disturbance
- All figures from the paper

### Option 2: Run Main Experiment Script
```matlab
main_experiment
```
This uses the modular code structure and saves results to files.

### Option 3: Test on Benchmark Functions
```matlab
test_benchmark_functions
```
This tests the algorithms on standard benchmark functions (Sphere, Rosenbrock, Rastrigin, Ackley, Griewank) to verify their performance.

> **Note**: All experimental results, data files, and generated figures will be saved in the `results/` directory.

## 🔧 Algorithm Descriptions

### 1. IC-SCA-BSO (Improved Circle-SCA-BSO) - Proposed Algorithm

Three key improvements over standard BSO:

**a) Circle mapping for population initialization**
```matlab
x(i+1) = x(i) + a - mod((b/2*pi)*sin(2*pi*x(i)), 1)
```
where `a=0.2`, `b=0.5`

**b) Nonlinear decreasing inertia weight (Equation 22)**
```matlab
omega = 0.5 * [(1 - log(1 + (e-1)*k/kmax)) + exp(-k/a)]
```
where `a=5`

**c) SCA-based learning factors with sine/cosine switching**
- Probability `p=0.5` switches between sine and cosine
- Nonlinear decreasing coefficient `r1` (Equation 25)
```matlab
r1 = a - (a-b) * log(1 + (e-1)*k/kmax), where a=2, b=0
```

### 2. BSO (Beetle Swarm Optimization)
- Standard parameters: `omega=0.7`, `c1=c2=1.8`, `eta=0.9`

### 3. PSO (Particle Swarm Optimization)
- Linear decreasing inertia weight: 0.9 to 0.4
- Learning factors: `c1=c2=2.0`

### 4. BAS (Beetle Antennae Search)
- Single-solution optimizer with population for fair comparison
- Initial step size: 0.5, Attenuation: 0.95

## ⚙️ Experiment Settings (Matching the Paper)

| Parameter | Value |
|-----------|-------|
| Population size | 100 |
| Maximum iterations | 100 |
| Independent runs | 20 |
| **PID bounds** | |
| Kp | [0, 2] |
| Ki | [0, 1.5] |
| Kd | [0, 1] |
| **Test Conditions** | |
| Base speed | 8300 r/min |
| Tracking commands | 8340, 8380, 8420 r/min |
| Disturbance level | 5% fuel flow increase at t=4s |
| **Fitness & Constraints** | |
| Fitness function | ITAE = ∫(t * \|e(t)\|)dt |
| Overshoot limit | ≤ 5% |
| Settling time limit | ≤ 10s |
```