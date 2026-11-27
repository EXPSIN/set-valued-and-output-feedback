# Simulation Code

MATLAB code for **Safety-Critical Output-Feedback Control of Mobile Robots with Finite-Range, Set-Valued Obstacle Measurements**, by Si Wu, Tengfei Liu, Jinliang Ding, Yuzhe Li, Tianyou Chai, and Zhong-Ping Jiang.

## Requirements

Checked with MATLAB R2025b.

## Run

From the repository root, enter the simulation directory and run the desired script:

```matlab
cd simulation
main                        % Nominal simulation
results = main_robust();     % Robustness comparison (100 trials per group)
estimate_L_rho              % Numerical Lipschitz estimate
```

Parameters are set near the top of each script. Shared functions are in `simulation_utils.m`. To export nominal figures, run `simulation_utils('export', H)` immediately after `main`.
