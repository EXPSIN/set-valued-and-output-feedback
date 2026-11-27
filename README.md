# Simulation Code

MATLAB code for **Safety-Critical Output-Feedback Control of Mobile Robots with Finite-Range, Set-Valued Obstacle Measurements**, by Si Wu, Tengfei Liu, Jinliang Ding, Yuzhe Li, Tianyou Chai, and Zhong-Ping Jiang.

## Requirements

MATLAB with Optimization Toolbox. Checked with MATLAB R2025b; no MEX binary is required.

## Run

From the repository root, enter the simulation directory and run the desired script:

```matlab
cd simulation
main                        % Nominal simulation
results = main_robust();     % Robustness comparison (100 trials per group)
estimate_L_rho              % Numerical Lipschitz estimate
```

Parameters are set near the top of each script. Shared functions are in `simulation_utils.m`. To export nominal figures, run `simulation_utils('export', H)` immediately after `main`.

## Implementation note

Obstacle measurements are discretized into 1,000 beams. The code uses a practical construction of `rho_s` by selecting a nearest beam. On the same sampled scan, this agrees with the paper's convex-hull construction when the minimizing direction is unique, giving almost-everywhere agreement in noiseless, nondegenerate configurations. Ties, including those caused by range clipping under noise, are resolved by selecting the first beam and may yield a different value.
