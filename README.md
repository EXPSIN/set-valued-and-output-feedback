# Safety-Critical Output-Feedback Control with Set-Valued Obstacle Measurements

MATLAB simulation code accompanying:

**Safety-Critical Output-Feedback Control of Mobile Robots with Finite-Range, Set-Valued Obstacle Measurements**
Si Wu, Tengfei Liu, Jinliang Ding, Yuzhe Li, Tianyou Chai, and Zhong-Ping Jiang.

The simulations consider a planar quadrotor model navigating among moving convex obstacles using finite-range direction-distance measurements and an observer-based velocity-regulation controller.

## Requirements

- MATLAB with Optimization Toolbox (`quadprog` and `optimoptions`).
- The plotting code uses `exportgraphics` and `tiledlayout`.
- No compiled MEX file is required: the LiDAR calculation includes a MATLAB implementation.

## Run

Open MATLAB in the repository root, then change to the simulation directory:

```matlab
cd simulation
```

### Nominal simulation

```matlab
main
```

Runs the 36 s scenario with a 0.05 s integration step and displays the obstacle scene, velocity feasible sets, and time responses. Figure export is disabled by default. After the simulation finishes, export the figures with:

```matlab
simulation_utils('export', H)
```

### Robustness comparison

```matlab
results = main_robust();
```

Runs a nominal trajectory and three groups of 100 trials: measurement errors, external disturbances, and their combination. The random seed is fixed to 1. The comparison image is saved as `simulation_08_distance_controlinput_observererror.jpg` in the current directory. This computation can take substantially longer than the nominal simulation; trial count and disturbance bounds are configured near the top of `main_robust.m`.

### Numerical Lipschitz estimate

```matlab
estimate_L_rho
```

Samples 1,500 pairs of nearby safe positions for a fixed obstacle layout and velocity command. This produces a sampled numerical estimate, not a certified global Lipschitz bound.

## Files

| File | Purpose |
| --- | --- |
| `simulation/main.m` | Nominal simulation and visualization |
| `simulation/main_robust.m` | Robustness trials and comparison plots |
| `simulation/estimate_L_rho.m` | Sampled Lipschitz estimate |
| `simulation/simulation_utils.m` | Shared controller, LiDAR, obstacle geometry, and plotting functions |

## Numerical setup

The implementation discretizes omnidirectional measurements using 1,000 angular beams and integrates the closed-loop dynamics using explicit Euler steps. The simulation uses the double-integrator example from the paper (`CB = 0`). The code uses the manuscript notation `gamma` for the regulation parameter and `E_eta` for the decoupling matrix.

## Correspondence with the paper

The default simulation settings match Section V-A of the manuscript:

| Quantity | Value |
| --- | --- |
| Plant | Planar double integrator, `A = [0,I;0,0]`, `B = [0;I]`, `C = [I,0]` |
| Coordinate matrix | `N = [0,0;0,0;-1,0;0,1]` |
| Initial states | `x(0) = [0;2;0;0]`, `z(0) = [0;-2]` |
| Command | `v_c = [2;0]` m/s |
| Safety and sensing distances | `D = 0.7`, `D_s = 1.0`, `D_B = 3.0` m |
| Safety-controller parameters | `k_alpha = 1`, `k_L = 2`, `n_L = 15` |
| Positive-spanning directions | Uniform angles, `c_A = cos(2*pi/15)` |
| Observer gain | `L = [-1,-0.5;-0.5,1]` |
| Regulation parameter | `gamma = 1/32` |
| Resulting gains | `K = diag(32,-32)`, `T = 32*eye(2)` |
| Design Lipschitz estimate | `L_rho = 3.2` |
| Integration | Explicit Euler, step 0.05 s, duration 36 s |

The reference circle has center `[5.9;2.5]` and radius 0.4 m. The triangle vertices are `[2.2;4.0]`, `[3.0;4.0]`, `[3.0;3.2]` and `[1.1;1.0]`, `[1.1;0.2]`, `[0.3;0.2]`. Obstacle `i` has displacement `[t - 0.3*sin(pi*t/10 + 2*pi*i/3); 0]`.

`main_robust` includes the two disturbance setups described in the paper, with separate measurement-only and disturbance-only groups to show their individual effects. State disturbances and output errors are sampled within Euclidean unit balls; range errors are sampled in `[-1,1]` and the measured ranges are clipped to `[0,D_B]`. These samples are held over each simulation step. The plots show sampled trajectories and trial envelopes.

### Practical construction of rho_s

The implementation uses a practical nearest-beam construction:

```text
r_min = min_j r_j
l_min = direction of the first beam attaining r_min
rho_s = -k_alpha * l_min * max(D_s - r_min, 0)
```

The paper defines the direction by minimizing its norm over the convex hull of all minimizing directions. On the same sampled scan, the two constructions coincide whenever the minimizing direction is unique; they also both give zero when `r_min >= D_s`. In noiseless, nondegenerate configurations, ties below `D_s` occur only at exceptional positions, giving almost-everywhere agreement between these two constructions on the sampled scan. At tied minima, selecting the first beam can differ from the minimum-norm convex-hull choice. Range clipping under measurement errors can create ties, so the almost-everywhere statement is not an unconditional claim for noisy scans.

Separately, 1,000 finite beams approximate the paper's ideal all-direction measurements. Agreement of the two direction-selection rules on a sampled scan does not imply exact equality to the continuous-angle controller. The code retains the practical construction used for the simulations.

### Figure outputs

- `main` and `simulation_utils('export', H)`: scene and feasible-set figures corresponding to Figures 6 and 7, plus separate time-response plots.
- `main_robust`: the distance, input, and observer-error comparison corresponding to Figure 8.
- Figure exports contain the numerical plots; presentation annotations in the manuscript images were added separately.

## Validation

Checked with MATLAB R2025b and Optimization Toolbox using the MATLAB LiDAR implementation, without a MEX binary:

- Full 36 s nominal simulation: 720 steps, minimum sampled distance 1.297815 m (required distance 0.7 m); controller gains match `K = diag(32,-32)` and `T = 32*eye(2)`.
- Full 36 s robustness checks with two trials per disturbance group: all recorded distances, inputs, and observation errors were finite. The minimum sampled distances were 1.516948 m (measurement errors), 0.827905 m (disturbances), and 1.475826 m (combined).
- The default 1,500-pair Lipschitz estimation produced a maximum sampled quotient of 3.042017; the simulation retains the paper's design value 3.2. Sampling does not certify a global bound.
- The published robustness script keeps the default 100 trials per group; the release check used two trials per group, not the full 100-trial experiment.

These checks concern the sampled numerical trajectories. The figures in the paper also include manual presentation annotations that are not part of the plotting routines.
