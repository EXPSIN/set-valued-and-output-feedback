close all; clear variables; clc; clear simulation_utils;

%% Safety-controller parameters
config.plant.D = 0.7;
config.plant.Ds = 1.0;
config.safety.N_L = 15;
config.safety.k_alpha = 1.0;
config.safety.k_L = 2.0;
config.lidar.D_B = 3.0;
config.lidar.num_beams = 1000;

%% Command bounds and selected regulation parameter
config.v_c = [2.0; 0.0];
config.command.v_bound = 2.0;
config.command.v_derivative_bound = 0.0;
config.analysis.L_rho_m = 3.2;
config.regulation.gamma = 1/32;

%% Moving-obstacle scene
config.scene.obstacle_speed = 1.0;
config.scene.obstacle_horizontal_amplitude = 0.3;
config.scene.obstacle_period = 20;
config.scene.circle_center0 = [5.9; 2.5];
config.scene.circle_radius = 0.4;
config.scene.triangle_vertices0 = zeros(2, 3, 2);
config.scene.triangle_vertices0(:, :, 1) = [2.2, 3.0, 3.0; 4.0, 4.0, 3.2];
config.scene.triangle_vertices0(:, :, 2) = [1.1, 1.1, 0.3; 1.0, 0.2, 0.2];

%% Numerical estimation settings
% The simulation uses the constant command v_c = [2;0]. For the current
% obstacle trajectories, the minimum inter-obstacle boundary clearance over
% one period occurs at t = 0 (circle and upper triangle, about 2.0812 m).
% Fix this most crowded obstacle layout, evaluate rho at nearby safe positions,
% and use the local quotient
% ||rho(y_2,v_c)-rho(y_1,v_c)||/||y_2-y_1||. This is a numerical constant
% for the simulated command; the theoretical result only requires existence.
config.estimate.snapshot_time = 0;
config.estimate.num_pairs = 1500;
config.estimate.pair_distance = 0.05;
config.estimate.random_seed = 3;

rng(config.estimate.random_seed);
[A_L, c_A] = gen_A_L_local(config.safety.N_L);
opt = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set', 'MaxIterations', 1000);
t = config.estimate.snapshot_time;
[circle_center, circle_radius, triangle_vertices] = simulation_utils('obstacle_geometry', config.scene, t);
[y1, y2] = sample_local_pairs(config, circle_center, circle_radius, triangle_vertices);
[L_rho_estimate, best] = estimate_local_slopes(config, circle_center, circle_radius, triangle_vertices, y1, y2, A_L, c_A, opt);
best.t = t;
L_rho = config.analysis.L_rho_m;
gamma_small_gain = 1/(2*L_rho);
gamma_safety_margin = config.safety.k_alpha*(config.plant.Ds-config.plant.D)/(L_rho*(config.command.v_bound+config.command.v_derivative_bound+2*config.safety.k_alpha*(config.plant.Ds-config.plant.D)));
gamma_upper_bound = min(gamma_small_gain, gamma_safety_margin);

print_parameter_summary(config, c_A, L_rho, L_rho_estimate, gamma_small_gain, gamma_safety_margin, gamma_upper_bound, best);


function [L_y, best] = estimate_local_slopes(config, circle_center, circle_radius, triangle_vertices, y1, y2, A_L, c_A, opt)
n_pairs = size(y1, 2);
L_y = 0;
best.y1 = [nan; nan];
best.y2 = [nan; nan];
best.rho1 = [nan; nan];
best.rho2 = [nan; nan];

for i = 1:n_pairs
    rho1 = rho_map(y1(:, i), config.v_c, circle_center, circle_radius, triangle_vertices, config, A_L, c_A, opt);
    rho2 = rho_map(y2(:, i), config.v_c, circle_center, circle_radius, triangle_vertices, config, A_L, c_A, opt);
    slope = norm(rho2-rho1)/norm(y2(:, i)-y1(:, i));
    if slope > L_y
        L_y = slope;
        best.y1 = y1(:, i);
        best.y2 = y2(:, i);
        best.rho1 = rho1;
        best.rho2 = rho2;
    end
end
end


function [y1, y2] = sample_local_pairs(config, circle_center, circle_radius, triangle_vertices)
all_x = [circle_center(1)-circle_radius, circle_center(1)+circle_radius, reshape(triangle_vertices(1, :, :), 1, [])];
all_y = [circle_center(2)-circle_radius, circle_center(2)+circle_radius, reshape(triangle_vertices(2, :, :), 1, [])];
x_range = [min(all_x)-config.lidar.D_B, max(all_x)+config.lidar.D_B];
y_range = [min(all_y)-config.lidar.D_B, max(all_y)+config.lidar.D_B];
y1 = zeros(2, 0);
y2 = zeros(2, 0);

while size(y1, 2) < config.estimate.num_pairs
    batch_size = config.estimate.num_pairs;
    candidate1 = [x_range(1)+diff(x_range)*rand(1, batch_size); y_range(1)+diff(y_range)*rand(1, batch_size)];
    theta = 2*pi*rand(1, batch_size);
    candidate2 = candidate1 + config.estimate.pair_distance*[cos(theta); sin(theta)];
    keep = false(1, batch_size);
    for k = 1:batch_size
        d1 = simulation_utils('obstacle_distance_gradient', candidate1(:, k), circle_center, circle_radius, triangle_vertices);
        d2 = simulation_utils('obstacle_distance_gradient', candidate2(:, k), circle_center, circle_radius, triangle_vertices);
        safe_pair = min(d1) >= config.plant.D && min(d2) >= config.plant.D;
        sensed_pair = min(d1) <= config.lidar.D_B || min(d2) <= config.lidar.D_B;
        keep(k) = safe_pair && sensed_pair;
    end
    y1 = [y1, candidate1(:, keep)]; %#ok<AGROW>
    y2 = [y2, candidate2(:, keep)]; %#ok<AGROW>
end

y1 = y1(:, 1:config.estimate.num_pairs);
y2 = y2(:, 1:config.estimate.num_pairs);
end


function v_ref = rho_map(y, v_c, circle_center, circle_radius, triangle_vertices, config, A_L, c_A, opt)
lidar = simulation_utils('lidar_feedback', y, circle_center, circle_radius, triangle_vertices, config.lidar, 0);

% Practical construction of rho_s from the nearest LiDAR distance and direction.
[r_min, min_idx] = min(lidar.r);
l_min = lidar.l(:, min_idx);
rho_s = config.safety.k_alpha*l_min*min(r_min-config.plant.Ds, 0);
r_L = regularized_distance_local(lidar, A_L', c_A, config.safety.k_L);
b_L = max(-A_L*rho_s+config.safety.k_alpha*(r_L(:)-config.plant.Ds), 0);
qp_b = b_L+A_L*rho_s;

% Since b_L >= 0, rho_s is feasible for A_L*(v-rho_s) <= b_L.
% The QP computes v_ref = rho(M(y),v_c); rho_s is only its feasible start.
[v_ref, ~, exitflag] = quadprog(eye(numel(y)), -v_c, A_L, qp_b, [], [], [], [], rho_s, opt);
if exitflag <= 0 || any(~isfinite(v_ref))
    error('The safety-critical QP failed while estimating L_rho (exitflag = %d).', exitflag);
end
end


function [A_L, c_A] = gen_A_L_local(N)
theta = linspace(0, 2*pi, N+1)';
A_L = [cos(theta(1:end-1)), sin(theta(1:end-1))];
c_A = cos(2*pi/N);
end


function r_L = regularized_distance_local(lidar, query_dirs, c_A, k_L)
n_query = size(query_dirs, 2);
r_L = zeros(n_query, 1);
for j = 1:n_query
    alignment = query_dirs(:, j)'*lidar.l;
    penalty = k_L*max(c_A-alignment, 0);
    r_L(j) = min(lidar.r+penalty);
end
end


function print_parameter_summary(config, c_A, L_rho, L_rho_estimate, gamma_small_gain, gamma_safety_margin, gamma_upper_bound, best)
fprintf('\n================ Parameter summary ================\n');
fprintf('D                         = %.6f\n', config.plant.D);
fprintf('D_s                       = %.6f\n', config.plant.Ds);
fprintf('D_B                       = %.6f\n', config.lidar.D_B);
fprintf('n_L                       = %d\n', config.safety.N_L);
fprintf('c_A = cos(2*pi/n_L)       = %.6f\n', c_A);
fprintf('k_alpha                   = %.6f\n', config.safety.k_alpha);
fprintf('k_L                       = %.6f\n', config.safety.k_L);
fprintf('v_c bound                 = %.6f\n', config.command.v_bound);
fprintf('dot(v_c) bound            = %.6f\n', config.command.v_derivative_bound);
fprintf('Fixed simulation v_c      = [%.6f; %.6f]\n', config.v_c(1), config.v_c(2));
fprintf('Fixed obstacle time       = %.6f s\n', best.t);
fprintf('Estimated L_rho (num_beams=%d) = %.6f\n', config.lidar.num_beams, L_rho_estimate);
fprintf('Design L_rho used         = %.6f\n', L_rho);
fprintf('\n================ gamma calculation ==============\n');
fprintf('Small-gain bound          = 1/(2*L_rho) = %.8f\n', gamma_small_gain);
fprintf('Safety-margin bound       = %.8f\n', gamma_safety_margin);
fprintf('Admissible interval       = 0 < gamma < %.8f\n', gamma_upper_bound);
fprintf('Selected gamma          = 1/32 = %.8f\n', config.regulation.gamma);
if config.regulation.gamma < gamma_upper_bound
    fprintf('Condition check           = satisfied\n');
else
    fprintf('Condition check           = NOT satisfied\n');
end
fprintf('\nMaximum sampled spatial slope at the fixed obstacle layout:\n');
fprintf('y_1  = [%.6f; %.6f], rho_1 = [%.6f; %.6f]\n', best.y1(1), best.y1(2), best.rho1(1), best.rho1(2));
fprintf('y_2  = [%.6f; %.6f], rho_2 = [%.6f; %.6f]\n', best.y2(1), best.y2(2), best.rho2(1), best.rho2(2));
fprintf('=====================================================\n');
end
