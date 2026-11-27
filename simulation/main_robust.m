function results = main_robust()
%MAIN_ROBUST Run the nominal and robust closed-loop simulations.
%
%   MAIN_ROBUST() runs all simulations, generates the figures, and writes
%   the configured image files in the current directory.
%
%   RESULTS = MAIN_ROBUST() additionally returns the time grid and all
%   nominal/robust simulation records.

close all; clc;
clear simulation_utils;

config.duration = 36;
config.step     = 50e-3;
config.N        = config.duration/config.step;

% State order follows the paper: x = [y_1; y_2; dot(y_1); dot(y_2)].
config.plant.A        = [0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 0, 0; 0, 0, 0, 0];
config.plant.B        = [0, 0; 0, 0; 1, 0; 0, 1];
config.plant.C        = [1, 0, 0, 0; 0, 1, 0, 0];
config.plant.n_x      = size(config.plant.A, 1);
config.plant.n_u      = size(config.plant.B, 2);
config.plant.n_y      = size(config.plant.C, 1);
% Use explicit N matrix from the paper (Section 5.1), NOT QR decomposition
config.plant.N        = [0,  0; 0, 0; -1, 0; 0, 1]; % satisfies CN=0, N^T N=I
config.plant.A_eta    = [0, 0; 0, 0];  % N^T A N = 0_{2x2}
config.plant.B_eta    = [-1, 0; 0, 1]; % N^T B
config.plant.C_eta    = [-1, 0; 0, 1]; % C A N = B_eta
config.plant.n_z      = size(config.plant.A_eta, 1);
config.plant.n_eta    = size(config.plant.A_eta, 1);
config.plant.Ds       = 1.0;
config.plant.D        = 0.7;

config.safety.N_L     = 15;
config.safety.k_alpha = 1.0;
config.safety.k_L     = 2.0;
config.lidar.D_B      = 3.0;
config.lidar.num_beams = 1000; % angular discretization of the omnidirectional measurement F
config.v_c            = [2.0; 0.00];
config.command.v_bound = 2.0;
config.command.v_derivative_bound = 0.0;
config.analysis.L_rho_m = 3.2;
config.regulation.gamma = 1/32;

config.scene.obstacle_horizontal_amplitude = 0.3;
config.scene.obstacle_period = 20;
config.scene.obstacle_speed = 1.0;
config.scene.circle_center0 = [5.9; 2.5];
config.scene.circle_radius = 0.4;
config.scene.triangle_vertices0 = zeros(2, 3, 2);
config.scene.triangle_vertices0(:, :, 1) = [2.2, 3.0, 3.0; 4.0, 4.0, 3.2];
config.scene.triangle_vertices0(:, :, 2) = [1.1, 1.1, 0.3; 1.0, 0.2, 0.2];
config.robust.obstacle_velocity_bound = abs(config.scene.obstacle_speed)+2*pi*abs(config.scene.obstacle_horizontal_amplitude)/config.scene.obstacle_period;

config.robust.num_trials = 100;
config.robust.random_seed = 1;
config.robust.output_measurement_error_bound = 1;
config.robust.range_measurement_error_bound = 1;
config.robust.disturbance_bound = 1;
config.robust.figure_position = [80, 40, 1000, 800];
config.robust.distance_ylim = [0, 3.5];
config.robust.summary_output_file = 'simulation_08_distance_controlinput_observererror.jpg';

validate_design_parameters(config);

rng(config.robust.random_seed);

t_grid = (1:config.N)*config.step;
fprintf('Running nominal simulation...\n');
[nominal, nominal_input, nominal_eta_error] = simulate_distance_trajectory(config, false, false);
fprintf('Running measurement-error trials...\n');
measurement = run_robust_case(config, true, false, config.robust.num_trials);
fprintf('Running disturbance trials...\n');
disturbance = run_robust_case(config, false, true, config.robust.num_trials);
fprintf('Running combined trials...\n');
combined = run_robust_case(config, true, true, config.robust.num_trials);

[summary_fig, summary_axes] = create_robust_summary_figure(config.robust.figure_position);
plot_robust_distance(summary_axes(1), t_grid, nominal, measurement.distance, disturbance.distance, combined.distance, config);
plot_robust_input(summary_axes(2), t_grid, nominal_input, measurement.input, disturbance.input, combined.input, config);
plot_robust_observer_error(summary_axes(3), t_grid, nominal_eta_error, measurement.eta_error, disturbance.eta_error, combined.eta_error, config);
linkaxes(summary_axes, 'x');
export_robust_figure(summary_fig, config.robust.summary_output_file);

results.config = config;
results.t = t_grid;
results.nominal.distance = nominal;
results.nominal.input = nominal_input;
results.nominal.eta_error = nominal_eta_error;
results.measurement = measurement;
results.disturbance = disturbance;
results.combined = combined;
results.figures.summary = summary_fig;
results.axes = summary_axes;
end

function records = run_robust_case(config, use_measurement_error, use_disturbance, num_trials)
records.distance = zeros(num_trials, config.N);
records.input = zeros(num_trials, config.plant.n_u, config.N);
records.eta_error = zeros(num_trials, config.plant.n_eta, config.N);
progress_interval = max(1, floor(num_trials/100));
case_timer = tic;
for trial = 1:num_trials
    [distance_record, input_record, eta_error_record] = simulate_distance_trajectory(config, use_measurement_error, use_disturbance);
    records.distance(trial, :) = distance_record;
    records.input(trial, :, :) = reshape(input_record, 1, config.plant.n_u, config.N);
    records.eta_error(trial, :, :) = reshape(eta_error_record, 1, config.plant.n_eta, config.N);
    if mod(trial, progress_interval) == 0 || trial == num_trials
        fprintf('  Progress: %3.0f%% (%d/%d), elapsed %.1f s\n', 100*trial/num_trials, trial, num_trials, toc(case_timer));
    end
end
end

function [d_min_record, u_record, eta_error_record] = simulate_distance_trajectory(config, use_measurement_error, use_disturbance)
X = [0; 2; 0; 0; 0; -2];
d_min_record = zeros(1, config.N);
u_record = zeros(config.plant.n_u, config.N);
eta_error_record = zeros(config.plant.n_eta, config.N);

for i = 1:config.N
    t = i*config.step;
    [circle_center, circle_radius, triangle_vertices] = simulation_utils('obstacle_geometry', config.scene, t);

    [dX, ctrl_out] = robust_closed_loop_system(X, t, circle_center, circle_radius, triangle_vertices, config, use_measurement_error, use_disturbance);
    X = X + dX*config.step;

    y_true = config.plant.C*X(1:config.plant.n_x);
    d_true = simulation_utils('obstacle_distance_gradient', y_true, circle_center, circle_radius, triangle_vertices);
    d_min_record(i) = min(d_true);
    u_record(:, i) = ctrl_out.u;

    x = X(1:config.plant.n_x);
    z = X(1+config.plant.n_x:end);
    eta = config.plant.N'*x;
    eta_hat = z + ctrl_out.L*(y_true + ctrl_out.delta_y);
    eta_error_record(:, i) = eta - eta_hat;
end
end

function [dX, ctrl_out] = robust_closed_loop_system(X, t, circle_center, circle_radius, triangle_vertices, config, use_measurement_error, use_disturbance)
x = X(1:config.plant.n_x);
z = X(1+config.plant.n_x:end);
y_true = config.plant.C*x;
delta_y = zeros(config.plant.n_y, 1);
if use_measurement_error
    delta_y = random_norm_bounded_vector(config.plant.n_y, config.robust.output_measurement_error_bound);
end
y_observer = y_true + delta_y;

[d_meas, nabla_meas] = simulation_utils('obstacle_distance_gradient', y_true, circle_center, circle_radius, triangle_vertices);
range_error_bound = use_measurement_error*config.robust.range_measurement_error_bound;
lidar = simulation_utils('lidar_feedback', y_true, circle_center, circle_radius, triangle_vertices, config.lidar, range_error_bound);
ctrl_out = simulation_utils('controller', y_true, d_meas, nabla_meas, z, t, lidar, config, y_observer);
ctrl_out.delta_y = delta_y;

delta_x = zeros(config.plant.n_x, 1);
if use_disturbance
    delta_x = random_norm_bounded_vector(config.plant.n_x, config.robust.disturbance_bound);
end

dx = config.plant.A*x + config.plant.B*ctrl_out.u + delta_x;
dz = (config.plant.A_eta-ctrl_out.L*config.plant.C_eta)*(z+ctrl_out.L*y_observer) + config.plant.B_eta*ctrl_out.u;
dX = [dx; dz];
end

function validate_design_parameters(config)
small_gain_limit = 1/(2*config.analysis.L_rho_m);
safety_margin_limit = config.safety.k_alpha*(config.plant.Ds-config.plant.D)/(config.analysis.L_rho_m*(config.command.v_bound+config.command.v_derivative_bound+2*config.safety.k_alpha*(config.plant.Ds-config.plant.D)));
assert(config.regulation.gamma > 0 && config.regulation.gamma < small_gain_limit, 'gamma does not satisfy the small-gain condition.');
assert(config.regulation.gamma < safety_margin_limit, 'gamma does not satisfy the safety-margin condition.');
assert(config.plant.Ds > config.plant.D, 'The design distance D_s must be larger than D.');
assert(config.lidar.D_B >= config.plant.Ds, 'The sensing range D_B must satisfy D_B >= D_s.');
end

function v = random_norm_bounded_vector(n, radius)
v = zeros(n, 1);
if radius <= 0
    return;
end

direction = randn(n, 1);
while norm(direction) <= eps
    direction = randn(n, 1);
end
v = radius*rand()^(1/n)*direction/norm(direction);
end

function plot_robust_distance(ax, t, nominal, measurement, disturbance, combined, config)
p = robust_palette();
measurement_low = min(measurement, [], 1);
measurement_high = max(measurement, [], 1);
disturbance_low = min(disturbance, [], 1);
disturbance_high = max(disturbance, [], 1);
combined_low = min(combined, [], 1);
combined_high = max(combined, [], 1);

patch(ax, [0, t(end), t(end), 0], [0, 0, config.plant.D, config.plant.D], p.D_region, 'FaceAlpha', 0.16, 'EdgeColor', 'none', 'HandleVisibility', 'off');

dx_val = config.robust.disturbance_bound;
dy_val = config.robust.output_measurement_error_bound;
dm_val = config.robust.range_measurement_error_bound;
leg_combined    = sprintf('$\\bar{\\delta}_x=%.1f,\\ \\bar{\\delta}_y=%.1f,\\ \\bar{\\delta}_m=%.1f$', dx_val, dy_val, dm_val);
leg_disturbance = sprintf('$\\bar{\\delta}_x=%.1f,\\ \\bar{\\delta}_y=%.1f,\\ \\bar{\\delta}_m=%.1f$', dx_val,0,0);
leg_measurement = sprintf('$\\bar{\\delta}_x=%.1f,\\ \\bar{\\delta}_y=%.1f,\\ \\bar{\\delta}_m=%.1f$', 0, dy_val, dm_val);
leg_nominal     = sprintf('$\\bar{\\delta}_x=%.1f,\\ \\bar{\\delta}_y=%.1f,\\ \\bar{\\delta}_m=%.1f$', 0,0,0);

draw_range(ax, t, combined_low, combined_high, p.combined_range, p.combined, ':', leg_combined);
draw_range(ax, t, measurement_low, measurement_high, p.measurement_range, p.measurement, '--', leg_measurement);
draw_range(ax, t, disturbance_low, disturbance_high, p.disturbance_range, p.disturbance, '-.', leg_disturbance);

plot(ax, t, nominal(1, :), '-', 'Color', p.nominal, 'LineWidth', 1.6, 'DisplayName', leg_nominal);

plot(ax, [0, t(end)], [config.plant.Ds, config.plant.Ds], '--', 'Color', [0.15, 0.15, 0.15], 'LineWidth', 1.6, 'HandleVisibility', 'off');

configure_robust_axis(ax, t, config.robust.distance_ylim, 'Minimum Distance (m)');
legend(ax, 'show', 'Interpreter', 'latex', 'FontSize', 16, 'Location', 'best');
fill_robust_axis(ax);
hold(ax, 'off');
end

function plot_robust_input(ax, t, nominal, measurement, disturbance, combined, ~)
p = robust_palette();
draw_component_ranges(ax, t, combined, p.combined_range);
draw_component_ranges(ax, t, measurement, p.measurement_range);
draw_component_ranges(ax, t, disturbance, p.disturbance_range);
plot(ax, t, nominal(1, :), '-', 'Color', p.component_1, 'LineWidth', 1.5, 'DisplayName', '$[u]_1$');
plot(ax, t, nominal(2, :), '-', 'Color', p.component_2, 'LineWidth', 1.5, 'DisplayName', '$[u]_2$');
configure_robust_axis(ax, t, [-5, 15], 'Control Input (m/s^2)');
legend(ax, 'show', 'Interpreter', 'latex', 'FontSize', 16, 'Location', 'best');
fill_robust_axis(ax);
hold(ax, 'off');
end

function plot_robust_observer_error(ax, t, nominal, measurement, disturbance, combined, ~)
p = robust_palette();
draw_component_ranges(ax, t, combined, p.combined_range);
draw_component_ranges(ax, t, measurement, p.measurement_range);
draw_component_ranges(ax, t, disturbance, p.disturbance_range);
plot(ax, t, nominal(1, :), '-', 'Color', p.component_1, 'LineWidth', 1.5, 'DisplayName', '$[\eta-\hat{\eta}]_1$');
plot(ax, t, nominal(2, :), '--', 'Color', p.component_2, 'LineWidth', 1.5, 'DisplayName', '$[\eta-\hat{\eta}]_2$');
configure_robust_axis(ax, t, [-0.25, 1], 'Observer Error (m/s)');
legend(ax, 'show', 'Interpreter', 'latex', 'FontSize', 16, 'Location', 'best');
fill_robust_axis(ax);
hold(ax, 'off');
end

function draw_component_ranges(ax, t, records, fill_color)
for component = 1:size(records, 2)
    component_records = reshape(records(:, component, :), size(records, 1), size(records, 3));
    low = min(component_records, [], 1);
    high = max(component_records, [], 1);
    patch(ax, [t, fliplr(t)], [low, fliplr(high)], fill_color, 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
end

function [fig_handle, ax] = create_robust_summary_figure(position)
fig_handle = figure('Name', 'Robust Simulation', 'NumberTitle', 'off', 'Color', 'w', 'Position', position);
layout = tiledlayout(fig_handle, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(3, 1);
for k = 1:3
    ax(k) = nexttile(layout, k);
    hold(ax(k), 'on');
    box(ax(k), 'on');
    grid(ax(k), 'on');
    set(ax(k), 'FontSize', 16);
end
end

function fill_robust_axis(ax)
% Remove the default loose margin so each tile uses its available space.
drawnow;
ax.LooseInset = max(ax.TightInset, [0.01, 0.01, 0.01, 0.01]);
end

function configure_robust_axis(ax, t, y_limits, y_label)
xlim(ax, [0, t(end)]);
ylim(ax, y_limits);
xlabel(ax, 'Time (s)', 'FontSize', 16);
ylabel(ax, y_label, 'FontSize', 16);
end

function draw_range(ax, t, low, high, fill_color, line_color, line_style, label)
patch(ax, [t, fliplr(t)], [low, fliplr(high)], fill_color, 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax, t, low, line_style, 'Color', line_color, 'LineWidth', 1.6, 'DisplayName', label);
end

function p = robust_palette()
p.nominal = [0.0000, 0.4470, 0.7410];
p.measurement = [200, 120, 50] / 255;
p.disturbance = [35, 145, 205] / 255;
p.combined = [20, 145, 85] / 255;
p.measurement_range = [250, 205, 150] / 255;
p.disturbance_range = [120, 215, 250] / 255;
p.combined_range = [80, 220, 155] / 255;
p.D_region = [1.00, 0.35, 0.25];
p.component_1 = [0.0000, 0.4470, 0.7410];
p.component_2 = [0.8500, 0.3250, 0.0980];
end

function export_robust_figure(fig_handle, file_name)
drawnow;
file_path = fullfile(pwd, file_name);
try
    exportgraphics(fig_handle, file_path, 'Resolution', 300);
catch
    print(fig_handle, file_path, '-djpeg', '-r300');
end
end
