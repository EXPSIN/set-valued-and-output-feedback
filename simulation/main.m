close all; clear all; clc; clear simulation_utils;

%% Simulation time
config.duration = 36;                         % Total simulation time [s].
config.step = 50e-3;                          % Explicit-Euler step [s].
config.N = config.duration/config.step;       % Number of integration steps.

%% Quadrotor model
% State order: x = [y_1; y_2; dot(y_1); dot(y_2)].
config.plant.A = [0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 0, 0; 0, 0, 0, 0];
config.plant.B = [0, 0; 0, 0; 1, 0; 0, 1];
config.plant.C = [1, 0, 0, 0; 0, 1, 0, 0];
config.plant.n_x = size(config.plant.A, 1);
config.plant.n_u = size(config.plant.B, 2);
config.plant.n_y = size(config.plant.C, 1);

%% Coordinate transformation and reduced-order observer model
% N is the explicit matrix used in the paper; it satisfies C*N = 0 and N'*N = I.
config.plant.N = [0, 0; 0, 0; -1, 0; 0, 1];
config.plant.A_eta = zeros(2);                 % N'*A*N.
config.plant.B_eta = [-1, 0; 0, 1];           % N'*B.
config.plant.C_eta = [-1, 0; 0, 1];           % C*A*N.
config.plant.n_z = size(config.plant.A_eta, 1);
config.plant.n_eta = size(config.plant.A_eta, 1);

%% Safety-critical controller and LiDAR
config.plant.D = 0.7;                         % Required safety distance [m].
config.plant.Ds = 1.0;                        % Enlarged design distance [m].
config.safety.N_L = 15;                       % Number of positive-basis directions.
config.safety.k_alpha = 1.0;                  % Distance-feedback gain.
config.safety.k_L = 2.0;                      % Lipschitz truncation gain.
config.lidar.D_B = 3.0;                       % Maximum sensing range [m].
config.lidar.num_beams = 1000;                % Angular discretization of the omnidirectional measurement.

%% Velocity command and small-gain design
config.v_c = [2; 0.0];                        % Desired planar velocity [m/s].
config.command.v_bound = 2;                   % Bound on |v_c|.
config.command.v_derivative_bound = 0.0;      % Bound on |dot(v_c)|.
config.analysis.L_rho_m = 3.2;                % Controller Lipschitz value used by the current design.
config.regulation.gamma = 1/32;             % Velocity-regulation time-scale parameter.

%% Moving ship and obstacle scene
% The ship position is an independent visualization interface in world coordinates.
config.scene.obstacle_speed = 1.0;             % Common longitudinal drift [m/s].
config.scene.ship_position = @(t) [config.scene.obstacle_speed*t; 0];
config.scene.obstacle_horizontal_amplitude = 0.3;
config.scene.obstacle_period = 20;             % Oscillation period [s].

% Reference obstacle geometry before applying the time-varying displacement.
config.scene.circle_center0 = [5.9; 2.5];
config.scene.circle_radius = 0.4;
config.scene.triangle_vertices0 = zeros(2, 3, 2);
config.scene.triangle_vertices0(:, :, 1) = [2.2, 3.0, 3.0; 4.0, 4.0, 3.2];
config.scene.triangle_vertices0(:, :, 2) = [1.1, 1.1, 0.3; 1.0, 0.2, 0.2];

validate_design_parameters(config);

%% Initial conditions and data storage
x0 = [0; 2; 0; 0];                            % [position; velocity].
z0 = [0; -2];                                 % Reduced-order observer state.
X = [x0; z0];
X_record = zeros(config.plant.n_x+config.plant.n_z, config.N);
u_record = zeros(config.plant.n_u, config.N);
y_record = zeros(config.plant.n_y, config.N);

%% Closed-loop simulation
y = config.plant.C*x0;
[circle_center, circle_radius, triangle_vertices] = simulation_utils('obstacle_geometry', config.scene, 0);
H = simulation_utils([], circle_center, circle_radius, triangle_vertices, y, [], [], 0, config);

for i = 1:config.N
    t = i*config.step;
    [circle_center, circle_radius, triangle_vertices] = simulation_utils('obstacle_geometry', config.scene, t);
    [dX, ctrl_out] = closed_loop_system(X, t, circle_center, circle_radius, triangle_vertices, config);
    X = X+dX*config.step;
    X_record(:, i) = X;
    u_record(:, i) = ctrl_out.u;
    y = config.plant.C*X(1:config.plant.n_x);
    y_record(:, i) = y;
    H = simulation_utils(H, circle_center, circle_radius, triangle_vertices, y, X, ctrl_out, t, config);
end

% simulation_utils('export', H);



function [dX, ctrl_out] = closed_loop_system(X, t, circle_center, circle_radius, triangle_vertices, config)
x = X(1:config.plant.n_x);
z = X(1+config.plant.n_x:end);

y            = config.plant.C*x;
[d, nabla_d] = simulation_utils('obstacle_distance_gradient', y, circle_center, circle_radius, triangle_vertices);
lidar = simulation_utils('lidar_feedback', y, circle_center, circle_radius, triangle_vertices, config.lidar, 0);
ctrl_out = simulation_utils('controller', y, d, nabla_d, z, t, lidar, config);

dx = config.plant.A*x + config.plant.B*ctrl_out.u;
dz = (config.plant.A_eta-ctrl_out.L*config.plant.C_eta)*(z+ctrl_out.L*y) + config.plant.B_eta*ctrl_out.u;
dX = [dx; dz];
end


function validate_design_parameters(config)
small_gain_limit = 1/(2*config.analysis.L_rho_m);
safety_margin_limit = config.safety.k_alpha*(config.plant.Ds-config.plant.D)/(config.analysis.L_rho_m*(config.command.v_bound+config.command.v_derivative_bound+2*config.safety.k_alpha*(config.plant.Ds-config.plant.D)));
assert(config.regulation.gamma > 0 && config.regulation.gamma < small_gain_limit, 'gamma does not satisfy the small-gain condition.');
assert(config.regulation.gamma < safety_margin_limit, 'gamma does not satisfy the safety-margin condition.');
assert(config.lidar.D_B >= config.plant.Ds, 'The sensing range D_B must satisfy D_B >= D_s.');
assert(config.plant.Ds > config.plant.D, 'The design distance D_s must be larger than D.');
end
