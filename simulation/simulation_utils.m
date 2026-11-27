function varargout = simulation_utils(action, varargin)
%SIMULATION_UTILS Centralized simulation utilities and visualization.
% Numeric first input updates the nominal-simulation figures. Text actions
% expose the shared controller, LiDAR, geometry, distance, and export code.

if ischar(action) || (isstring(action) && isscalar(action))
    switch char(action)
        case 'controller'
            varargout{1} = controller(varargin{:});
        case 'lidar_feedback'
            varargout{1} = lidar_feedback(varargin{:});
        case 'obstacle_geometry'
            [varargout{1:nargout}] = obstacle_geometry(varargin{:});
        case 'obstacle_distance_gradient'
            [varargout{1:nargout}] = obstacle_distance_gradient(varargin{:});
        case 'export'
            export_simulation_figures(varargin{:});
        otherwise
            error('Unknown simulation utility action: %s.', char(action));
    end
    return;
end

varargout{1} = graphic_update(action, varargin{:});
end


function H = graphic_update(H, circle_center, circle_radius, triangle_vertices, y, X, ctrl_out, t_current, config)
    if nargin < 1 || isempty(H)
        H = struct();
    end
    y_world = y;
    D = config.plant.D;
    Ds = config.plant.Ds;
    plot_config = graphics_configuration();

    if isempty(fieldnames(H))
        H.obstacle_fig = figure('Name', 'Obstacles', 'NumberTitle', 'off');
        axis equal;
        grid on;
        hold on;
        box on;
        set(gcf, 'position', plot_config.scene.figure_position, 'color', 'w');
        set(gca, 'fontsize', 16);
        set(gca, 'Position', [0.04, 0.19, 0.95, 0.80]);
        axis equal;
        axis(plot_config.scene.axis);
        xlabel('$[y]_1 (m)$', 'Interpreter', 'latex', 'FontSize', 16);
        ylabel('$[y]_2 (m)$', 'Interpreter', 'latex', 'FontSize', 16);
        H.obstacle_ax = gca;
        H.ghost_layer = hggroup('Parent', H.obstacle_ax);
        H.platform = draw_obstacle_platform(config.scene.ship_position(0));
        H.lidar = draw_lidar_overlay(gca, plot_config.scene);
        H.handles = draw_obstacles(circle_center, circle_radius, triangle_vertices);
        H.traj = draw_trajectory(y_world);
        [H.circleD, H.circleDs] = draw_vehicle_circles(y_world, D, Ds);
        H.timeLabel = draw_ghost_time_label(gca, y_world, 0);
        H.ghost = init_scene_ghosts(plot_config.ghost, H.obstacle_ax, H.ghost_layer);
        H.scene_velocity = draw_velocity_overlay(gca);

        d_labels = arrayfun(@(j) sprintf('$d_%d$', j), 1:3, 'UniformOutput', false);
        u_labels = arrayfun(@(j) sprintf('$[u]_%d$', j), 1:config.plant.n_u, 'UniformOutput', false);
        v_labels = [arrayfun(@(j) sprintf('$[v_c]_%d$', j), 1:config.plant.n_y, 'UniformOutput', false), ...
                    arrayfun(@(j) sprintf('$[v_*]_%d$', j), 1:config.plant.n_y, 'UniformOutput', false)];
        eta_labels = arrayfun(@(j) sprintf('$[\\eta-\\hat{\\eta}]_%d$', j), 1:config.plant.n_eta, 'UniformOutput', false);

        [H.distance_fig, H.distance_lines] = create_time_figure('Distance', [40, 620, 850, 320], d_labels, distance_line_specs(3));
        add_distance_guides(H.distance_fig, config.duration, D, Ds);
        [H.input_fig, H.input_lines] = create_time_figure('Control Input', [80, 590, 850, 320], u_labels, component_line_specs(config.plant.n_u, 'solid'));
        [H.velocity_fig, H.velocity_lines] = create_time_figure('Velocity Command and Safe Velocity', [120, 560, 850, 320], v_labels, velocity_line_specs(config.plant.n_y));
        [H.eta_error_fig, H.eta_error_lines] = create_time_figure('Observer Error', [160, 530, 850, 320], eta_labels, component_line_specs(config.plant.n_eta, 'mixed'));
        H.feasible_fig = create_feasible_figure('Feasible Set Reshaping', [20, 450, 1900, 320], plot_config);
        H.ghost.feasible_fig = H.feasible_fig;
    else
        update_obstacle_platform(H.platform, config.scene.ship_position(t_current));
        update_obstacles(H.handles, circle_center, circle_radius, triangle_vertices);
        update_trajectory(H.traj, y_world);

        update_vehicle_circles(H.circleD, H.circleDs, y_world, D, Ds);
        update_lidar_overlay(H.lidar, ctrl_out.safety, y);
        update_time_label(H.timeLabel, y_world, t_current);
        if ~isfield(H.ghost, 'feasible_fig') || ~isgraphics(H.ghost.feasible_fig, 'figure')
            H.ghost.feasible_fig = H.feasible_fig;
        end
        update_velocity_overlay(H.scene_velocity, ctrl_out.safety, y_world);
        update_feasible_figure(H.feasible_fig, ctrl_out.safety);
        H.ghost = update_scene_ghosts(H.ghost, H.platform, H.handles, y, y_world, D, Ds, t_current, ctrl_out.safety);

        d = obstacle_distance_gradient( ...
            y, circle_center, circle_radius, triangle_vertices);
        append_time_points(H.distance_lines, t_current, d(:));
        append_time_points(H.input_lines, t_current, ctrl_out.u(:));
        append_time_points(H.velocity_lines, t_current, [ctrl_out.v_c(:); ctrl_out.v_ref(:)]);

        x = X(1:config.plant.n_x);
        z = X(1+config.plant.n_x:end);
        eta = config.plant.N' * x;
        eta_hat = z + ctrl_out.L * y;
        eta_error = eta - eta_hat;
        append_time_points(H.eta_error_lines, t_current, eta_error(:));
    end
    drawnow limitrate;
end

function config = graphics_configuration()
    % Main scene window and LiDAR-region appearance.
    config.scene.figure_position = [20, 50, 1900, 360];
    config.scene.axis = [-3, 46, -1.6, 5.6];
    config.scene.lidar_alpha = 0.5;
    config.scene.regularized_alpha = 0.5;

    % Historical snapshots shown in the scene and feasible-set figure.
    config.ghost.interval = 9;
    config.ghost.max_frames = 5;
    config.ghost.vehicle_d_alpha = 1;
    config.ghost.lidar_alpha = 0.5;
    config.ghost.regularized_alpha = 0.5;

    % Velocity-space limits in the feasible-set figure.
    config.feasible.xlim = [-3, 3];
    config.feasible.ylim = [-3, 3];
end

function h = draw_obstacle_platform(position)
    heading = 0;
    p = scene_palette();
    outline_x = [0, 5, 7, 5, 0, 0];
    outline_y = [4, 4, 2, 0.2, 0.2, 4];

    h.Transform = hgtransform('Parent', gca);
    h.Patch = patch('Parent', h.Transform, ...
        'XData', outline_x, ...
        'YData', outline_y, ...
        'FaceColor', p.shipFace, ...
        'FaceAlpha', 1, ...
        'EdgeColor', p.shipEdge, ...
        'LineWidth', 1.1);
    update_obstacle_platform(h, position, heading);
end

function update_obstacle_platform(platform, position, heading)
    if nargin < 3
        heading = 0;
    end
    if ~isstruct(platform) || ~isfield(platform, 'Transform') || ~isgraphics(platform.Transform)
        return;
    end
    position = reshape(position, 2, 1);
    c = cos(heading);
    s = sin(heading);
    transform_matrix = [c, -s, 0, position(1); s, c, 0, position(2); 0, 0, 1, 0; 0, 0, 0, 1];
    set(platform.Transform, 'Matrix', transform_matrix);
end

function ghost = init_scene_ghosts(ghost_config, ax, layer)
    ghost.config = ghost_config;
    ghost.ax = ax;
    ghost.layer = layer;
    ghost.feasible_fig = [];
    ghost.last_time = -inf;
    ghost.count = 0;
end

function ghost = update_scene_ghosts(ghost, platform, obstacle_handles, y, y_world, D, Ds, t_current, safety)
    if t_current - ghost.last_time < ghost.config.interval || ghost.count >= ghost.config.max_frames
        return;
    end

    draw_scene_ghost(ghost.ax, ghost.layer, platform, obstacle_handles, y, y_world, D, Ds, t_current, safety, ghost.config);
    if isfield(ghost, 'feasible_fig') && isgraphics(ghost.feasible_fig, 'figure')
        record_feasible_snapshot(ghost.feasible_fig, safety);
    end
    ghost.last_time = t_current;
    ghost.count = ghost.count+1;
end

function draw_scene_ghost(ax, parent, platform, obstacle_handles, y, y_world, D, Ds, t_current, safety, cfg)
    copy_ship_ghost(parent, platform);
    draw_lidar_ghost(parent, safety, y, cfg);
    copy_obstacle_ghosts(parent, obstacle_handles);
    draw_vehicle_ds_ghost(parent, y_world, Ds);
    draw_vehicle_d_ghost(parent, y_world, D, cfg.vehicle_d_alpha);
    draw_velocity_ghost(parent, safety, y_world);
    draw_ghost_time_label(parent, y_world, t_current, ax);
end

function h = draw_ghost_time_label(parent, y_world, t_current, ax)
    if nargin < 4 || isempty(ax)
        ax = parent_axes(parent);
    end
    p = scene_palette();
    y_limit = ylim(ax);
    y_pos = y_limit(2);
    h = text('Parent', parent, ...
        'Position', [y_world(1), y_pos-1.0, 0], ...
        'String', sprintf('$t=%.0f\\,\\mathrm{s}$', t_current), ...
        'Interpreter', 'latex', ...
        'FontSize', 16, ...
        'FontWeight', 'bold', ...
        'Color', p.text, ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'HandleVisibility', 'off');
end

function update_time_label(h, y_world, t_current)
    if ~isgraphics(h)
        return;
    end
    ax = parent_axes(get(h, 'Parent'));
    y_limit = ylim(ax);
    y_pos = y_limit(2);
    set(h, 'Position', [y_world(1), y_pos-1.0, 0], ...
        'HorizontalAlignment', 'center', ...
        'String', sprintf('$t=%.0f\\,\\mathrm{s}$', t_current));
end

function h = copy_ship_ghost(parent, platform)
    h = copyobj(platform.Transform, parent);
    set(findall(h), 'HandleVisibility', 'off');
end

function handles = copy_obstacle_ghosts(parent, obstacle_handles)
    handles = copyobj(obstacle_handles, parent);
    set(handles, 'HandleVisibility', 'off');
end

function h = draw_vehicle_ds_ghost(ax, y_world, Ds)
    p = scene_palette();
    theta = linspace(0, 2*pi, 80);
    circleDs = [Ds*cos(theta); Ds*sin(theta)] + y_world;
    h = line('Parent', ax, ...
        'XData', circleDs(1, :), ...
        'YData', circleDs(2, :), ...
        'LineStyle', '--', ...
        'Color', p.safety, ...
        'LineWidth', 0.9, ...
        'HandleVisibility', 'off');
end

function h = draw_vehicle_d_ghost(ax, y_world, D, alpha)
    p = scene_palette();
    theta = linspace(0, 2*pi, 64);
    circleD = [D*cos(theta); D*sin(theta)] + y_world;
    h = patch('Parent', ax, ...
        'XData', circleD(1, :), ...
        'YData', circleD(2, :), ...
        'FaceColor', p.vehicle, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', alpha, ...
        'HandleVisibility', 'off');
end

function handles = draw_lidar_ghost(ax, safety, y, cfg)
    handles = gobjects(2, 1);
    if ~isstruct(safety) || ~isfield(safety, 'lidar')
        return;
    end

    p = scene_palette();
    lidar = safety.lidar;
    scan_points_world = lidar.points;
    feedback_boundary = close_region_boundary(scan_points_world);
    handles(1) = patch('Parent', ax, ...
        'XData', feedback_boundary(1, :), ...
        'YData', feedback_boundary(2, :), ...
        'FaceColor', p.lidar, ...
        'FaceAlpha', cfg.lidar_alpha, ...
        'EdgeColor', p.lidar, ...
        'EdgeAlpha', 1.0, ...
        'LineWidth', 0.9, ...
        'HandleVisibility', 'off');

    if isfield(safety, 'r_L_scan')
        rL_points = repmat(y, 1, size(lidar.l, 2)) + lidar.l.*repmat(safety.r_L_scan, 2, 1);
        rL_points_world = rL_points;
        regularized_boundary = close_region_boundary(rL_points_world);
        handles(2) = patch('Parent', ax, ...
            'XData', regularized_boundary(1, :), ...
            'YData', regularized_boundary(2, :), ...
            'FaceColor', p.regularized, ...
            'FaceAlpha', cfg.regularized_alpha, ...
            'EdgeColor', p.regularized, ...
            'EdgeAlpha', 1.0, ...
            'LineWidth', 1.7, ...
            'HandleVisibility', 'off');
    end
end

function handles = draw_velocity_ghost(ax, safety, y_world)
    handles = gobjects(2, 1);
    if ~isstruct(safety) || ~isfield(safety, 'v_c') || ~isfield(safety, 'v_ref')
        return;
    end

    p = scene_palette();
    colors = [p.nominal; p.safe];
    handles(1) = draw_quiver(ax, y_world(1), y_world(2), ...
        safety.v_c(1), safety.v_c(2), 0, ...
        'Color', colors(1, :), ...
        'LineStyle', ':', ...
        'LineWidth', 1.5, ...
        'MaxHeadSize', 0.9, ...
        'HandleVisibility', 'off');
    handles(2) = draw_quiver(ax, y_world(1), y_world(2), ...
        safety.v_ref(1), safety.v_ref(2), 0, ...
        'Color', colors(2, :), ...
        'LineStyle', '-', ...
        'LineWidth', 3.0, ...
        'MaxHeadSize', 1.05, ...
        'HandleVisibility', 'off');
end

function ax = parent_axes(parent)
    if isgraphics(parent, 'axes')
        ax = parent;
    else
        ax = ancestor(parent, 'axes');
    end
end

function h = draw_quiver(parent, x, y, u, v, scale, varargin)
    ax = parent_axes(parent);
    if isgraphics(parent, 'axes')
        h = quiver(ax, x, y, u, v, scale, varargin{:});
    else
        h = draw_group_arrow(parent, x, y, u, v, varargin{:});
    end
end

function h = draw_group_arrow(parent, x, y, u, v, varargin)
    color = option_value(varargin, 'Color', [0, 0, 0]);
    line_width = option_value(varargin, 'LineWidth', 1.0);
    line_style = option_value(varargin, 'LineStyle', '-');
    max_head_size = option_value(varargin, 'MaxHeadSize', 0.8);
    handle_visibility = option_value(varargin, 'HandleVisibility', 'off');

    h = hggroup('Parent', parent, 'HandleVisibility', handle_visibility);
    p0 = [x; y];
    vec = [u; v];
    if any(~isfinite([p0; vec])) || norm(vec) <= eps
        line('Parent', h, ...
            'XData', [nan, nan], ...
            'YData', [nan, nan], ...
            'Color', color, ...
            'LineWidth', line_width, ...
            'LineStyle', line_style, ...
            'HandleVisibility', 'off');
        patch('Parent', h, ...
            'XData', [nan, nan, nan], ...
            'YData', [nan, nan, nan], ...
            'FaceColor', color, ...
            'EdgeColor', color, ...
            'HandleVisibility', 'off');
        return;
    end

    p1 = p0 + vec;
    dir_vec = vec/norm(vec);
    normal_vec = [-dir_vec(2); dir_vec(1)];
    head_len = 0.18*norm(vec)*max(0.5, min(1.8, max_head_size));
    head_width = 0.42*head_len;
    base = p1 - head_len*dir_vec;
    head = [p1, base + head_width*normal_vec, base - head_width*normal_vec];

    line('Parent', h, ...
        'XData', [p0(1), p1(1)], ...
        'YData', [p0(2), p1(2)], ...
        'Color', color, ...
        'LineWidth', line_width, ...
        'LineStyle', line_style, ...
        'HandleVisibility', 'off');
    patch('Parent', h, ...
        'XData', head(1, :), ...
        'YData', head(2, :), ...
        'FaceColor', color, ...
        'EdgeColor', color, ...
        'LineWidth', 0.5*line_width, ...
        'HandleVisibility', 'off');
end

function value = option_value(args, name, default_value)
    value = default_value;
    for k = 1:2:numel(args)-1
        if ischar(args{k}) && strcmpi(args{k}, name)
            value = args{k+1};
            return;
        end
    end
end

function p = scene_palette()
    p.peach = [254, 219, 179]/255;
    p.sky = [160, 230, 255]/255;
    p.mint = [113, 232, 178]/255;
    p.shipFace = [0.84, 0.85, 0.86];
    p.shipEdge = [0.32, 0.34, 0.36];
    p.shipLine = [0.55, 0.57, 0.59];
    p.obstacleFace = [0, 0, 0];
    p.obstacleEdge = [0, 0, 0];
    p.dynamicFace = [0, 0, 0];
    p.dynamicEdge = [0, 0, 0];
    p.vehicle = [0.42, 0.70, 0.95];
    p.safety = [0.18, 0.18, 0.18];
    p.lidar = p.sky;
    p.regularized = p.peach;
    p.nominal = [0.00, 0.00, 0.00];
    p.safe = [0.90, 0.24, 0.06];
    p.originalFeasibleFace = p.sky;
    p.originalFeasibleEdge = [0.05, 0.48, 0.72];
    p.feasibleFace = p.peach;
    p.feasibleEdge = [0.76, 0.42, 0.12];
    p.text = [0.18, 0.22, 0.24];
end

function data = draw_lidar_overlay(ax, scene_config)
    p = scene_palette();
    lidar_alpha = scene_config.lidar_alpha;
    regularized_alpha = scene_config.regularized_alpha;
    data.feedback_region = patch('Parent', ax, ...
        'XData', nan, ...
        'YData', nan, ...
        'FaceColor', p.lidar, ...
        'FaceAlpha', lidar_alpha, ...
        'EdgeColor', p.lidar, ...
        'EdgeAlpha', 1.0, ...
        'LineWidth', 1.0, ...
        'DisplayName', '$\mathcal{F}$');
    data.regularized_region = patch('Parent', ax, ...
        'XData', nan, ...
        'YData', nan, ...
        'FaceColor', p.regularized, ...
        'FaceAlpha', regularized_alpha, ...
        'EdgeColor', p.regularized, ...
        'EdgeAlpha', 1.0, ...
        'LineWidth', 2.2, ...
        'DisplayName', '$r_L(\mathcal{F},l)$');
end

function update_lidar_overlay(data, safety, y)
    if ~isfield(safety, 'lidar')
        return;
    end
    lidar = safety.lidar;

    scan_points_world = lidar.points;
    feedback_boundary = close_region_boundary(scan_points_world);
    set(data.feedback_region, ...
        'XData', feedback_boundary(1, :), ...
        'YData', feedback_boundary(2, :));

    rL_points = repmat(y, 1, size(lidar.l, 2)) + lidar.l.*repmat(safety.r_L_scan, 2, 1);
    rL_points_world = rL_points;
    regularized_boundary = close_region_boundary(rL_points_world);
    set(data.regularized_region, ...
        'XData', regularized_boundary(1, :), ...
        'YData', regularized_boundary(2, :));
end

function data = draw_velocity_overlay(ax)
    p = scene_palette();
    data.v_c = quiver(ax, nan, nan, nan, nan, 0, ...
        'Color', p.nominal, ...
        'LineStyle', ':', ...
        'LineWidth', 2.2, ...
        'MaxHeadSize', 1.0, ...
        'DisplayName', '$v_c$');
    data.v_ref = quiver(ax, nan, nan, nan, nan, 0, ...
        'Color', p.safe, ...
        'LineStyle', '-', ...
        'LineWidth', 3.2, ...
        'MaxHeadSize', 1.05, ...
        'DisplayName', '$v_*$');
end

function update_velocity_overlay(data, safety, y_world)
    if ~isstruct(safety) || ~isfield(safety, 'v_c') || ~isfield(safety, 'v_ref')
        return;
    end

    set_scene_quiver(data.v_c, y_world, safety.v_c(:));
    set_scene_quiver(data.v_ref, y_world, safety.v_ref(:));
end

function set_scene_quiver(h, origin, vector)
    set(h, ...
        'XData', origin(1), ...
        'YData', origin(2), ...
        'UData', vector(1), ...
        'VData', vector(2));
end

function boundary = close_region_boundary(points)
    if isempty(points)
        boundary = [nan; nan];
        return;
    end
    boundary = [points, points(:, 1)];
end

function fig_handle = create_feasible_figure(fig_name, position, config)
    p = scene_palette();
    fig_handle = figure('Name', fig_name, 'NumberTitle', 'off');
    set(fig_handle, 'Position', position, 'Color', 'w');

    n_cols = max(1, config.ghost.max_frames);
    layout = tiledlayout(fig_handle, 1, n_cols, 'TileSpacing', 'compact', 'Padding', 'compact');

    data.axes = gobjects(1, n_cols);
    data.items = cell(1, n_cols);
    data.snapshot_count = max(0, n_cols - 1);
    data.live_index = n_cols;
    data.history = {};
    data.palette = p;
    data.fixed_limits = feasible_axis_limits(config.feasible);

    for k = 1:n_cols
        ax = nexttile(layout, k);
        data.axes(k) = ax;
        setup_feasible_axis(ax, data.fixed_limits);
        data.items{k} = create_feasible_axis_items(ax);
        xlabel(ax, '$[v]_1$', 'Interpreter', 'latex', 'FontSize', 12);
        ylabel(ax, '$[v]_2$', 'Interpreter', 'latex', 'FontSize', 12);
        if k == n_cols
            title(ax, '$t=0\,\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 13);
        end
    end

    set(fig_handle, 'UserData', data);
end

function setup_feasible_axis(ax, limits)
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, limits(1:2));
    ylim(ax, limits(3:4));
    set(ax, 'FontSize', 11);
    try
        set(ax, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
    catch
    end
end

function limits = feasible_axis_limits(feasible_config)
    limits = [feasible_config.xlim(:).', feasible_config.ylim(:).'];
end

function items = create_feasible_axis_items(ax)
    p = scene_palette();
    visibility = 'off';

    items.original = patch(ax, nan, nan, p.originalFeasibleFace, ...
        'FaceAlpha', 0.30, ...
        'EdgeColor', p.originalFeasibleEdge, ...
        'LineStyle', '--', ...
        'LineWidth', 1.1, ...
        'DisplayName', '$\mathcal{V}_0$', ...
        'HandleVisibility', visibility);
    items.reshaped = patch(ax, nan, nan, p.feasibleFace, ...
        'FaceAlpha', 0.40, ...
        'EdgeColor', p.feasibleEdge, ...
        'LineWidth', 1.4, ...
        'DisplayName', '$\mathcal{V}_*(\mathcal{F})$', ...
        'HandleVisibility', visibility);
    items.v_c = quiver(ax, 0, 0, 0, 0, 0, ...
        'Color', p.nominal, ...
        'LineStyle', ':', ...
        'LineWidth', 1.9, ...
        'MaxHeadSize', 1.0, ...
        'DisplayName', '$v_c$', ...
        'HandleVisibility', visibility);
    items.v_ref = quiver(ax, 0, 0, 0, 0, 0, ...
        'Color', p.safe, ...
        'LineStyle', '-', ...
        'LineWidth', 2.8, ...
        'MaxHeadSize', 1.05, ...
        'DisplayName', '$v_*$', ...
        'HandleVisibility', visibility);
end

function update_feasible_figure(fig_handle, safety)
    if ~isgraphics(fig_handle, 'figure')
        return;
    end
    data = get(fig_handle, 'UserData');
    if ~isstruct(data) || ~isfield(data, 'axes') || ~isfield(data, 'items')
        return;
    end

    live_ax = data.axes(data.live_index);
    if ~isgraphics(live_ax)
        return;
    end
    if ~isfield(data, 'fixed_limits')
        data.fixed_limits = [-3, 3, -3, 3];
    end

    limits = update_feasible_axis(data.items{data.live_index}, safety, data.fixed_limits);
    if isfield(safety, 't')
        title(live_ax, sprintf('$t=%.0f\\,\\mathrm{s}$', safety.t), ...
            'Interpreter', 'latex', 'FontSize', 13);
    end
    data.live_limits = limits;
    data.live_safety = compact_feasible_safety(safety);
    set(fig_handle, 'UserData', data);
end

function record_feasible_snapshot(fig_handle, safety)
    if ~isgraphics(fig_handle, 'figure')
        return;
    end
    data = get(fig_handle, 'UserData');
    if ~isstruct(data) || ~isfield(data, 'history')
        return;
    end

    snapshot.safety = compact_feasible_safety(safety);
    if isfield(data, 'fixed_limits')
        snapshot.limits = data.fixed_limits;
    elseif isfield(data, 'live_limits')
        snapshot.limits = data.live_limits;
    else
        [~, ~, limits] = feasible_vertices_for_plot(snapshot.safety);
        snapshot.limits = limits;
    end

    data.history{end+1} = snapshot;
    while numel(data.history) > data.snapshot_count
        data.history(1) = [];
    end
    data = redraw_feasible_snapshots(data);
    set(fig_handle, 'UserData', data);
end

function data = redraw_feasible_snapshots(data)
    for k = 1:data.snapshot_count
        ax = data.axes(k);
        items = data.items{k};
        if k <= numel(data.history)
            snapshot = data.history{k};
            update_feasible_axis(items, snapshot.safety, snapshot.limits);
            if isfield(snapshot.safety, 't')
                title(ax, sprintf('$t=%.0f\\,\\mathrm{s}$', snapshot.safety.t), ...
                    'Interpreter', 'latex', 'FontSize', 12);
            else
                title(ax, '');
            end
        else
            clear_feasible_axis(items);
            title(ax, '');
        end
    end
end

function limits = update_feasible_axis(items, safety, limits)
    if nargin < 3 || isempty(limits)
        [original_vertices, reshaped_vertices, limits] = feasible_vertices_for_plot(safety);
    else
        [original_vertices, reshaped_vertices] = feasible_vertices_for_plot(safety, limits);
    end

    set_patch_vertices(items.original, original_vertices);
    set_patch_vertices(items.reshaped, reshaped_vertices);
    set_quiver_vector(items.v_c, safety.v_c);
    set_quiver_vector(items.v_ref, safety.v_ref);
end

function clear_feasible_axis(items)
    set(items.original, 'XData', nan, 'YData', nan);
    set(items.reshaped, 'XData', nan, 'YData', nan);
    set_quiver_vector(items.v_c, [nan; nan]);
    set_quiver_vector(items.v_ref, [nan; nan]);
end

function safety = compact_feasible_safety(safety_in)
    names = {'A_L', 'b_L', 'qp_b', 'original_A', 'original_b', 'feasible_vertices', 'v_c', 'v_ref', 't'};
    safety = struct();
    for k = 1:numel(names)
        if isfield(safety_in, names{k})
            safety.(names{k}) = safety_in.(names{k});
        end
    end
end

function [original_vertices, reshaped_vertices, limits] = feasible_vertices_for_plot(safety, plot_limits)
    if nargin < 2
        plot_limits = [];
    end
    original_vertices = zeros(2, 0);
    reshaped_vertices = zeros(2, 0);
    if isfield(safety, 'original_A') && isfield(safety, 'original_b')
        original_vertices = feasible_vertices_2d_local(safety.original_A, safety.original_b(:), plot_limits);
    elseif isfield(safety, 'A_L') && isfield(safety, 'b_L')
        original_vertices = feasible_vertices_2d_local(safety.A_L, safety.b_L(:), plot_limits);
    end
    if isfield(safety, 'A_L') && isfield(safety, 'qp_b')
        reshaped_vertices = feasible_vertices_2d_local(safety.A_L, safety.qp_b(:), plot_limits);
    elseif isfield(safety, 'feasible_vertices')
        reshaped_vertices = safety.feasible_vertices;
    end

    values = [original_vertices(:); reshaped_vertices(:)];
    if isfield(safety, 'v_c')
        values = [values; safety.v_c(:)];
    end
    if isfield(safety, 'v_ref')
        values = [values; safety.v_ref(:)];
    end
    values = values(isfinite(values));
    lim = max([0.5; abs(values)])*1.25;
    limits = [-lim, lim, -lim, lim];
end

function vertices = feasible_vertices_2d_local(A, b, plot_limits)
    vertices = zeros(2, 0);
    if nargin < 3
        plot_limits = [];
    end
    if isempty(A)
        A = zeros(0, 2);
        b = zeros(0, 1);
    end
    if size(A, 2) ~= 2 || size(A, 1) ~= numel(b)
        return;
    end
    if ~isempty(plot_limits) && numel(plot_limits) == 4
        x_limits = plot_limits(1:2);
        y_limits = plot_limits(3:4);
        A = [A; 1, 0; -1, 0; 0, 1; 0, -1];
        b = [b(:); x_limits(2); -x_limits(1); y_limits(2); -y_limits(1)];
    else
        b = b(:);
    end
    n_constraints = size(A, 1);
    for ii = 1:n_constraints-1
        for jj = ii+1:n_constraints
            M = [A(ii, :); A(jj, :)];
            if abs(det(M)) <= 1e-10
                continue;
            end
            candidate = M\[b(ii); b(jj)];
            if all(A*candidate <= b + 1e-8)
                vertices(:, end+1) = candidate; %#ok<AGROW>
            end
        end
    end
    if isempty(vertices)
        return;
    end
    vertices = unique(round(vertices'*1e10)/1e10, 'rows')';
    center = mean(vertices, 2);
    [~, order] = sort(atan2(vertices(2, :) - center(2), vertices(1, :) - center(1)));
    vertices = vertices(:, order);
end

function set_patch_vertices(h, vertices)
    if isempty(vertices)
        set(h, 'XData', nan, 'YData', nan);
    else
        set(h, 'XData', [vertices(1, :), vertices(1, 1)], ...
               'YData', [vertices(2, :), vertices(2, 1)]);
    end
end

function set_quiver_vector(h, v)
    set(h, 'UData', v(1), 'VData', v(2));
end

function [fig_handle, line_handles] = create_time_figure(fig_name, position, labels, specs)
    if nargin < 4 || isempty(specs)
        specs = component_line_specs(numel(labels), 'mixed');
    end
    fig_handle = figure('Name', fig_name, 'NumberTitle', 'off');
    set(fig_handle, 'Position', position, 'Color', 'w');
    ax = axes('Parent', fig_handle);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    set(ax, 'FontSize', 16);

    line_handles = cell(numel(labels), 1);
    for k = 1:numel(labels)
        line_args = {'LineWidth', specs(k).LineWidth, ...
            'Color', specs(k).Color, ...
            'LineStyle', specs(k).LineStyle, ...
            'DisplayName', labels{k}};
        if isfield(specs, 'Marker')
            line_args = [line_args, {'Marker', specs(k).Marker}]; %#ok<AGROW>
        end
        if isfield(specs, 'MarkerSize')
            line_args = [line_args, {'MarkerSize', specs(k).MarkerSize}]; %#ok<AGROW>
        end
        line_handles{k} = animatedline(ax, line_args{:});
    end
    xlabel(ax, 'Time (s)', 'FontSize', 16);
    legend(ax, 'show', 'Interpreter', 'latex', 'FontSize', 16, 'Location', 'best');
    hold(ax, 'off');
end

function add_distance_guides(fig_handle, duration, D, Ds)
    ax = findobj(fig_handle, 'Type', 'axes');
    ax = ax(1);
    hold(ax, 'on');
    hBand = patch(ax, [0, duration, duration, 0], [0, 0, D, D], [1.0, 0.35, 0.25], ...
        'FaceAlpha', 0.16, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
    plot(ax, [0, duration], [Ds, Ds], '--', ...
        'Color', [0.15, 0.15, 0.15], ...
        'LineWidth', 1.6, ...
        'HandleVisibility', 'off');
    xlim(ax, [0, duration]);
    ylim(ax, [0, 3]);
    uistack(hBand, 'bottom');
    hold(ax, 'off');
end

function specs = distance_line_specs(num_obstacles)
    colors = component_colors(num_obstacles);
    styles = {'-', '--', ':', '-.'};
    specs = repmat(struct('Color', [0 0 0], 'LineStyle', '-', 'LineWidth', 1.5), num_obstacles, 1);
    for k = 1:num_obstacles
        specs(k).Color = colors(k, :);
        specs(k).LineStyle = styles{mod(k-1, numel(styles)) + 1};
        specs(k).LineWidth = 1.6;
    end
end

function specs = velocity_line_specs(n_y)
    colors = component_colors(n_y);
    specs = repmat(struct('Color', [0 0 0], 'LineStyle', '-', 'LineWidth', 1.5), 2*n_y, 1);
    for k = 1:n_y
        specs(k).Color = colors(k, :);
        specs(k).LineStyle = '--';
        specs(k+n_y).Color = colors(k, :);
        specs(k+n_y).LineStyle = '-';
    end
end

function specs = component_line_specs(n, mode)
    colors = component_colors(n);
    styles = {'-', '--', ':', '-.'};
    specs = repmat(struct('Color', [0 0 0], 'LineStyle', '-', 'LineWidth', 1.5), n, 1);
    for k = 1:n
        specs(k).Color = colors(k, :);
        if strcmp(mode, 'solid')
            specs(k).LineStyle = '-';
        else
            specs(k).LineStyle = styles{mod(k-1, numel(styles)) + 1};
        end
    end
end

function colors = component_colors(n)
    base_colors = [0.0000 0.4470 0.7410;
                   0.8500 0.3250 0.0980;
                   0.4660 0.6740 0.1880;
                   0.4940 0.1840 0.5560;
                   0.9290 0.6940 0.1250;
                   0.3010 0.7450 0.9330;
                   0.6350 0.0780 0.1840];
    colors = base_colors(mod(0:n-1, size(base_colors, 1)) + 1, :);
end

function append_time_points(line_handles, t, values)
    for k = 1:numel(line_handles)
        addpoints(line_handles{k}, t, values(k));
    end
end

function handles = draw_obstacles(circle_center, circle_radius, triangle_vertices)
    p = scene_palette();
    handles = gobjects(3, 1);
    theta = linspace(0, 2*pi, 64);
    circle_boundary = circle_center + circle_radius*[cos(theta); sin(theta)];
    handles(1) = patch('XData', circle_boundary(1, :), ...
        'YData', circle_boundary(2, :), ...
        'FaceColor', p.dynamicFace, 'EdgeColor', p.dynamicEdge, ...
        'FaceAlpha', 1.0, 'LineWidth', 1.2);
    for k = 1:2
        vertices = triangle_vertices(:, :, k);
        vertices = [vertices, vertices(:, 1)]; %#ok<AGROW>
        handles(k+1) = patch('XData', vertices(1, :), ...
            'YData', vertices(2, :), ...
            'FaceColor', p.obstacleFace, 'EdgeColor', p.obstacleEdge, ...
            'FaceAlpha', 1.0, 'LineWidth', 1.2);
    end
end

function update_obstacles(handles, circle_center, circle_radius, triangle_vertices)
    theta = linspace(0, 2*pi, 64);
    circle_boundary = circle_center + circle_radius*[cos(theta); sin(theta)];
    set(handles(1), 'XData', circle_boundary(1, :), ...
        'YData', circle_boundary(2, :));
    for k = 1:2
        vertices = triangle_vertices(:, :, k);
        vertices = [vertices, vertices(:, 1)]; %#ok<AGROW>
        set(handles(k+1), 'XData', vertices(1, :), ...
            'YData', vertices(2, :));
    end
end

function h = draw_trajectory(y)
    p = scene_palette();
    if isempty(y)
        h = plot(nan, nan, '-', 'Color', p.vehicle, 'LineWidth', 2.0);
    else
        h = plot(y(1), y(2), '-', 'Color', p.vehicle, 'LineWidth', 2.0);
    end
end

function update_trajectory(h, y)
    if isempty(y)
        return;
    end
    xdata = get(h, 'XData');
    ydata = get(h, 'YData');
    xdata(end+1) = y(1);
    ydata(end+1) = y(2);
    set(h, 'XData', xdata, 'YData', ydata);
end

function [hD, hDs] = draw_vehicle_circles(y, D, Ds)
    p = scene_palette();
    if isempty(y) || isempty(D) || isempty(Ds)
        hD = plot(nan, nan, 'k-', 'LineWidth', 1.0);
        hDs = plot(nan, nan, 'k--', 'LineWidth', 1.0);
        return;
    end
    theta = linspace(0, 2*pi, 64);
    circleD = [D*cos(theta); D*sin(theta)] + y;
    circleDs = [Ds*cos(theta); Ds*sin(theta)] + y;
    hD = patch('XData', circleD(1,:), 'YData', circleD(2,:), ...
               'FaceColor', p.vehicle, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    hDs = plot(circleDs(1,:), circleDs(2,:), '--', 'Color', p.safety, 'LineWidth', 1.0);
end

function update_vehicle_circles(hD, hDs, y, D, Ds)
    if isempty(y) || isempty(D) || isempty(Ds)
        return;
    end
    theta = linspace(0, 2*pi, 64);
    circleD = [D*cos(theta); D*sin(theta)] + y;
    circleDs = [Ds*cos(theta); Ds*sin(theta)] + y;
    set(hD, 'XData', circleD(1,:), 'YData', circleD(2,:));
    set(hDs, 'XData', circleDs(1,:), 'YData', circleDs(2,:));
end


function controller_output = controller(y, d, nabla_d, z, t, lidar, config, y_observer)
if nargin < 8
    y_observer = y;
end

persistent L K T cached_gamma;
gamma = config.regulation.gamma;
if isempty(L) || isempty(cached_gamma) || cached_gamma ~= gamma
    n_eta = config.plant.n_eta;
    n_y = config.plant.n_y;
    L = [-1.0, -0.5; -0.5, 1.0];
    r = zeros(n_y, 1);
    for i = 1:n_y
        for kk = 0:n_eta
            if norm(config.plant.C_eta(i, :)*config.plant.A_eta^kk*config.plant.B_eta) > 1e-10
                break;
            end
        end
        r(i) = kk+1;
    end

    r_bar = max(r);
    lambda = -r_bar/gamma;
    k = zeros(n_y, n_eta);
    for i = 1:n_y
        for q = 1:r(i)
            k(i, q) = factorial(r(i))/(factorial(q-1)*factorial(r(i)-q+1))*abs(lambda)^(r(i)-q+1);
        end
    end

    E_eta = zeros(n_y, size(config.plant.B_eta, 2));
    F_eta = zeros(n_y, size(config.plant.A_eta, 2));
    for i = 1:n_y
        E_eta(i, :) = config.plant.C_eta(i, :)*config.plant.A_eta^(r(i)-1)*config.plant.B_eta;
        F_eta(i, :) = config.plant.C_eta(i, :)*config.plant.A_eta^r(i);
        for q = 1:r(i)
            F_eta(i, :) = F_eta(i, :) + k(i, q)*config.plant.C_eta(i, :)*config.plant.A_eta^(q-1);
        end
    end
    if rank(E_eta) < n_y
        error('E_eta is not full row rank. Cannot compute controller gains K and T.');
    end
    K = -E_eta'/(E_eta*E_eta')*F_eta;
    T = E_eta'/(E_eta*E_eta')*diag(k(:, 1));
    cached_gamma = gamma;
end

v_c = config.v_c;
[v_ref, safety_info] = safety_critical_control_law(y, t, v_c, config.plant.Ds, d, nabla_d, lidar, config);
eta_hat = z+L*y_observer;
u = K*eta_hat+T*v_ref;
controller_output.u = u;
controller_output.eta_hat = eta_hat;
controller_output.v_ref = v_ref;
controller_output.v_c = v_c;
controller_output.L = L;
controller_output.K = K;
controller_output.T = T;
controller_output.safety = safety_info;
end


function [v_ref, info] = safety_critical_control_law(y, t, v_c, D_s, d, nabla_d, lidar, config)
p = safety_parameters(config);
p.D_s = D_s;
persistent A_L opt c_A cached_N_L cached_n_y;
if isempty(A_L) || isempty(cached_N_L) || cached_N_L ~= p.N_L || cached_n_y ~= numel(y)
    [A_L, c_A] = gen_A_L(p.N_L, numel(y));
    opt = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set', 'MaxIterations', 1000);
    cached_N_L = p.N_L;
    cached_n_y = numel(y);
end
if isempty(lidar)
    error('The safety-critical controller requires a LiDAR scan.');
end

[rho_s, min_idx] = gen_rho_s_from_feedback(lidar, p);
r_L = regularized_distance(lidar, A_L', c_A, p.k_L);
r_L_scan = regularized_distance(lidar, lidar.l, c_A, p.k_L);
b_L = max(-A_L*rho_s+p.k_alpha*(r_L(:)-D_s), 0);
qp_b = b_L+A_L*rho_s;
[original_A, original_b] = original_feasible_constraints(d, nabla_d, D_s, p.k_alpha, numel(y));
[v_ref, ~, exitflag] = quadprog(eye(numel(y)), -v_c, A_L, qp_b, [], [], [], [], rho_s, opt);
if exitflag <= 0 || any(~isfinite(v_ref))
    error('The safety-critical QP failed at t = %.6g (exitflag = %d).', t, exitflag);
end

info.A_L = A_L;
info.c_A = c_A;
info.b_L = b_L;
info.qp_b = qp_b;
info.original_A = original_A;
info.original_b = original_b;
info.rho_s = rho_s;
info.r_L = r_L(:);
info.r_L_scan = r_L_scan(:)';
info.v_c = v_c;
info.v_ref = v_ref;
info.exitflag = exitflag;
info.feasible_vertices = feasible_vertices_2d(A_L, qp_b);
info.lidar = lidar;
info.lidar.min_idx = min_idx;
info.t = t;
end


function params = safety_parameters(config)
params.N_L = config.safety.N_L;
params.k_alpha = config.safety.k_alpha;
params.k_L = config.safety.k_L;
end


function [A_L, c_A] = gen_A_L(N, n_y)
if n_y ~= 2
    error('The set-valued LiDAR controller is implemented for n_y = 2.');
end
theta = linspace(0, 2*pi, N+1)';
A_L = [cos(theta(1:end-1)), sin(theta(1:end-1))];
c_A = cos(2*pi/N);
end


function [rho_s, min_idx] = gen_rho_s_from_feedback(lidar, params)
% Practical construction of rho_s from the nearest LiDAR distance and direction.
[r_min, min_idx] = min(lidar.r);
l_min = lidar.l(:, min_idx);
rho_s = params.k_alpha*l_min*min(r_min-params.D_s, 0);
end


function r_L = regularized_distance(lidar, query_dirs, c_A, k_L)
persistent scan_l scan_c_A scan_k_L scan_penalty;
persistent query_l query_dirs_cached query_c_A query_k_L query_penalty;
if isequal(query_dirs, lidar.l)
    if isempty(scan_penalty) || scan_c_A ~= c_A || scan_k_L ~= k_L || ~isequal(scan_l, lidar.l)
        scan_penalty = regularization_penalty(lidar.l, lidar.l, c_A, k_L);
        scan_l = lidar.l;
        scan_c_A = c_A;
        scan_k_L = k_L;
    end
    penalty = scan_penalty;
else
    if isempty(query_penalty) || query_c_A ~= c_A || query_k_L ~= k_L || ~isequal(query_l, lidar.l) || ~isequal(query_dirs_cached, query_dirs)
        query_penalty = regularization_penalty(query_dirs, lidar.l, c_A, k_L);
        query_l = lidar.l;
        query_dirs_cached = query_dirs;
        query_c_A = c_A;
        query_k_L = k_L;
    end
    penalty = query_penalty;
end
r_L = min(penalty+lidar.r, [], 2);
end


function penalty = regularization_penalty(query_dirs, lidar_dirs, c_A, k_L)
penalty = zeros(size(query_dirs, 2), size(lidar_dirs, 2));
for jj = 1:size(query_dirs, 2)
    alignment = query_dirs(:, jj)'*lidar_dirs;
    penalty(jj, :) = k_L*max(c_A-alignment, 0);
end
end


function [A_original, b_original] = original_feasible_constraints(d, nabla_d, D_s, k_alpha, n_y)
if isempty(d) || isempty(nabla_d)
    A_original = zeros(0, n_y);
    b_original = zeros(0, 1);
    return;
end
A_original = -nabla_d.';
b_original = k_alpha*(d(:)-D_s);
end


function vertices = feasible_vertices_2d(A, b)
vertices = zeros(2, 0);
for ii = 1:size(A, 1)-1
    for jj = ii+1:size(A, 1)
        M = [A(ii, :); A(jj, :)];
        if abs(det(M)) <= 1e-10
            continue;
        end
        candidate = M\[b(ii); b(jj)];
        if all(A*candidate <= b+1e-8)
            vertices(:, end+1) = candidate; %#ok<AGROW>
        end
    end
end
if isempty(vertices)
    return;
end
vertices = unique(round(vertices'*1e10)/1e10, 'rows')';
center = mean(vertices, 2);
[~, order] = sort(atan2(vertices(2, :)-center(2), vertices(1, :)-center(1)));
vertices = vertices(:, order);
end


function lidar = lidar_feedback(y, circle_center, circle_radius, triangle_vertices, lidar_config, range_error_bound)
if nargin < 6
    range_error_bound = 0;
end
persistent cached_num_beams cached_theta cached_directions;
if isempty(cached_num_beams) || cached_num_beams ~= lidar_config.num_beams
    cached_theta = linspace(-pi, pi, lidar_config.num_beams+1);
    cached_theta = cached_theta(1:end-1);
    cached_directions = [cos(cached_theta); sin(cached_theta)];
    cached_num_beams = lidar_config.num_beams;
end
ranges = lidar_ranges_fast(y, cached_directions, circle_center, circle_radius, triangle_vertices, lidar_config.D_B);
if range_error_bound > 0
    ranges = min(max(ranges+range_error_bound*(2*rand(size(ranges))-1), 0), lidar_config.D_B);
end
lidar.theta = cached_theta;
lidar.l = cached_directions;
lidar.r = ranges;
lidar.points = y+cached_directions.*ranges;
lidar.hit = ranges < lidar_config.D_B-1e-8;
lidar.D_B = lidar_config.D_B;
end


function ranges = lidar_ranges_fast(y, directions, circle_center, circle_radius, triangle_vertices, D_B)
obstacle_type = int32([1, 2, 2]);
circle_centers = zeros(2, 3);
circle_centers(:, 1) = circle_center;
circle_radii = [circle_radius, 0, 0];
polygon_vertices = zeros(2, 64, 3);
polygon_vertices(:, 1:3, 2:3) = triangle_vertices;
polygon_counts = int32([0, 3, 3]);
if exist('lidar_ranges_kernel_mex', 'file') == 3
    ranges = lidar_ranges_kernel_mex(y, directions, obstacle_type, circle_centers, circle_radii, polygon_vertices, polygon_counts, D_B);
else
    ranges = lidar_ranges_kernel(y, directions, obstacle_type, circle_centers, circle_radii, polygon_vertices, polygon_counts, D_B);
end
end


function ranges = lidar_ranges_kernel(y, directions, obstacle_type, circle_centers, circle_radii, polygon_vertices, polygon_counts, D_B)
n_beams = size(directions, 2);
ranges = D_B*ones(1, n_beams);
tol = 1e-10;
for q = 1:n_beams
    lx = directions(1, q);
    ly = directions(2, q);
    best = D_B;
    for k = 1:numel(obstacle_type)
        if obstacle_type(k) == 1
            mx = y(1)-circle_centers(1, k);
            my = y(2)-circle_centers(2, k);
            b = lx*mx+ly*my;
            c = mx*mx+my*my-circle_radii(k)*circle_radii(k);
            discriminant = b*b-c;
            if discriminant >= 0
                root_term = sqrt(discriminant);
                tau_1 = -b-root_term;
                tau_2 = -b+root_term;
                if tau_1 >= 0
                    best = min(best, tau_1);
                elseif tau_2 >= 0
                    best = min(best, tau_2);
                end
            end
        elseif obstacle_type(k) == 2
            n_vertices = double(polygon_counts(k));
            for j = 1:n_vertices
                j_next = j+1;
                if j == n_vertices
                    j_next = 1;
                end
                p1x = polygon_vertices(1, j, k);
                p1y = polygon_vertices(2, j, k);
                sx = polygon_vertices(1, j_next, k)-p1x;
                sy = polygon_vertices(2, j_next, k)-p1y;
                denominator = lx*sy-ly*sx;
                if abs(denominator) > tol
                    dx = p1x-y(1);
                    dy = p1y-y(2);
                    tau = (dx*sy-dy*sx)/denominator;
                    mu = (dx*ly-dy*lx)/denominator;
                    if tau >= 0 && mu >= -tol && mu <= 1+tol
                        best = min(best, tau);
                    end
                end
            end
        end
    end
    ranges(q) = min(best, D_B);
end
end


function [circle_center, circle_radius, triangle_vertices] = obstacle_geometry(scene, t)
obstacle_index = 1:3;
delta = [scene.obstacle_speed*t-scene.obstacle_horizontal_amplitude*sin(2*pi*t/scene.obstacle_period+2*pi*obstacle_index/3); zeros(1, 3)];
circle_center = scene.circle_center0+delta(:, 1);
circle_radius = scene.circle_radius;
triangle_vertices = scene.triangle_vertices0;
for k = 1:2
    triangle_vertices(:, :, k) = triangle_vertices(:, :, k)+delta(:, k+1);
end
end


function [d, nabla_d] = obstacle_distance_gradient(y, circle_center, circle_radius, triangle_vertices)
d = zeros(1, 3);
nabla_d = zeros(2, 3);
circle_delta = y-circle_center;
circle_norm = norm(circle_delta);
d(1) = circle_norm-circle_radius;
if circle_norm > 0
    nabla_d(:, 1) = circle_delta/circle_norm;
end
for k = 1:2
    [d(k+1), nabla_d(:, k+1)] = point_triangle_distance_gradient(y, triangle_vertices(:, :, k));
end
end


function [distance, gradient] = point_triangle_distance_gradient(point, vertices)
best_distance = inf;
best_normal = [0; 0];
for k = 1:3
    a = vertices(:, k);
    b = vertices(:, mod(k, 3)+1);
    edge = b-a;
    tau = dot(point-a, edge)/max(dot(edge, edge), eps);
    tau = max(0, min(1, tau));
    closest = a+tau*edge;
    difference = point-closest;
    candidate_distance = norm(difference);
    if candidate_distance < best_distance
        best_distance = candidate_distance;
        if candidate_distance > 0
            best_normal = difference/candidate_distance;
        else
            best_normal = [0; 0];
        end
    end
end
inside = inpolygon(point(1), point(2), vertices(1, :), vertices(2, :));
if inside
    distance = -best_distance;
    gradient = -best_normal;
else
    distance = best_distance;
    gradient = best_normal;
end
end


function export_simulation_figures(H)
drawnow;
figure_specs = {H.obstacle_fig, 'simulation_01_scene.jpg'; H.distance_fig, 'simulation_02_distance.jpg'; H.input_fig, 'simulation_03_control_input.jpg'; H.velocity_fig, 'simulation_04_velocity.jpg'; H.eta_error_fig, 'simulation_05_observer_error.jpg'; H.feasible_fig, 'simulation_06_safety_feasible_set.jpg'};
for k = 1:size(figure_specs, 1)
    fig_handle = figure_specs{k, 1};
    if ~isgraphics(fig_handle, 'figure')
        continue;
    end
    file_path = fullfile(pwd, figure_specs{k, 2});
    set(fig_handle, 'Color', 'w');
    try
        exportgraphics(fig_handle, file_path, 'Resolution', 300);
    catch
        print(fig_handle, file_path, '-djpeg', '-r300');
    end
end
end
