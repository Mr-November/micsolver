clear;
clc;
close all;

% Robot section lengths.
L1 = 1;
L2 = 1;
L3 = 1;

% Generate a path.
smp = 30;
tolerance = 6e-4;
R0 = rot([1;0;0], pi/3);
h = 2.7;
yp = -0.2;
[x, y, z] = generate_path([-0.5; yp; h], R0, [1; 1; 0], smp);
rs = [x; y; z];
qs = nan(4, smp);
for smp_idx = 1: smp
    qs(:, smp_idx) = rot2q(R0);
end

% Solve the problem.
n_success = nan(smp, 1);
t_runtime = nan(smp, 1);
n_iteration = nan(smp, 1);
is_a_sol = @(e, x) e < tolerance;
rvsd_solns = cell(1, length(x));
for smp_idx = 1: smp
    r = rs(:, smp_idx);
    q = qs(:, smp_idx);
    tic;
    [sol, nos, noi] = micsolverd(L1, L2, L3, q, r, 0.03, [5, 5], tolerance, 5, is_a_sol, 'noplot', 'noplot');
    t_runtime(smp_idx) = toc;
    n_success(smp_idx) = nos;
    n_iteration(smp_idx) = noi;

    rvsd_solns{smp_idx} = sol;
    if n_success(smp_idx) == 0
        fprintf('Sample: %d has no solution.\n', smp_idx);
    end
end
fprintf('##### INVERSE KINEMATICS #####\n\n');
fprintf('Runtime (ms): '); disp(1000*sum(t_runtime));

% Energy planning.
tic;
[path, cost] = dp(rvsd_solns, @lossfun2);
runtime_dp = toc;
fprintf('##### ENERGY PLANNING #####\n\n');
fprintf('Number of Cases: '); disp(size(path, 1));
fprintf('Cost: '); disp(cost.');
fprintf('Runtime (ms): '); disp(1000 * runtime_dp);

% Time allocation.
chs = path{1};
xis = nan(smp, 6);
for smp_idx = 1: smp
    sol = rvsd_solns{smp_idx};
    xis(smp_idx, :) = sol(:, chs(smp_idx)).';
end
tic;
ts = allocate_time(xis.');
runtime_alloc = toc;
fprintf('##### TIME ALLOCATION #####\n\n');
fprintf('Runtime (ms): '); disp(1000 * runtime_alloc);

% Visualisation.
c_begin = [238, 0, 255]/255;
c_end = [0, 238, 255]/255;
cmap = [linspace(c_begin(1), c_end(1), smp).', ...
    linspace(c_begin(2), c_end(2), smp).', ...
    linspace(c_begin(3), c_end(3), smp).'];
calpha = linspace(1, 1, smp);

% Case 1.
fh = figure(1);
subplot(1, 2, 1);
chs = path{1};
plot3([-1, 0.8, -1, 0.8], ones(1, 4)*yp, ones(1, 4)*h, '-', 'Color', [ones(1, 3)*0.5, 0.2], 'LineWidth', 10);
hold on;
for smp_idx = ceil(sqrt(1: 140: smp^2))
    colour = [cmap(smp_idx, :), calpha(smp_idx)];
    sol = rvsd_solns{smp_idx};
    xi = sol(:, chs(smp_idx));
    circles3c(fh, L1, L2, L3, xi, '-', [0.1, 0.1, 0.1]);
    for i = 1: size(sol, 2)
        if i ~= chs(smp_idx)
            circles3c(fh, L1, L2, L3, sol(:, i), '-', [0.8, 0.8, 0.8, 0.4]);
        end
    end
    frame(fh, [R0, rs(:, smp_idx); 0, 0, 0, 1], colour(4));
end
pbaspect([1, 1, 1]);
daspect([1, 1, 1]);
xlabel('$x$', 'Interpreter', 'latex');
ylabel('$y$', 'Interpreter', 'latex');
zlabel('$z$~~~~~~~~', 'Rotation', 0, 'Interpreter', 'latex');
view(-52, 15);
xlim([-1.55, 1.35]);
xticks([-1, 0, 1]);
ylim([-1.5, 1.4]);
yticks([-1, 0, 1]);
zlim([-0.1, 3.1]);
zticks([0, 1, 2, 3]);
set(gca, 'LineWidth', 1.2, 'TickLength', [0.006, 0.01], ...
    'Box', 'off', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'Color', 'w', ...
    'FontName', 'Times New Roman', 'FontSize', 16);

subplot(1, 2, 2);
act_len = xi2len(xis.');
intv = ts(1): 0.01: ts(end);
for i = 1: 9
    plot(intv, spline(ts, act_len(i,:), intv), '-', 'LineWidth', 2);
    hold on;
end
xlim([ts(1), ts(end)]);
xlabel('Time (s)');
ylim([-50, 50]);
ylabel('Actuation (mm)');
set(gca, 'LineWidth', 1.2, 'TickLength', [0.006, 0.01], ...
    'Box', 'off', 'XColor', 'k', 'YColor', 'k', 'Color', 'w', ...
    'FontName', 'Times New Roman', 'FontSize', 16);

set(gcf, 'Position', [100, 100, 800, 300]);

% Case 2.
fh = figure(2);
subplot(1, 2, 1);
chs = path{2};
plot3([-1, 0.8, -1, 0.8], ones(1, 4)*yp, ones(1, 4)*h, '-', 'Color', [ones(1, 3)*0.5, 0.2], 'LineWidth', 10);
hold on;
for smp_idx = ceil(sqrt(1: 140: smp^2))
    colour = [cmap(smp_idx, :), calpha(smp_idx)];
    sol = rvsd_solns{smp_idx};
    xi = sol(:, chs(smp_idx));
    circles3c(fh, L1, L2, L3, xi, '-', [0.1, 0.1, 0.1]);
    for i = 1: size(sol, 2)
        if i ~= chs(smp_idx)
            circles3c(fh, L1, L2, L3, sol(:, i), '-', [0.8, 0.8, 0.8, 0.4]);
        end
    end
    frame(fh, [R0, rs(:, smp_idx); 0, 0, 0, 1], colour(4));
end
pbaspect([1, 1, 1]);
daspect([1, 1, 1]);
xlabel('$x$', 'Interpreter', 'latex');
ylabel('$y$', 'Interpreter', 'latex');
zlabel('$z$~~~~~~~~', 'Rotation', 0, 'Interpreter', 'latex');
view(-52, 15);
xlim([-1.55, 1.35]);
xticks([-1, 0, 1]);
ylim([-1.5, 1.4]);
yticks([-1, 0, 1]);
zlim([-0.1, 3.1]);
zticks([0, 1, 2, 3]);
set(gca, 'LineWidth', 1.2, 'TickLength', [0.006, 0.01], ...
    'Box', 'off', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'Color', 'w', ...
    'FontName', 'Times New Roman', 'FontSize', 16);

subplot(1, 2, 2);
chs = path{2};
xis = nan(smp, 6);
for smp_idx = 1: smp
    sol = rvsd_solns{smp_idx};
    xis(smp_idx, :) = sol(:, chs(smp_idx)).';
end
ts = allocate_time(xis.');
act_len = xi2len(xis.');
intv = ts(1): 0.01: ts(end);
for i = 1: 9
    plot(intv, spline(ts, act_len(i,:), intv), '-', 'LineWidth', 2);
    hold on;
end
xlim([ts(1), ts(end)]);
xlabel('Time (s)');
ylim([-50, 50]);
ylabel('Actuation (mm)');
set(gca, 'LineWidth', 1.2, 'TickLength', [0.006, 0.01], ...
    'Box', 'off', 'XColor', 'k', 'YColor', 'k', 'Color', 'w', ...
    'FontName', 'Times New Roman', 'FontSize', 16);

set(gcf, 'Position', [200, 300, 800, 300]);

% Loss function for planning.
function loss = lossfun2(xi1, xi2)
loss = norm(xi2len(xi1) - xi2len(xi2))^2 / 2;
end

% Generate a straight line path.
function [x, y, z] = generate_path(centre, orientation, scale, sample)
x0 = [linspace(0, 1, sample); zeros(1, sample); zeros(1, sample)];
x1 = centre + orientation * (scale .* x0);
x = x1(1, :);
y = x1(2, :);
z = x1(3, :);
end

% Rotation matrix from exponential coordiante.
function mat = rot(axis, angle)
mat = expm(up_hat(axis) * angle);
end