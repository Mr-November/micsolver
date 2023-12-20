function [sol, nos, noi] = micsolverd(L1, L2, L3, q, r, par, noc, tol, mstep, is_a_sol, plot_ec, plot_it)
%MICSOLVERD Multi-solution solver (debug) for the inverse kinematics of
%           3-section constant-curvature robots.
%   [SOL, NOS, NOI] = MICSOLVERD(L1, L2, L3, Q, R, ...
%   PAR, NOC, TOL, MSTEP, IS_A_SOL, PLOT_EC, PLOT_IT) returns the result
%   of solving the 3-section inverse kinematics problem.
%
%   Input parameters
%      L1, L2, L3      section length
%      Q, R            desired end rotation and translation
%      PAR             length of the partition
%                      Scalar. The interval [0, 1) is partitioned into subintervals
%                      with length PAR. A smaller PAR means finer resolution. We
%                      recommend PAR = 0.03 for better efficiency and PAR = 0.01 for
%                      more solutions.
%      NOC             number of corrections
%                      A 1-by-2 array. The model parameters hat r_3 and hat r_1 are
%                      corrected for NOC(1) and NOC(2) times after the approximation,
%                      respectively. Large NOC provides closer initial value and more
%                      computations. We recommend NOC = [1, 2] for better efficiency
%                      and NOC = [5, 5] for better estimation.
%      TOL             error tolerance
%      MSTEP           allowed maximum steps of iterations
%      IS_A_SOL        function handle
%                      This function is used to judge if the given parameter is a
%                      solution to the inverse kinematics problem. The function has
%                      two inputs (ERR, XI) and one output in boolean type.
%      PLOT_EC         set 'plot' to visualise the traversal
%      PLOT_IT         set 'plot' to visualise the numerical correction
%
%   Output parameters
%      SOL             solutions
%                      A 6-by-NOS array. Each column is the overall exponential
%                      coordinate XI.
%      NOS             number of solutions
%      NOI             number of iterations in numerical correction
%
%   Example
%      L1 = 1; L2 = 1; L3 = 1;
%      alpha = 15*pi/16; omega = [0.48; sqrt(3)/10; -0.86];
%      q = [cos(alpha/2); sin(alpha/2)*omega];
%      r = [-0.4; 1.1; 0.8];
%      tol = 1e-2; fun = @(e, x) e < tol;
%      [sol, ~, ~] = micsolverd(L1, L2, L3, q, r, ...
%                               0.01, [5, 5], tol, 10, fun, ...
%                               'plot', 'plot');
%      for eta = 1: size(sol, 2)
%         fh = figure();
%         circles3(fh, L1, L2, L3, sol(:, eta), 'k-');
%         view(119, 20);
%      end

noi = 0; % Number of iterations.
nos = 0; % Number of solutions.
solns = micsolver__(L1, L2, L3, q, r, par, noc, plot_ec);
sol = zeros(6, size(solns, 2));
for soln = solns
    [xi, err, cnt] = revise_newton(L1, L2, L3, q, r, soln2xi(L1, L2, L3, soln), mstep, tol, plot_it);
    noi = noi + cnt;
    if is_a_sol(err, xi) % A solution is found.
        nos = nos + 1;
        sol(:, nos) = xi;
    end
end
sol = sol(:, 1:nos);
end



function solns = micsolver__(L1, L2, L3, q, r, par, noc, type)
a = q(1); b = q(2); c = q(3); d = q(4);
B = [d, a, b; -a, d, c; -b, -c, d];
n0 = B.' * r;
n = n0 / norm(n0);
r0 = (1/2 + 1/pi) .* (L3 * d) / norm(n0) .* n; % I prefer gamma = 1/2+1/pi.

norm_r0 = norm(r0);
norm_r01 = sqrt(1 - norm_r0^2);

u = [n(2); -n(1); 0] / norm([n(2); -n(1); 0]);
v = cross(n, u);
P = [u, v, n];

zeta = 0: par: 1;
npar = length(zeta);
err = nan(1, npar);
err_r = nan(1, npar);
err_t = nan(1, npar);
solns = nan(9, npar);
is = nan(1, npar);
nts = 0;
for i = 1: npar
    t = zeta(i);
    r3 = r0 + norm_r01 .* (P * [sin(2*pi*t); cos(2*pi*t); 0]);

    if d ~= 0 % When d==0, the solution is exact, we do not need corrections.
        for cor_idx = 1: noc(1)
            % One-step correction.
            tmp = r3 - (n0.' * r3 - rho(r3(3), L3) * d) / ((n0.' + [0, 0, L3*d*(1/acos(r3(3)) - 1/sqrt(1-r3(3)^2))]) * r0) * r0;
            r3 = tmp / norm(tmp);
        end
    end

    r1 = solve_r1(L1, q, r, r3, noc(2));
    [r2r, r2t] = solve_r2(L1, L3, q, r, r3, r1);

    e_r = get_err(r1, r2r, r3, L1, L2, L3, q, r);
    e_t = get_err(r1, r2t, r3, L1, L2, L3, q, r);

    if e_r < e_t
        r2 = r2r;
        e = e_r;
    else
        r2 = r2t;
        e = e_t;
    end

    if i == 1
        % No operation here.
    elseif i == 2
        nts = nts + 1;
        solns(:, nts) = soln;
        is(nts) = 1;
    else % i > 2
        if e > err(i-1) && err(i-1) <= err(i-2) % Local minimum.
            nts = nts + 1;
            solns(:, nts) = soln;
            is(nts) = i-1;
        end
    end
    soln = [r1; r2; r3];
    err(i) = e;
    err_r(i) = e_r;
    err_t(i) = e_t;
end

if zeta(i) ~= 1
    % Check if the last point is a local minimum.
    if err(1) > err(i) && err(i) <= err(i-1) % Yes, it is.
        nts = nts + 1;
        solns(:, nts) = soln;
        is(nts) = i;
    end
    % Check if the first point is a local minimum.
    if err(2) > err(1) && err(1) <= err(i) % Yes, it is.
        solns = solns(:, 1:nts);
        is = is(1:nts);
    else
        solns = solns(:, 2:nts);
        is = is(2:nts);
    end
else % The last point coincide with the first.
    % Check if the first (also the last) point is a local minimum.
    if err(2) > err(1) && err(1) <= err(i-1) % Yes, it is.
        solns = solns(:, 1:nts);
        is = is(1:nts);
    else
        solns = solns(:, 2:nts);
        is = is(2:nts);
    end
end

ts = zeta(is);
[~, idx] = sort(abs(ts - 0.5));
solns = solns(:, idx);
if isequal(type, 'plot')
    fprintf('Candidate $t$ includes: ');
    disp(ts(idx));
    traverse_plot(norm_r01, zeta, err_r, err_t, err, is);
    traverse_plot2(r0, norm_r01, P, zeta, err_r, err_t, err, is)
end

end

function traverse_plot2(r0, norm_r01, P, zeta, err_r, err_t, err, is)
lwb = 4; lwt = 2; % Line width for error curves.
lwb2 = 3; lwt2 = 2; % Line width for legends.
ms = 26; lwbox = 2; % Box of equilibria.
new_red = [238, 0, 0]/255;
new_blue = [0, 0, 238]/255;

figure(221);
err_scale = -1/max(max(err_r), max(err_t));

radius = 1;
data = [radius.*sin(2*pi*zeta); radius.*cos(2*pi*zeta); 0.*zeta];
data = r0 + norm_r01 .* (P * data);
plot3(data(1, :), data(2, :), data(3, :), 'k-', 'LineWidth', lwt);
hold on;

data = [radius.*sin(2*pi*zeta); radius.*cos(2*pi*zeta); err_r*err_scale];
data = r0 + norm_r01 .* (P * data);
plot3(data(1, :), data(2, :), data(3, :), '--', 'LineWidth', lwt, 'Color', new_red);

data = [radius.*sin(2*pi*zeta); radius.*cos(2*pi*zeta); err_t*err_scale];
data = r0 + norm_r01 .* (P * data);
plot3(data(1, :), data(2, :), data(3, :), '--', 'LineWidth', lwt, 'Color', new_blue);

data = [radius.*sin(2*pi*zeta); radius.*cos(2*pi*zeta); err*err_scale];
data = r0 + norm_r01 .* (P * data);
plot3(data(1, :), data(2, :), data(3, :), 'k--', 'LineWidth', lwb);

for i = is
    t = zeta(i);
    data = [radius.*sin(2*pi*(t)); radius.*cos(2*pi*(t)); err(i)*err_scale];
    data = r0 + norm_r01 .* (P * data);
    plot3(data(1, :), data(2, :), data(3, :), ...
        'ks', 'MarkerSize', 12, 'LineWidth', 2);
end
[X,Y,Z] = sphere(256); surf(X, Y, Z, (Z+1)/2, 'LineStyle', 'none', 'FaceAlpha', 0.7);
colormap(linspace(0.4, 1, 256).'.*ones(256, 3));
xlabel('$c_3$', 'Interpreter', 'latex');
ylabel('$-b_3$', 'Interpreter', 'latex');
zlabel('$a_3$~~~~~~~~', 'Rotation', 0, 'Interpreter', 'latex');
pbaspect([1, 1, 1]);
daspect([1, 1, 1]);
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
zlim([-1.5, 1.5]);
xticks([-1, 0, 1]);
yticks([-1, 0, 1]);
zticks([-1, 0, 1]);
view(-60, 20);
set(gcf, 'Position', [50, 50, 680, 600]);
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
    'LineWidth', 0.7, 'FontName', 'Times New Roman', 'FontSize', 24);
end

function traverse_plot(radius, zeta, err_r, err_t, err, is)
lwb = 4; lwt = 2.5; % Line width for error curves.
lwb2 = 3; lwt2 = 2.5; % Line width for legends.
ms = 26; lwbox = 2.5; % Box of equilibria.
new_red = [238, 0, 0]/255;
new_blue = [0, 0, 238]/255;
% new_blue = [14,65,156]/255;
% new_red = [204,26,26]/255;

figure(220);
err_scale = 1/max(max(err_r), max(err_t));
zoff = sqrt(1-radius^2);
plot3(radius.*sin(2*pi*zeta), radius.*cos(2*pi*zeta), 0.*zeta+zoff, 'k-', 'LineWidth', lwt);
hold on;
plot3(radius.*sin(2*pi*zeta), radius.*cos(2*pi*zeta), err_r*err_scale+zoff, '--', 'LineWidth', lwt, 'Color', new_red);
plot3(radius.*sin(2*pi*zeta), radius.*cos(2*pi*zeta), err_t*err_scale+zoff, '--', 'LineWidth', lwt, 'Color', new_blue);
plot3(radius.*sin(2*pi*zeta), radius.*cos(2*pi*zeta), err*err_scale+zoff, 'k--', 'LineWidth', lwb);
for i = is
    t = zeta(i);
    plot3(radius.*sin(2*pi*(t)), radius.*cos(2*pi*(t)), err(i)*err_scale+zoff, ...
        'ks', 'MarkerSize', 12, 'LineWidth', 2);
end
% [X,Y,Z] = sphere(256); surf(X, Y, Z, (Z+1)/2, 'LineStyle', 'none', 'FaceAlpha', 0.7);
% colormap(linspace(0.4, 1, 256).'.*ones(256, 3));
pbaspect([1, 1, 1]);
daspect([1, 1, 1]);
xlim([-1.2, 1.2]);
ylim([-1.2, 1.2]);
zlim([-0.4, 2]);
xticks([]);
yticks([]);
zticks([]);
set(gcf, 'Position', [50, 50, 680, 600]);
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
    'LineWidth', 0.7, 'FontName', 'Times New Roman', 'FontSize', 24);



figure(219);
plot(zeta, err_r, '--', 'LineWidth', lwt, 'Color', new_red);
hold on;
plot(zeta, err_t, '--', 'LineWidth', lwt, 'Color', new_blue);
plot(zeta, err, 'k--', 'LineWidth', lwb);
for i = is
    t = zeta(i);
    plot(t, err(i), 'ks', 'MarkerSize', ms, 'LineWidth', lwbox);
end
xlabel('$s$', 'Interpreter', 'latex', 'Position', [0.5,-0.2,-1]);
ylabel('Error');
xlim([0, 1]);
xticks(0: 0.2: 1);
rc = plot([2,3], [2,2], '--', 'LineWidth', lwt2, 'Color', new_red);
bc = plot([2,3], [1,1], '--', 'LineWidth', lwt2, 'Color', new_blue);
kc = plot([2,3], [0,0], 'k--', 'LineWidth', lwb2);
legend([rc, bc, kc], ...
    "$~e(\hat{\mbox{\boldmath$r$}}_2^{\prime})~~~$", "$~e(\hat{\mbox{\boldmath$r$}}_2^{\prime\prime})~~~$", "$~e$", ...
    'Interpreter', 'latex', 'Box', 'off', 'Position', [0.58,0.8,0.4,0.16], 'NumColumns', 3, 'EdgeColor', 'none', ...
    'FontName', 'Times New Roman', 'FontSize', 24);
set(gcf, 'Position', [50,50,1000,400]);
set(gca, 'Position', [0.07,0.155,0.88,0.635], 'XColor', 'k', 'YColor', 'k', 'TickLength', [0.003, 0.01], ...
    'Box', 'on', 'LineWidth', 1.2, ...
    'FontName', 'Times New Roman', 'FontSize', 24);

end
