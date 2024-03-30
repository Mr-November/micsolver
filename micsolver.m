function [nos, noi] = micsolver(L1, L2, L3, q, r, tol, is_a_sol)
%MICSOLVER Multi-solution solver for the inverse kinematics of 3-section
%          constant-curvature robots.
%   [NOS, NOI] = MICSOLVER(L1, L2, L3, Q, R, TOL, IS_A_SOL) returns the
%   result of solving the 3-section inverse kinematics problem. The
%   function uses preset resolutions or numerical methods to address the
%   inverse kinematics problem. The function exits after one solution is
%   found.
%
%   Input parameters
%      L1, L2, L3      section length
%      Q, R            desired end rotation and translation
%      TOL             error tolerance
%      IS_A_SOL        function handle
%                      This function is used to judge if the given parameter is a
%                      solution to the inverse kinematics problem. The function has
%                      two inputs (ERR, XI) and one output in boolean type.
%
%   Output parameters
%      NOS             number of solutions
%      NOI             number of iterations in numerical correction
%
%   Example
%      L1 = 1; L2 = 1; L3 = 1;
%      xi = arc2xi(L1, L2, L3, pi.*[1,2,1,2,1,2].*rand(1, 6));
%      T = get_end(L1, L2, L3, xi);
%      q = rot2q(T(1:3, 1:3));
%      r = T(1:3, 4);
%      tol = 1e-2; fun = @(e, x) e < tol;
%      tic;
%      [nos, ~] = micsolver(L1, L2, L3, q, r, tol, fun);
%      rt = toc*1000;
%      if nos
%         fprintf('A solution is found in %.2f ms.\n', rt);
%      end

noi = 0; % Number of iterations.
nos = 0; % Number of solutions.
solns = micsolver__(L1, L2, L3, q, r, 0.03, [1, 2]);
for soln = solns
    [xi, err, cnt] = revise_newton(L1, L2, L3, q, r, soln2xi(L1, L2, L3, soln), 6, tol, 'noplot');
    noi = noi + cnt;
    if is_a_sol(err, xi) % A solution is found.
        nos = nos + 1;
        break; % Only one solution is needed, use this line.
    end
end
if nos == 0 % Can't find solutions in coarser resolution, use finer resolution.
    solns = micsolver__(L1, L2, L3, q, r, 0.01, [5, 2]);
    for soln = solns
        [xi, err, cnt] = revise_newton(L1, L2, L3, q, r, soln2xi(L1, L2, L3, soln), 6, tol, 'noplot');
        noi = noi + cnt;
        if is_a_sol(err, xi) % A solution is found.
            nos = nos + 1;
            break;
        end
    end
end
if nos == 0 % Can't find solutions in coarser resolution, use finer resolution.
    solns = micsolver__(L1, L2, L3, q, r, 0.001, [5, 5]);
    for soln = solns
        [xi, err, cnt] = revise_newton(L1, L2, L3, q, r, soln2xi(L1, L2, L3, soln), 4, tol, 'noplot');
        noi = noi + cnt;
        if is_a_sol(err, xi) % A solution is found.
            nos = nos + 1;
            break;
        end
    end
end
if nos == 0 % Use damped least-squares method.
    for soln = solns
        [xi, err, cnt] = revise_dls(L1, L2, L3, q, r, soln2xi(L1, L2, L3, soln), 200, tol, 'noplot');
        noi = noi + cnt;
        if is_a_sol(err, xi) % A solution is found.
            nos = nos + 1;
            break;
        end
    end
end
end

function solns = micsolver__(L1, L2, L3, q, r, par, noc)
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
end
