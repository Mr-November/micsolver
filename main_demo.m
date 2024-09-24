clear;
clc;
close all;

% Robot section lengths.
L1 = 1; L2 = 1; L3 = 1;

% Self-defined end pose.
% This example provides an example with four multiple solutions.
alpha = 15*pi/16; omega = [0.48; sqrt(3)/10; -0.86];
q = [cos(alpha/2); sin(alpha/2)*omega];
r = [-0.4; 1.1; 0.8];

% Solve the problem.
tol = 1e-2; exitfcn = @(e, x) e < tol;
[sol, ~, ~] = micsolverd(L1, L2, L3, q, r, 0.01, [5, 5], tol, 4, exitfcn, 'plot', 'plot');

% Plot the results.
for eta = 1: size(sol, 2)
    fh = figure(10+eta);
    circles3(fh, L1, L2, L3, sol(:, 2), 'k--');
    circles3(fh, L1, L2, L3, sol(:, eta), 'k-');

    % Set the azimuth and elevation angles of the sight.
    view(119, 20);

    % Set axis limits.
    xlim([-1.6, 0.8]);
    xticks([-1, 0, 1]);
    ylim([-0.8, 1.6]);
    yticks([0, 1]);
    zlim([-0.7, 1.7]);
    zticks([0, 1]);
end

