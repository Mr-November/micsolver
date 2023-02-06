clear;
clc;
close all;

% Robot section lengths.
L1 = 1; L2 = 1; L3 = 1;

% Self-defined end pose.
% This example provides a heart-shaped multiple solutions.
alpha = 4*pi/5; omega = [-0.8; 0.24; -0.5];
q = [cos(alpha/2); sin(alpha/2)*omega/norm(omega)];
r = [1.12; -0.03; -1.13];

% Solve the problem.
tol = 1e-2; fun = @(e, x) e < tol;
[sol, ~, ~] = micsolverd(L1, L2, L3, q, r, 0.01, [5, 5], tol, 4, fun, 'plot', 'plot');

% Plot the results.
fh = figure();
circles3(fh, L1, L2, L3, sol(:, 1), 'k--');
circles3(fh, L1, L2, L3, sol(:, 2), 'k-');

% Sets the azimuth and elevation angles of the sight.
view(60, 15);

