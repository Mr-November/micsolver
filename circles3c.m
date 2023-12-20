function circles3c(fh, L1, L2, L3, xi, type, colour)
%CIRCLES3C Visualises the 3-section constant-curvature robot with given
%          model parameters and a specified colour.
%   CIRCLES3C(FH, L1, L2, L3, XI, TYPE, COLOUR) displays the plot in target
%   figure FH. The robot is described by the section lengths L1, L2, L3 and
%   the overall exponential coordinate XI. Line styles are specified in the
%   character string TYPE, line colours are specified in the triple COLOUR.
%
%   Example
%      L1 = 1; L2 = 1; L3 = 1;
%      xi_1 = [-1.60; 0.08; 1.20; -0.20; 0.60; 0.20];
%      circles3c(1, L1, L2, L3, xi_1, '--', [0.1, 0.1, 0.1]);
%      xi_2 = [-0.39; 0.48; -1.13; 0.47; 1.79; -0.17];
%      circles3c(1, L1, L2, L3, xi_2, '-', [0.2, 0.2, 0.2]);
%      view(75, 9);

tmp = xi2arc(L1, L2, L3, xi);
arc = [tmp(1: 2), L1, tmp(3: 4), L2, tmp(5: 6), L3];
Tstd = eye(4);
figure(fh);
frame(fh, Tstd, 1);
lw = 2;
ms = 11;

for k = 1: length(arc)/3
    kp = arc((k-1)*3+1);
    ph = arc((k-1)*3+2);
    L = arc((k-1)*3+3);
    if kp*L < 1e-3
        % Line.
        cx = [0, 0];
        cy = [0, 0];
        cz = [0, L];
        c1 = [1, 1];
        cod = Tstd * [cx; cy; cz; c1];
        plot3(cod(1, :), cod(2, :), cod(3, :), type, 'LineWidth', lw, 'Color', colour);
        hold on;
        Tstd = Tstd * expm(up_hat([0; 0; 0; 0; 0; L]));
        plot3(Tstd(1, 4), Tstd(2, 4), Tstd(3, 4), '.', 'MarkerSize', ms, 'Color', colour);
        hold on;
    else
        % Circle.
        cr = 1 / kp;
        % Arc.
        ax = cos(ph).*(1+cos(linspace(pi-kp*L, pi, 20)));
        ay = sin(ph).*(1+cos(linspace(pi-kp*L, pi, 20)));
        az = sin(linspace(pi-kp*L, pi, 20));
        a1 = ones(size(ax));
        cod = Tstd * [cr .* [ax; ay; az]; a1];
        plot3(cod(1, :), cod(2, :), cod(3, :), type, 'LineWidth', lw, 'Color', colour);
        hold on;
        Tstd = Tstd * expm(up_hat([-kp*L*sin(ph); kp*L*cos(ph); 0; 0; 0; L]));
        plot3(Tstd(1, 4), Tstd(2, 4), Tstd(3, 4), '.', 'MarkerSize', ms, 'Color', colour);
        hold on;
    end
end
figure(fh);
plot3(Tstd(1, 4), Tstd(2, 4), Tstd(3, 4), '.', 'MarkerSize', ms, 'Color', colour);

pos = [50, 50, 680, 600];
xlabel('$x$', 'Interpreter', 'latex');
ylabel('$y$', 'Interpreter', 'latex');
zlabel('$z$~~~~~~~~', 'Rotation', 0, 'Interpreter', 'latex');
pbaspect([1, 1, 1]);
daspect([1, 1, 1]);
xticks([-2, -1, 0, 1, 2]);
yticks([-2, -1, 0, 1, 2]);
zticks([-2, -1, 0, 1, 2, 3]);
set(gcf, 'Position', pos);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, ...
    'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
    'LineWidth', 0.7);
end

function frame(fh, A, alpha)
figure(fh);
len = 0.2;
lw = 6;
new_red = [238, 0, 0]/255;
new_green = [0, 238, 0]/255;
new_blue = [0, 0, 238]/255;
if size(A) == [4, 4]
    cod = [len .* eye(3); 1, 1, 1];
    org = A * [0; 0; 0; 1];
    cod = A * cod;
    plot3([org(1), cod(1, 1)], [org(2), cod(2, 1)], [org(3), cod(3, 1)], ...
        '-', 'LineWidth', lw, 'Color', [new_red, alpha]);
    hold on;
    plot3([org(1), cod(1, 2)], [org(2), cod(2, 2)], [org(3), cod(3, 2)], ...
        '-', 'LineWidth', lw, 'Color', [new_green, alpha]);
    plot3([org(1), cod(1, 3)], [org(2), cod(2, 3)], [org(3), cod(3, 3)], ...
        '-', 'LineWidth', lw, 'Color', [new_blue, alpha]);
elseif size(A) == [3, 3]
    cod = len .* eye(3);
    org = [0; 0; 0];
    cod = A * cod;
    plot3([org(1), cod(1, 1)], [org(2), cod(2, 1)], [org(3), cod(3, 1)], ...
        '-', 'LineWidth', lw, 'Color', [new_red, alpha]);
    hold on;
    plot3([org(1), cod(1, 2)], [org(2), cod(2, 2)], [org(3), cod(3, 2)], ...
        '-', 'LineWidth', lw, 'Color', [new_green, alpha]);
    plot3([org(1), cod(1, 3)], [org(2), cod(2, 3)], [org(3), cod(3, 3)], ...
        '-', 'LineWidth', lw, 'Color', [new_blue, alpha]);
elseif size(A) == [2, 3]
    cod = len .* eye(2);
    org = A(:, 3);
    cod = org + A(:, 1:2) * cod;
    plot([org(1), cod(1, 1)], [org(2), cod(2, 1)], ...
        '-', 'LineWidth', lw, 'Color', [new_red, alpha]);
    hold on;
    plot([org(1), cod(1, 2)], [org(2), cod(2, 2)], ...
        '-', 'LineWidth', lw, 'Color', [new_blue, alpha]);
end
end