function circles3(fh, L1, L2, L3, xi, type)
%CIRCLES3 Visualises the 3-section constant-curvature robot with given
%         model parameters.
%   CIRCLES3(FH, L1, L2, L3, XI, TYPE) displays the plot in target figure
%   FH. The robot is described by the section lengths L1, L2, L3 and the
%   overall exponential coordinate XI. Line colours and styles are
%   specified in the character string TYPE.
%
%   Example
%      L1 = 1; L2 = 1; L3 = 1;
%      xi_1 = [-1.60; 0.08; 1.20; -0.20; 0.60; 0.20];
%      circles3(1, L1, L2, L3, xi_1, 'k--');
%      xi_2 = [-0.39; 0.48; -1.13; 0.47; 1.79; -0.17];
%      circles3(1, L1, L2, L3, xi_2, 'k-');
%      view(75, 9);

arc = xi2arc(L1, L2, L3, xi);
para = [arc(1: 2), L1, arc(3: 4), L2, arc(5: 6), L3];
Tstd = eye(4);
figure(fh);
frame(fh, Tstd, 1);
% extend = 0.2;
lw = 4;
ms = 22;
% Use smaller line width and marker size if not clear.
% lw = 2;
% ms = 11;
pos = [50, 50, 680, 600];
if isempty(type)
    type = 'k-';
end

for k = 1: length(para)/3
    kp = para((k-1)*3+1);
    ph = para((k-1)*3+2);
    L = para((k-1)*3+3);
    if kp*L < 1e-3
        % Line.
        cx = [0, 0];
        cy = [0, 0];
        cz = [0, L];
        c1 = [1, 1];
        cod = Tstd * [cx; cy; cz; c1];
        plot3(cod(1, :), cod(2, :), cod(3, :), type, 'LineWidth', lw);
        hold on;
        Tstd = Tstd * expm(up_hat([0; 0; 0; 0; 0; L]));
        plot3(Tstd(1, 4), Tstd(2, 4), Tstd(3, 4), [type(1), '.'], 'MarkerSize', ms);
        hold on;
    else
        % Circle.
        % cx = cos(ph).*(1+cos(pi-kp*L-extend*kp*L: 0.02: pi+extend*kp*L));
        % cy = sin(ph).*(1+cos(pi-kp*L-extend*kp*L: 0.02: pi+extend*kp*L));
        % cz = sin(pi-kp*L-extend*kp*L: 0.02: pi+extend*kp*L);
        % c1 = ones(size(cx));
        % cr = 1 / kp;
        % cod = Tstd * [cr .* [cx; cy; cz]; c1];
        % plot3(cod(1, :), cod(2, :), cod(3, :), type);
        % hold on;

        % Arc.
        cr = 1 / kp;
        ax = cos(ph).*(1+cos(linspace(pi-kp*L, pi, 20)));
        ay = sin(ph).*(1+cos(linspace(pi-kp*L, pi, 20)));
        az = sin(linspace(pi-kp*L, pi, 20));
        a1 = ones(size(ax));
        cod = Tstd * [cr .* [ax; ay; az]; a1];
        plot3(cod(1, :), cod(2, :), cod(3, :), type, 'LineWidth', lw);
        hold on;
        Tstd = Tstd * expm(up_hat([-kp*L*sin(ph); kp*L*cos(ph); 0; 0; 0; L]));
        plot3(Tstd(1, 4), Tstd(2, 4), Tstd(3, 4), [type(1), '.'], 'MarkerSize', ms);
        hold on;
    end
    % frame(fh, Tstd, 0.8);
end
figure(fh);
frame(fh, Tstd, 0.8); % The end frame.
plot3(Tstd(1, 4), Tstd(2, 4), Tstd(3, 4), 'k.', 'MarkerSize', ms);
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