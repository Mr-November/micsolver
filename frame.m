function frame(fh, A, alpha)
%FRAME draws a coordinate frame with specified position (and orientation)
%      and transparency.
% 
%   FRAME(FH, A, ALPHA) displays the plot in target figure FH. The position
%   (and orientation) is specified by A and the transparency is specified
%   by ALPHA.
% 
%   Example
%      frame(1, [eye(3), zeros(3, 1); 0, 0, 0, 1], 0.3);

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