function revise_plot(k, omg_e, v_e, err, tol, method)
%REVISE_PLOT Visualises the numerical correction.
%            This is a private function of numerical methods.
figure();
pos = [50, 50, 680, 600];
lwt = 3;
ms = 5.5;
colour2 = [255,78,166]/255;
colour1 = [127,17,70]/255;
rs = semilogy(k, omg_e, 's--', 'LineWidth', lwt, 'MarkerSize', ms, 'Color', colour1, 'MarkerFaceColor', colour1);
hold on;
bs = semilogy(k, v_e, 's--', 'LineWidth', lwt, 'MarkerSize', ms, 'Color', colour2, 'MarkerFaceColor', colour2);
ks = semilogy(k, err, 's-', 'LineWidth', lwt, 'MarkerSize', ms, 'Color', [0, 0, 0, 1]);

xlabel('Step of iterations', 'Units', 'normalized', 'Position', [0.5,-0.09,0]);
ylabel('Error', 'Units', 'normalized', 'Position', [-0.13,0.5,0]);
grid off;

xl = xlim;
yl = ylim;
plot([0, k(end)], [tol, tol], '-', 'Color', [0.7, 0.7, 0.7, 0.2], 'LineWidth', 2*lwt);
tolax = fill([-1,-1,k(end),k(end)], [1e-20,tol,tol,1e-20], [1,1,1]*0.8, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlim(xl);
ylim(yl);

if method == 1
    title('Gradient Method');
    legend([rs, bs, ks, tolax], ' Angular error', ' Linear error', ' Total error', ' Tolerance', ...
        'Location', 'northeast');
elseif method == 2
    title('Damped least-squares Method');
    legend([rs, bs, ks, tolax], ' Angular error', ' Linear error', ' Total error', ' Tolerance', ...
        'Location', 'northeast');
elseif method == 3
    title('Newton-Raphson Method');
    legend([rs, bs, ks, tolax], ' Angular error', ' Linear error', ' Total error', ' Tolerance', ...
        'Location', 'southwest');
end
set(gcf, 'Position', pos);
set(gca, 'InnerPosition', pos, 'Units', 'centimeters', 'Position', [2.7,2.2,14.3,12.7], ...
    'FontName', 'Times New Roman', 'FontSize', 20, ...
    'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
    'YGrid', 'on', 'YMinorGrid', 'on', ...
    'LineWidth', 0.7, 'Box', 'off');

end
