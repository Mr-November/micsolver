function [xi_star, err, k] = revise_grad(L1, L2, L3, q, r, xi, mstep, tol, type)
%REVISE_GRAD Correct the initial value with the gradient method.
%   [XI_STAR, ERR, K] = REVISE_*(L1, L2, L3, Q, R, XI, MSTEP, TOL, TYPE)
%   returns the result of numerical correction.
%   
%   Methods
%      grad            gradient method
%      dls             damped least-squares method
%      newton          Newton-Raphson method
%   
%   Input parameters
%      L1, L2, L3      section length
%      Q, R            desired end rotation and translation
%      XI              initial value
%      MSTEP           allowed maximum steps of iterations
%      TOL             error tolerance
%      TYPE            set 'plot' to show the error of numerical correction
%   
%   Output parameters
%      XI_STAR         final value
%      ERR             final error
%      K               steps of iterations
%
%   Example
%      L1 = 1; L2 = 1; L3 = 1;
%      alpha = 15*pi/16; omega = [0.48; sqrt(3)/10; -0.86];
%      q = [cos(alpha/2); sin(alpha/2)*omega];
%      r = [-0.4; 1.1; 0.8];
%      xi_0 = arc2xi(L1, L2, L3, pi.*[1, 2, 1, 2, 1, 2].*rand(1, 6));
%      [xi, err, noi] = revise_grad(L1, L2, L3, q, r, xi_0, 2000, 1e-2, 'plot');
%      [xi, err, noi] = revise_dls(L1, L2, L3, q, r, xi_0, 2000, 1e-2, 'plot');
%      [xi, err, noi] = revise_newton(L1, L2, L3, q, r, xi_0, 200, 1e-2, 'plot');

Td = [q2rot(q), r; 0, 0, 0, 1];
omg_e = NaN(1, mstep);
v_e = NaN(1, mstep);
e = NaN(1, mstep);
k = 0;
while k < mstep
    Tt = get_end(L1, L2, L3, xi);
    V = up_vee(logm(Tt \ Td));
    omg_e(k+1) = norm(V(1: 3));
    v_e(k+1) = norm(V(4: 6));
    e(k+1) = norm(V);
    if e(k+1) < tol
        break;
    else
        % Update.
        J = jacobian3cc(L1, L2, L3, xi); % When at zero position, rank(J) = 4.
        lamd = 0.2;
        xi = xi + lamd * J' * V;
        xi = arc2xi(L1, L2, L3, xi2arc(L1, L2, L3, xi));
        k = k + 1;
    end
end
if k == mstep
    Tt = get_end(L1, L2, L3, xi);
    V = up_vee(logm(Tt \ Td));
    omg_e(k+1) = norm(V(1: 3));
    v_e(k+1) = norm(V(4: 6));
    e(k+1) = norm(V);
end
xi_star = xi;
err = e(k+1);
if isequal(type, 'plot')
    revise_plot(0: k, omg_e(1:k+1), v_e(1:k+1), e(1: k+1), tol, 1);
end
end