function J = jacobian3cc(L1, L2, L3, xi)
%JACOBIAN3CC Computes the Jacobian matrix when the forward kinematics is
%            expressed by the product of exponentials formula.
%   J = JACOBIAN3CC(L1, L2, L3, XI) returns the 6-by-6 Jacobian matrix.

j3 = jaco_c12(xi(5), xi(6), L3);

T3 = expm(up_hat( [xi(5); xi(6); 0; 0; 0; L3] ));
invT3 = [T3(1:3, 1:3).', -T3(1:3, 1:3).' * T3(1:3, 4); 0, 0, 0, 1];
j2 = jaco_c12(xi(3), xi(4), L2);
j2_c1 = up_vee(invT3 * up_hat(j2(:, 1)) * T3);
j2_c2 = up_vee(invT3 * up_hat(j2(:, 2)) * T3);
j2 = [j2_c1, j2_c2];

T2 = expm(up_hat( [xi(3); xi(4); 0; 0; 0; L2] ));
invT2 = [T2(1:3, 1:3).', -T2(1:3, 1:3).' * T2(1:3, 4); 0, 0, 0, 1];
j1 = jaco_c12(xi(1), xi(2), L1);
j1_c1 = up_vee(invT3 * invT2 * up_hat(j1(:, 1)) * T2 * T3);
j1_c2 = up_vee(invT3 * invT2 * up_hat(j1(:, 2)) * T2 * T3);
j1 = [j1_c1, j1_c2];

J = [j1, j2, j3];
end

function Jc = jaco_c12(w1, w2, L)
% Jleft = @(w) eye(3) + (1 - cos(norm(w)))/(norm(w))^2 .* up_hat(w) + (norm(w) - sin(norm(w)))/(norm(w))^3 .* (up_hat(w))^2;
% Jright = @(w) eye(3) - (1 - cos(norm(w)))/(norm(w))^2 .* up_hat(w) + (norm(w) - sin(norm(w)))/(norm(w))^3 .* (up_hat(w))^2;
% g = @(w, v) [expm(up_hat(w)), Jleft(w)*v; 0, 0, 0, 1];
w = [w1, w2, 0];
n = norm(w);
if n == 0
    ML = L/2;
    Jc = [eye(3); [0, ML, 0; -ML, 0, 0; 0, 0, 0]];
else
    M = (1 - cos(n)) / n^2;
    N = (n - sin(n)) / n^3;
    p1M = w1/n^2 - w1*N - 2*w1*M/n^2;
    p2M = w2/n^2 - w2*N - 2*w2*M/n^2;
    p1N = w1*M/n^2 - 3*w1*N/n^2;
    p2N = w2*M/n^2 - 3*w2*N/n^2;
    pwJleftwv = L * [           p1M*w2,           p2M*w2 + M,    0;
                           -p1M*w1 - M,              -p2M*w1,    0;
                     -p1N*n^2 - 2*N*w1,    -p2N*n^2 - 2*N*w2,    0];
    Jc = [eye(3) - M .* up_hat(w) + N .* (up_hat(w))^2;
                         expm(up_hat(w)).' * pwJleftwv];
end
Jc = Jc(:, 1:2);
end