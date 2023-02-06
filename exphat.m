function M = exphat(V)
%EXPHAT Composition of the hat map and the matrix exponential.
%   M = EXPHAT(V) is a matrix in \mathsf{SO}_3 or \mathsf{SE}_3 and is
%   computed using Rodrigues' formula. The vector V is in \mathbb{R}^3 or
%   \mathbb{R}^4. The hat map sends V to an element of \mathsf{so}_3 or
%   \mathsf{se}_3.

theta = norm(V(1:3));
if theta < 2e-8
    M = [eye(3), V(4:6); 0, 0, 0, 1];
else
    omega = V(1:3) ./ theta;
    v = V(4:6) ./ theta;
    M = [eye(3) + sin(theta) * up_hat(omega) + (1 - cos(theta)) * up_hat(omega)^2, (eye(3) * theta + (1 - cos(theta)) * up_hat(omega) + (theta - sin(theta)) * up_hat(omega)^2) * v; 0, 0, 0, 1];
end
end