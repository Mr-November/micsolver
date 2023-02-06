function V = veelog(M)
%VEELOG Composition of the matrix logarithm and the vee map.
%   V = VEELOG(M) is a vector in \mathbb{R}^3 or \mathbb{R}^4 and is
%   computed using Rodrigues' formula. The matrix M is in \mathsf{SO}_3 or
%   \mathsf{SE}_3. The vee map sends an element of \mathsf{so}_3 or
%   \mathsf{se}_3 to a vector.

R = M(1: 3, 1: 3);
if norm(R - eye(3)) < 2e-8
    V = [0; 0; 0; M(1:3, 4)];
else
    theta = acos((trace(R)-1)/2);
    omega_hat = 1/(2*sin(theta))*(R - R.');
    V = [up_vee(omega_hat) * theta; (eye(3) - theta/2*omega_hat + (1 - theta/2*cot(theta/2)) * omega_hat^2) * M(1:3, 4)];
end
end