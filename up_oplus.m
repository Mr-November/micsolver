function q_up_oplus = up_oplus(q)
%UP_OPLUS Computes the matrix for right multiplications of quaternions.
%   Q_UP_OPLUS = UP_OPLUS(Q) returns the right multiplication matrix of Q.

% q^\oplus = (\delta, -\epsilon^t
%             \epsilon^t, \delta I - \epsilon^\times)
delta = q(1);
epsilon = q(2:4);
q_up_oplus = [delta, -epsilon.'; epsilon, delta * eye(3) - up_hat(epsilon)];
end