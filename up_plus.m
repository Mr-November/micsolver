function q_up_plus = up_plus(q)
%UP_PLUS Computes the matrix for left multiplications of quaternions.
%   Q_UP_PLUS = UP_PLUS(Q) returns the left multiplication matrix of Q.

% q^+ = (\delta, -\epsilon^t
%        \epsilon^t, \delta I + \epsilon^\times)
delta = q(1);
epsilon = q(2:4);
q_up_plus = [delta, -epsilon.'; epsilon, delta * eye(3) + up_hat(epsilon)];
end