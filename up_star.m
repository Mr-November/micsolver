function q_up_star = up_star(q)
%UP_STAR Computes the quaternion conjugation.
%   Q_UP_STAR = UP_STAR(Q) returns the conjugation of Q.

% q^* = (\delta, -\epsilon)
delta = q(1);
epsilon = q(2:4);
q_up_star = [delta; -epsilon];
end