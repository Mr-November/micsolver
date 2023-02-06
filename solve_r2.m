function [r2r, r2t] = solve_r2(L1, L3, q, r, r3, r1)
%SOLVE_R2 Computes the model parameter of the 2nd section using rotational and translational constraints.
%         This is a private function of our solver.

a = q(1); b = q(2); c = q(3); d = q(4);
B = [d, a, b; -a, d, c; -b, -c, d];
m = [c; -b; a];
qe = [m.' * r3; B * r3];
re = [0; r] - rho(r3(3), L3) .* up_plus(qe) * up_oplus(up_star(qe)) * [0; r3]; re = re(2:4);

% Use rotational constraint.
% q_e = q \otimes q_3^{-1}
% A_e = ( -a_e  -d_e   c_e
%          d_e  -a_e  -b_e
%          c_e, -b_e,  a_e )
% r2 = A_e r1
ae = qe(1); be = qe(2); ce = qe(3); de = qe(4);
Ae = [-ae, -de, ce; de, -ae, -be; ce, -be, ae];
r2r = Ae * r1;

% Use translational constraint.
rv = re - rho(r1(3), L1) .* r1;
negtive_w2 = (2 * (r1 * r1.') - eye(3)) * rv / norm(rv);
r2t = [-negtive_w2(1); -negtive_w2(2); negtive_w2(3)];
end