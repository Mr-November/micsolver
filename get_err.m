function err = get_err(r1, r2, r3, L1, L2, L3, q, r)
%GET_ERR Computes the error between desired and current end pose.
%        This is a private function of our solver.

Td = [q2rot(q), r; 0, 0, 0, 1];
T1 = [q2rot([r1(3), -r1(2), r1(1), 0]), rho(r1(3), L1).*r1; 0, 0, 0, 1];
T2 = [q2rot([r2(3), -r2(2), r2(1), 0]), rho(r2(3), L2).*r2; 0, 0, 0, 1];
T3 = [q2rot([r3(3), -r3(2), r3(1), 0]), rho(r3(3), L3).*r3; 0, 0, 0, 1];
Tt = T1 * T2 * T3;
V = veelog(Tt \ Td);
err = norm(V);
end