function xi = soln2xi(L1, L2, L3, soln)
%SOLN2XI Converts the output of our solver to an exponential coordinate.
%        This is a private function of our solver.

% kappa = 2/L*acos(q(1));
% phi = atan2(-q(2), q(3));
% arc = [kappa, phi];
k1 = 2/L1*acos(soln(3));
p1 = atan2(soln(2), soln(1));
k2 = 2/L2*acos(soln(6));
p2 = atan2(soln(5), soln(4));
k3 = 2/L3*acos(soln(9));
p3 = atan2(soln(8), soln(7));
xi = [-L1*k1*sin(p1);
       L1*k1*cos(p1);
      -L2*k2*sin(p2);
       L2*k2*cos(p2);
      -L3*k3*sin(p3);
       L3*k3*cos(p3)];
end