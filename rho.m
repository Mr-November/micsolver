function d = rho(a, L)
%RHO Computes the linear distance between two ends of a circular arc.
%    This is a private function of our solver.

if a == 1
    d = L;
else
    d = L*sqrt(1-a^2)/acos(a);
end
end