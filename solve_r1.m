function r1 = solve_r1(L1, q, r, r3, noc)
%SOLVE_R1 Computes the model parameter of the 1st section.
%         This is a private function of our solver.

a = q(1); b = q(2); c = q(3); d = q(4);
B = [d, a, b; -a, d, c; -b, -c, d];
n0 = B.' * r;
r0 = (L1 * d)/norm(n0)^2 .* n0;
ne = B * r3;

r1 = spp(n0, (1/2 + 1/pi) * L1 * d, ne, r3);

if d ~= 0
    for cor_idx = 1: noc % One-step correction.
        tmp = r1 - [r0, ne] * (([n0.' + [0, 0, L1*d*(1/acos(r1(3)) - 1/sqrt(1-r1(3)^2))]; ne.'] * [r0, ne]) \ [n0.' * r1 - rho(r1(3), L1) * d; ne.' * r1]);
        r1 = tmp / norm(tmp);
    end
end
end


function soln = spp(n1, d, n2, rn)
% This function solves $r$ that satisfies
% n_1 \cdot r - d = 0,
% n_2 \cdot r = 0,
% | r | = 1.
% Geometrically, $r$ is the intersections of a sphere and two planes.
% Here, we need n_1 cross n_2 \neq 0.

n11 = n1(1);
n12 = n1(2);
n13 = n1(3);

n21 = n2(1);
n22 = n2(2);
n23 = n2(3);

det0 = n11 * n22 - n12 * n21;
det1 = n12 * n23 - n13 * n22;
det2 = n11 * n23 - n13 * n21;

a = det1^2 + det2^2 + det0^2;
if a < eps
    soln = [-rn(1)*rn(3); -rn(2)*rn(3); rn(1)^2+rn(2)^2]/sqrt(rn(1)^2+rn(2)^2);
else
    b = 2 * d * (n22 * det1 + n21 * det2);
    c = d^2 * (n22^2 + n21^2) - det0^2;
    delta = b^2 - 4 * a * c;
    if delta < 0
        r3 = -b / (2 * a);
        r1 = (det1 * r3 + n22 * d) / det0;
        r2 = -(det2 * r3 + n21 * d) / det0;
        soln = [r1; r2; r3] ./ norm([r1, r2, r3]);
    else
        r3 = (-b + sqrt(delta)) / (2 * a);
        r1 = (det1 * r3 + n22 * d) / det0;
        r2 = -(det2 * r3 + n21 * d) / det0;
        soln = [r1; r2; r3];
    end
end
end