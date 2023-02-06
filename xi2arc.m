function arc = xi2arc(L1, L2, L3, xi)
%XI2ARC Converts the overall exponential coordinate to the arc parameters
%       of 3 sections.
%   ARC = XI2ARC(L1, L2, L3, XI) computes the arc parameter ARC of a
%   3-section constant-curvature robot. The section lengths are L1, L2 and
%   L3, respectively. The parameter XI is the overall exponential
%   coordinate.
%
%   Example
%      xi = [-1.6; 0.8; 1.2; -0.2; 0.6; 0.2];
%      arc = xi2arc(1, 1, 1, xi);

k1 = mod(sqrt(xi(1)^2 + xi(2)^2), 2*pi) / L1;
p1 = atan2(-xi(1), xi(2));
k2 = mod(sqrt(xi(3)^2 + xi(4)^2), 2*pi) / L2;
p2 = atan2(-xi(3), xi(4));
k3 = mod(sqrt(xi(5)^2 + xi(6)^2), 2*pi) / L3;
p3 = atan2(-xi(5), xi(6));
arc = [k1, p1, k2, p2, k3, p3];
end