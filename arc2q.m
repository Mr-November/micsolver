function q = arc2q(kappa, phi, L)
%ARC2Q Converts arc parameters to a quaternion.
%   Q = ARC2Q(KAPPA, PHI, L) computes the quaternion Q representing the end
%   rotation of a 1-section constant-curvature robot with curvature KAPPA,
%   bending angle PHI, and section length L.
%
%   Example
%      kappa = 2*pi/3;
%      phi = pi/3;
%      q = arc2q(kappa, phi, 1);

q(1) = cos(kappa*L/2);
q(2) = -sin(kappa*L/2)*sin(phi);
q(3) = sin(kappa*L/2)*cos(phi);
q(4) = 0;
end
