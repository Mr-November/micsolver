function arc = q2arc(q, L)
%Q2ARC Converts a quaternion to arc parameters.
%   ARC = Q2ARC(Q, L) computes the arc parameters of a 1-section
%   constant-curvature robot, including the curvature KAPPA and bending
%   angle PHI. The section length is L. The quaternion Q represents the end
%   rotation.
%
%   Example
%      q = [1/2; -3/4; sqrt(3)/4; 0];
%      arc = q2arc(q, 1);

kappa = 2/L*acos(q(1));
phi = atan2(-q(2), q(3));
arc = [kappa, phi];
end