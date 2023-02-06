function q = rot2q(R)
%ROT2Q Converts a rotation matrix to a quaternion.
%   Q = ROT2Q(R) returns the quaternion that is equivalent to the rotation
%   matrix.

u = up_vee(logm(R));
theta = norm(u);
u = u / theta;
q = [cos(theta/2); sin(theta/2).*u];
if q(1) < 0
    q = -q;
end
end