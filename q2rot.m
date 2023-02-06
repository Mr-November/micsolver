function R = q2rot(q)
%Q2ROT Converts a quaternion to a rotation matrix.
%   R = Q2ROT(Q) returns the rotation matrix that is equivalent to the
%   quaternion.

a = q(1);
b = q(2);
c = q(3);
d = q(4);
R = [1-2*c^2-2*d^2, 2*b*c-2*a*d, 2*a*c+2*b*d;
     2*b*c+2*a*d, 1-2*b^2-2*d^2, 2*c*d-2*a*b;
     2*b*d-2*a*c, 2*a*b+2*c*d, 1-2*b^2-2*c^2];
end