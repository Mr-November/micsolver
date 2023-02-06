function xi = arc2xi(L1, L2, L3, arc)
%ARC2XI Converts the arc parameters of 3 sections to the exponential coordinate.
%   XI = ARC2XI(L1, L2, L3, ARC) computes the exponential coordinate XI of
%   a 3-section constant-curvature robot. The section lengths are L1, L2
%   and L3, respectively. The parameter ARC is an array containing
%   curvatures and bending angles of each section.
%
%   Example
%      k1 = 4*sqrt(5)/5; p1 = atan(2);
%      k2 = sqrt(37)/5; p2 = -pi+atan(6);
%      k3 = sqrt(10)/5; p3 = -atan(3);
%      xi = arc2xi(1, 1, 1, [k1, p1, k2, p2, k3, p3]);

xi = [-L1*arc(1)*sin(arc(2));
       L1*arc(1)*cos(arc(2));
      -L2*arc(3)*sin(arc(4));
       L2*arc(3)*cos(arc(4));
      -L3*arc(5)*sin(arc(6));
       L3*arc(5)*cos(arc(6))];
end