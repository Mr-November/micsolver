function T = get_end(L1, L2, L3, xi)
%GET_END Computes the end pose of a 3-section constant-curvature robot.
%   T = GET_END(L1, L2, L3, XI) returns the 4-by-4 matrix of end pose.
%
%   Example
%      xi = [-1.6; 0.8; 1.2; -0.2; 0.6; 0.2];
%      T = get_end(1, 1, 1, xi);

T1 = exphat( [xi(1); xi(2); 0; 0; 0; L1] );
T2 = exphat( [xi(3); xi(4); 0; 0; 0; L2] );
T3 = exphat( [xi(5); xi(6); 0; 0; 0; L3] );
T = T1 * T2 * T3;
end