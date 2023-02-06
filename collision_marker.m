function collision_marker(L1, L2, L3, xi, ro, ror)
%COLLISION_MARKER Visualises the collision part in yellow.
%   COLLISION_MARKER(L1, L2, L3, XI, RO, ROR) draws the part of the robot
%   that is collided with obstacles. The robot is described by the section
%   lengths L1, L2, L3 and the overall exponential coordinate XI. The
%   obstacles are spheres centring at RO with radius ROR.
%
%   Example
%      L1 = 1; L2 = 1; L3 = 1;
%      xi = [-1.6; 0.8; 1.2; -0.2; 0.6; 0.2];
%      ro = [0.8; 0.7; 0.6];
%      ror = 0.4;
%      circles3(1, L1, L2, L3, xi, 'k-');
%      collision_marker(L1, L2, L3, xi, ro, ror);
%      view(75, 9);

ms = 16;
clr = [[255, 221, 0]/255, 1];
smp = 600;
k = length(ror);
T1 = exphat( [xi(1); xi(2); 0; 0; 0; L1] );
T12 = T1 * exphat( [xi(3); xi(4); 0; 0; 0; L2] );
m = [];
for i = 1: smp
    Tt1 = exphat( [xi(1); xi(2); 0; 0; 0; L1]/smp*i );
    r = Tt1(1:3, 4);
    for j = 1: k
        dr = r - ro(:, j);
        dist = norm(dr);
        if dist < ror(k)
            m = [m, [r(1); r(2); r(3)]];
        end
    end

    Tt2 = T1 * exphat( [xi(3); xi(4); 0; 0; 0; L2]/smp*i );
    r = Tt2(1:3, 4);
    for j = 1: k
        dr = r - ro(:, j);
        dist = norm(dr);
        if dist < ror(k)
            m = [m, [r(1); r(2); r(3)]];
        end
    end

    Tt3 = T12 * exphat( [xi(5); xi(6); 0; 0; 0; L3]/smp*i );
    r = Tt3(1:3, 4);
    for j = 1: k
        dr = r - ro(:, j);
        dist = norm(dr);
        if dist < ror(k)
            m = [m, [r(1); r(2); r(3)]];
        end
    end
end
if ~isempty(m)
    plot3(m(1, :), m(2, :), m(3, :), '.', 'MarkerSize', ms, 'Color', clr);
end
end