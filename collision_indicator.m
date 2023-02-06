function collide = collision_indicator(L1, L2, L3, xi, ro, ror, smp)
%COLLISION_INDICATOR Computes the minimal distance between the sample
%                    points and the spherical obstacles.
%   COLLIDE = COLLISION_INDICATOR(L1, L2, L3, XI, RO, ROR, SMP) returns the
%   minimal distance COLLIDE. If COLLIDE > 0, then no collision occurs. If
%   COLLIDE < 0, then the robot collides with obstacles. The robot is
%   described by the section lengths L1, L2, L3 and the overall exponential
%   coordinate XI. The obstacles are spheres centring at RO with radius
%   ROR. The sample points are distributed uniformly along the robot curve
%   with the number SMP.
%
%   Example
%      L1 = 1; L2 = 1; L3 = 1;
%      xi = [-1.6; 0.8; 1.2; -0.2; 0.6; 0.2];
%      ro = [0.8; 0.7; 0.6];
%      ror = 0.4;
%      collide = collision_indicator(L1, L2, L3, xi, ro, ror, 10);

k = length(ror);
if k == 0
    collide = inf;
else
    T1 = exphat( [xi(1); xi(2); 0; 0; 0; L1] );
    T12 = T1 * exphat( [xi(3); xi(4); 0; 0; 0; L2] );
    ind = nan(1, smp*3);
    for i = 1: smp
        Tt1 = exphat( [xi(1); xi(2); 0; 0; 0; L1]/smp*i );
        inds = nan(1, k);
        for j = 1: k
            dr = Tt1(1:3, 4) - ro(:, j);
            dist = norm(dr);
            inds(j) = dist - ror(k);
        end
        ind(i) = min(inds);

        Tt2 = T1 * exphat( [xi(3); xi(4); 0; 0; 0; L2]/smp*i );
        inds = nan(1, k);
        for j = 1: k
            dr = Tt2(1:3, 4) - ro(:, j);
            dist = norm(dr);
            inds(j) = dist - ror(k);
        end
        ind(smp+i) = min(inds);

        Tt3 = T12 * exphat( [xi(5); xi(6); 0; 0; 0; L3]/smp*i );
        inds = nan(1, k);
        for j = 1: k
            dr = Tt3(1:3, 4) - ro(:, j);
            dist = norm(dr);
            inds(j) = dist - ror(k);
        end
        ind(2*smp+i) = min(inds);
    end
    collide = min(ind);
end
end