function ts = allocate_time(xis)
%ALLOCATE_TIME allocates optimal time to a given sequence of concatenated
%   parameters, considering the actuator velocity constraints.
% 
%   TS = ALLOCATE_TIME(XIS) returns the time array TS.

RESOLUTION = -200000/(21*pi);
FACTOR = sqrt(2);
VEL_MAX = [4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000].';
L_DOT_MAX = VEL_MAX ./ RESOLUTION ./ FACTOR;

p = size(xis, 2);
titv = zeros(1, p);
for i = 2:p
    xi_diff = xis(:, i) - xis(:, i-1);
    diff = xi2len(xi_diff);
    titv(i) = max(abs(diff) ./ abs(L_DOT_MAX));
end
ts = zeros(1, p);
for i = 2:p
    ts(i) = titv(i) + ts(i-1);
end

end

