function ts = allocate_time(xis)
%ALLOCATE_TIME allocates optimal time to a given sequence of concatenated
%   parameters, considering the actuator velocity constraints.
% 
%   TS = ALLOCATE_TIME(XIS) returns the time array TS.

% Load mechanism constants.
load 'PUTC.mat' PUTC
RESOLUTION = -200000/pi ./ [7, 7, 7, 21, 21, 21, 21, 21, 21].';
FACTOR = sqrt(2);
VEL = [12000, 12000, 12000, 4000, 4000, 4000, 4000, 4000, 4000].';
L_DOT_MAX = VEL ./ RESOLUTION ./ FACTOR;

p = size(xis, 2);
titv = zeros(1, p);
for i = 2:p
    xi_diff = xis(:, i) - xis(:, i-1);
    diff = PUTC * xi_diff;
    titv(i) = max(abs(diff) ./ abs(L_DOT_MAX));
end
ts = zeros(1, p);
for i = 2:p
    ts(i) = titv(i) + ts(i-1);
end

end

