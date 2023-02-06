function V = up_vee(M)
%UP_VEE Computes the Lie algebra of a vector.
%   V = UP_VEE(M) is an element of \mathbb{R}^3 or \mathbb{R}^6, where M is
%   an element of \mathsf{so}_3 or \mathsf{se}_3, respectively.

if length(M) == 3
    V = [-M(2, 3); M(1, 3); -M(1, 2)];
elseif length(M) == 4
    V = [-M(2, 3); M(1, 3); -M(1, 2); M(1:3, 4)];
else
    error('Input not in so/se3');
end
end