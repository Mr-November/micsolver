function M = up_hat(V)
%UP_HAT Computes the Lie algebra of a vector.
%   M = UP_HAT(V) is an element of \mathsf{so}_3 or \mathsf{se}_3, where V
%   is an element of \mathbb{R}^3 or \mathbb{R}^6, respectively.

if length(V) == 3
    M = [    0,  -V(3),   V(2);
          V(3),      0,  -V(1);
         -V(2),   V(1),      0];
elseif length(V) == 6
    M = [    0,  -V(3),   V(2),   V(4);
          V(3),      0,  -V(1),   V(5);
         -V(2),   V(1),      0,   V(6);
             0,      0,      0,      0];
else
    error('Input not in R3/R6');
end
end