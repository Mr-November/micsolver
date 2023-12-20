function len = xi2len(xi)
%XI2LEN Converts the concatenated parameter to the actuator lengths.
% 
%   LEN = XI2LEN(XI) computes the actuator lengths LEN of a 3-section
%   constant-curvature robot. The concatenated parameter XI is defined in
%   our article.
% 
%   Example
%      xi = [-1.6; 0.8; 1.2; -0.2; 0.6; 0.2];
%      len = xi2len(xi);

r = 84/2;
psi_lambda = [0, 4*pi/9, 2*pi/9];
psi_mu = [0, 2*pi/3, 4*pi/3];
delta_1_mu = r * [cos(psi_lambda(1) + psi_mu);
                  sin(psi_lambda(1) + psi_mu)];
delta_2_mu = r * [cos(psi_lambda(2) + psi_mu);
                  sin(psi_lambda(2) + psi_mu)];
delta_3_mu = r * [cos(psi_lambda(3) + psi_mu);
                  sin(psi_lambda(3) + psi_mu)];
U = [delta_1_mu, delta_2_mu, delta_3_mu;
     zeros(size(delta_1_mu)), delta_2_mu, delta_3_mu;
     zeros(size(delta_1_mu)), zeros(size(delta_2_mu)), delta_3_mu];
C = [0, -1, 0, 0, 0, 0;
    1, 0, 0, 0, 0, 0;
    0, 0, 0, -1, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, -1;
    0, 0, 0, 0, 1, 0;];
P = [1, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 1;
     0, 0, 0, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0, 0];
len = P * U.' * C * xi; % xi here shall be the incremental value.
end

