function [mu_err, sigma, psi] = update(muNom_bar, muErr_bar, sigma_bar, z, Q, M, lambda_M)
% Inputs
%   z: a i*j matrix, where i corresponds to the dimension of the measurement 
% and j to the number of measurements 

dim_meas = size(z, 1); % Dimension of measurement
n_obs = size(z, 2); % Number of observations
N = size(muErr_bar,1); % state dimension  N = 19 or 18
n_land = size(M,2); % number of landmarks

% Initialize variables
c = zeros(1,n_obs); % landmark index for each observation
outlier = zeros(1, n_obs); % binary value for outlier detection 
nu = zeros(dim_meas, n_obs); % innovation
psi = zeros(1,n_obs);

H = zeros(dim_meas, N, n_land); % jacobian: store jacobian for every landmark
h = zeros(dim_meas,n_land); % expected measurements for each landmark
S = zeros(dim_meas,dim_meas, n_land);

% Loop over all observations
for i = 1:n_obs
    % Compute and store best association. 
    % Output-> best landmark: best_j, outlier_i, nu_i, H_i
    [c(i), psi(i), outlier(i), nu(:, i), H, h, S] = associate(muNom_bar, muErr_bar, ...
     sigma_bar, z(:,i), M, Q, lambda_M, H, h, S, i, dim_meas, n_obs, n_land);
end

% use indexes vector (c) to get correct H for each observation
H = H(:,:,c);

% Initialize nu_bar and H_bar
index_inliers = find(outlier == 0);
n_inliers = length(index_inliers);
nu_Bar = reshape(nu(:, index_inliers), [], 1);   % N*n x 1
H_Bar = reshape( permute(H(:, :, index_inliers), [1, 3, 2]) ,  [],  N);   % might cause errors, check this. before, N was size(H,2)  

% Create block diagonal matrix
Q_Bar = kron(eye(n_inliers), Q);

% Update covariance and mu
K = sigma_bar * H_Bar'* (H_Bar * sigma_bar * H_Bar' + Q_Bar)^-1;
mu_err = K * nu_Bar; 
sigma = (eye(N)- K * H_Bar) * sigma_bar;
sigma = (sigma + sigma')/2; %to ensure symmetry;



end

% function [Q_bar] = measCovMatrix(Q, n_inliers, k)
% Q_bar = zeros(n_inliers*k);
% for i = 1:n_inliers
%     ii = k*(i-1)+1;
%     Q_bar(ii:ii+k-1, ii:ii+k-1) = Q;
% end
% 
% end