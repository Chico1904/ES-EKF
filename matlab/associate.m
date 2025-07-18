function [c, max_psi, outlier, nu_bar_i, H, h, S] = associate(muNom_bar, ...
    muErr_bar, sigma_bar, z, M, Q, lambda_M, H, h, S, i, k, n, m)
% INPUT
%   z: k*n matrix, where k is the dimension of the measurement and n is the
% number of observations in a given time instant
%   M: 3*m matrix, where m is the number of landmarks
%   i, k, n, m: observation index, dimensionality of each measurement,
%   number os observations and number of landmarks, respectively
% CALLED FUNCTIONS:
%   [h, H] = measure(muNom_bar, muErr_bar,j, M): measurement model: obtain 
% (pseudo) measurements and respective jacobian (MARIA)
% Note: as h, H and S_j depend solely on the landmark, we'll compute them
% for the first observation and store it (i =1). For the next observations,
% we'll reuse them to avoid redundant computations

nu = zeros(k,m);
psi = zeros(1,m);

for j = 1:m % iterate over landmarks
    if i == 1
        [hh, HH] = measure(muNom_bar, muErr_bar, j, M); % calculate expected measurement and jacobian for each landmark
        H(:,:,j) = HH; 
        h(:,j) = hh;
        S(:,:,j) = H(:,:,j) * sigma_bar * H(:,:,j)' + Q; 
    end
    nu(:,j) = z - h(:,j); % innovation
    DM = nu(:,j)'*(S(:,:,j)\nu(:,j)); % mahalanobis distance
    psi(j) = det(2*pi*S(:,:,j))^(-0.5)*exp(-0.5*DM); % likelihood
end

if abs(sum(psi)) < 1e-50 
    psi = zeros(1,size(psi,2));
else
    psi = psi/sum(psi);
end

% threshold in the likelihood instead of mahalanobis
[max_psi, c] = max(psi);
outlier = psi(c) < lambda_M;
nu_bar_i = nu(:,c);

end