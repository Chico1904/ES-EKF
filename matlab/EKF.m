function [muNomTotal, muErrTotal, sigmaTotal, M, dt, ground_truth] = EKF(dataset, pseudo_Q, mode, pseudo_R, ...
    R, Q, simulated_gt, muNom, muErr, sigma, lambda_M, outlierSpecs) 
% Main function of the filter
% MODE
%   0: creation of simulated ground_truth -> initialize function is
%   necessary
%   1: simulated ground truth with additional measurement and process noise
% measurement noise comes from createMeasurements, process noise is added
% in the function NoisesCovariance.
%   2: true ground truth 

if nargin  < 12
    dataset = 'dataset9_davis';
end

reverse = '';
% Process data: each ground_truth and imu_meas element should refer to 
% the same interval, hence they're arrays with the same size (time-wise).
% As there are several datasets, we should define which one we want (MARIA)
if mode == 1
    [true_ground_truth, imu_meas, n_timesteps, dt] = process(dataset); %ground-truth: 8*n_timesteps matrix; imu_meas: 7*n_timesteps matrix
    ground_truth = zeros(8,n_timesteps);
    ground_truth(2:8,:) = simulated_gt;
    ground_truth(1,:) = true_ground_truth(1,:);
elseif mode == 2
    [ground_truth, imu_meas, n_timesteps, dt] = process(dataset); %ground-truth: 8*n_timesteps matrix; imu_meas: 7*n_timesteps matrix
end

% Create pseudo land-marks (M) and pseudo measurements(y) (MARIA)
[M, Z] = createMeasurements(ground_truth, pseudo_Q, outlierSpecs);

% Compute R (process noise) and add noise to IMU measurements
if mode == 1
    %added_R = diag(added_process_noise);
    imu_meas(2:7,:) = imu_meas(2:7,:) + mvnrnd(zeros(1,6),pseudo_R, n_timesteps)';   
elseif mode == 2
    [R]  = IMUNoisesCovariances(dt);
end

% Initialize mu and sigma with uncertainties (CHICO)
%[muNom, muErr, sigma, Q] = initialize(ground_truth(:,1),init_unc_process, unc_meas);
if mode == 0 || mode == 2
    init_state = ground_truth(:,1);
    post_state = ground_truth(:,2);
    postpost_state = ground_truth(:,3);
    init_meas = imu_meas(2:end,1);
    if mode == 2
        [muNom, ~, sigma] = initialize(init_state, post_state, postpost_state, init_meas);
    else
        [muNom, muErr, sigma] = initialize(init_state, post_state, postpost_state, init_meas);
    end
end

% pre allocate mu's, sigma's and psi
muNomTotal = zeros(19, n_timesteps);
muNomTotal(:,1) = muNom;
muErrTotal = zeros(18, n_timesteps);
muErrTotal(:,1) = muErr;
sigmaTotal = zeros(18,18, n_timesteps);
sigmaTotal(:,:,1) = sigma;


% Filter loop
for i = 2:n_timesteps
    % Predict step (CHICO)
    u = imu_meas(2:end,i);
    [muNom_bar, muErr_bar, sigma_bar] = predict(muNom, muErr, sigma, u, R, dt);

    % associate and update (CHICO)
    n_land = size(M,2);
    chosenLandmarks = 1:n_land; % with 1:5 we're passing every observation (ie, from every landmark)
    % chosenLandmarks = unique(randi([1 n_land],1,n_land)) % chose some
    % % landmarks but not repeated

    if mod(i,1) == 0  %MUDAR
        % update step
        [muErr, sigma, psi] = update(muNom_bar, muErr_bar, sigma_bar, Z(:,chosenLandmarks,i), Q, M, lambda_M); 
        
        % store error state
        muErrTotal(:,i) = muErr;
    
        % Error injection and ESKF reset
        [muNom, muErr, sigma] = injectAndReset(muNom_bar,muErr, sigma);
    else
        muErr = muErr_bar;
        muNom = muNom_bar;
        sigma = sigma_bar;
    end


    % Store variables
    muNomTotal(:,i) = muNom;
    sigmaTotal(:,:,i) = sigma;

    % Display progress
    % progress = (i / n_timesteps) * 100;
    % fprintf('Progress: %.2f%%\r', progress);
    % drawnow;
    progress = sprintf('%d', fix(100*i/n_timesteps));
    fprintf([reverse, progress, ' %%']);
    reverse = repmat(sprintf('\b'), 1, length(progress)+2);
    drawnow;

end

end