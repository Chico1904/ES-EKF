function [M, Z] = createMeasurements(ground_truth, pseudo_Q, outlierSpecs)
% Function that creates pseudo-measurements. It takes the ground truth, 
% computes the measurement accordingly (distance, bearing, coordinates,
% etc) and adds some noise to simulate a measurement
% INPUT
%   unc_meas: 7*7 covariance matrix with noise variance for each
%   component of the measurement (dx,dy,dz,qw,qx,qy,qz)
%   outlierSpecs: structure with the following fields
%                   - on: true if some observations should be outliers
%                   - rate: % of outliers in the observations
% OUTPUTS
%   M: n*m matrix, where n is the number of coordinates of each landmark
% (2 or 3 probably) and m is the number of landmarks
%   Z: n*m*k, where n is the dimension of each measurement (1 if it's
% distance, 2 if it's distance + bearing, etc); m is the number of
% measurements in each time instant (usually <= number of landmarks) and
% k is the number of time instants
    

    num_landmarks=5;

    positions = ground_truth(2:4, :); % 2-4 are position
    n_timesteps = size(positions, 2); %in case we change dt


    spacing = floor(n_timesteps / num_landmarks); 

    landmark_ind = 1:spacing:n_timesteps;

    if length(landmark_ind) > num_landmarks
        landmark_ind = landmark_ind(1:num_landmarks);
    elseif length(landmark_ind) < num_landmarks
        landmark_ind = [landmark_ind, n_timesteps]; %we are adding the final ind to make sure we have the correct nº of landmarks
    end

    % make proper landmarks vector
    M = positions(:, landmark_ind);

    Z = zeros(size(pseudo_Q,2), num_landmarks, n_timesteps);  %m is the number of measurements in each time instant (usually <= number of landmarks) and
% k is the number of time instants

    % in case there's outliers, we have to take into account the percentage
    % of outliers we want in the measurements

    outlierObsIndexes = 1: round(100/outlierSpecs.rate): n_timesteps;

    for t = 1:n_timesteps
        drone_pos = ground_truth(2:4, t);  % (tx, ty, tz)
        drone_quaternions=[ground_truth(8, t);ground_truth(5:7, t)];
        for j = 1:num_landmarks  
            distance = M(:,j) - drone_pos;
            noise =  mvnrnd(zeros(7,1),pseudo_Q);
            distance_with_noise = distance + noise(1:3)';  
            quaternions_with_noise = drone_quaternions + noise(4:end)';  % Assuming std dev of 0.1

            if outlierSpecs.on == true && any(outlierObsIndexes == t)
                distance_with_noise = distance_with_noise + randi([10 50],1,3)';
            end

            % Store noisy measurements
            Z(:, j, t) = [distance_with_noise; quaternions_with_noise];
        end
    end

    % codigo anterior
    % for t = 1:n_timesteps
    %     drone_pos = ground_truth(2:4, t);  % (tx, ty, tz)
    %     drone_quaternions=ground_truth(5:8, t);
    %     for j = 1:num_landmarks  
    %         distance = norm(M(:,j) - drone_pos);
    % 
    % 
    %         distance_with_noise = distance + randn() * 0.1;  % Assuming std dev of 0.1
    %         quaternions_with_noise = drone_quaternions + randn(4,1) * 0.1;  % Assuming std dev of 0.1
    % 
    %         % Store noisy measurements
    %         Z(:, j, t) = [distance_with_noise; quaternions_with_noise];
    %     end
    % end

end