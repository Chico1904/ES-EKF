function [ground_truth, imu_meas, n_timesteps, dt] = process(dataset)


ground_truth_data = readmatrix([dataset '_groundtruth.txt']); 
imu_data = readmatrix([dataset '_imu.txt']);


gt_timestamps = ground_truth_data(:, 1); 
imu_timestamps = imu_data(:, 2); 

imu_indices = zeros(size(gt_timestamps)); % using these to find the rest of the parameters. we have LESS GT

for i = 1:length(gt_timestamps)
    [~, imu_indices(i)] = min(abs(imu_timestamps - gt_timestamps(i)));%minimum difference of all timestamps and our current timestamp
end

imu_data_aligned = imu_data(imu_indices, 2:end); %  IMU parameters aligned with the gt timestamps
imu_meas=imu_data_aligned';%to get 7x...

ground_truth=ground_truth_data'; %to get 7x...
all_data = [ground_truth_data, imu_data_aligned(:, 3:end)];


 n_timesteps = length(gt_timestamps);

 dt = mean(diff(gt_timestamps)); %time difference between consecutive timesteps




%To get dt=0.01
ground_truth = ground_truth(:, 1:5:end);
imu_meas = imu_meas(:, 1:5:end);
n_timesteps = size(ground_truth, 2);
dt = dt * 5;

% Save aligned datasets
 % writematrix(ground_truth', [dataset '_aligned_ground_truth.txt'], 'Delimiter', 'tab');
 % writematrix(imu_meas', [dataset '_aligned_imu_data.txt'], 'Delimiter', 'tab');