function [covIMU]  = IMUNoisesCovariances(dt)
% see https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
% MODE
%   1: simulated ground truth with additional measurement and process noise
% measurement noise comes from createMeasurements, process noise is added
% in the function NoisesCovariance.
%   2: true ground truth 

f = 1000; % rate, Hz
% linear acceleration and angular velocities noise
sigma_a =  0.1; %accelerometer_noise_density
sigma_g =  0.05; % gyroscope_noise_density

% linear acceleration and angular velocities bias noise
sigma_ba = 0.002; %accelerometer_random_walk
sigma_bg = 4.0e-05; %gyroscope_random_walk

covIMU= dt.*[sigma_a^2.*eye(3), sigma_g^2.*eye(3), sigma_ba^2.*eye(3), sigma_bg^2.*eye(3)]; %,*dt

end