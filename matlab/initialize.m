function [muNom, muErr, sigma] = initialize(init_state, post_state, ...
    postpost_state, init_meas)
% muNom = [x,y,z,vx,vy,vz,qw,qx,qy,qx, axb, ayb, azb, wxb, wyb, wzb, gx, gy, gz]
% muErr = [dx,dy,dz,dvx,dvy,dvz, dthetax, dthetay, dthetaz, daxb, dayb, dazb, dwxb, dwyb, dwzb, dgx, dgy, dgz]
% ini_state = [time tx ty tz qx qy qz qw]

% process noise covariance matrix
%unc_process = e_x, e_y, e_theta, e_linvel, e_angvel
% unc_process(3) = unc_process(3)*pi/180; %e_theta
% unc_process(5) = unc_process(5)*pi/180; % e_velocity
% R = diag(unc_process.^2);

% initial cov. (belief)
sigma = eye(18);

% initial state (belief): nominal state
muNom = zeros(19,1);
muNom(1:3) = init_state(2:4); % position from ground truth
muNom(7:10) = [init_state(8);init_state(5:7)]; % quaternion from ground truth, GROUND TRUTH AND OUR CONVENTION FOR QUATERNIONS ARE SHIFTED!
muNom(17:19) = [ 0 0 -9.81]'; %[ 0.05630158 -9.30066998 -3.10850302]' -> body frame gravity, calibration

R = quat2rotm(muNom(7:10)'); % rotation matrix to compute velocity vector
%muNom(17:19) = R*[ 0.05630158 -9.30066998 -3.10850302]'; % gravity vector in the WF using 
dt = (post_state(1)-init_state(1));
muNom(4:6) = (post_state(2:4) - init_state(2:4))/dt; % velocity in the world frame
v_kp1 = (postpost_state(2:4) - post_state(2:4))/dt; % velocity in the next time instant (in the WF)
muNom(11:13) =  init_meas(4:6) - R'*( (v_kp1-muNom(4:6))/dt - muNom(17:19)); 
%muNom(11:13) = [0;0;0]; % accel bias to zero just to test

q_delta = quatmultiply(quatconj(muNom(7:10)'), [post_state(8);post_state(5:7)]');
omega = (2/dt)*q_delta(2:4);
muNom(14:16) = init_meas(1:3) - omega';
%muNom(14:16) = [0;0;0]; % ang vel bias to zero just to test

% initial state (belief): error state
muErr = zeros(18,1);

% noise measurement covariance matrix
%Q = unc_meas^2.*eye(2); % INSERT CORRECT DIMENSIONS LATER ON
% unc_meas(2) = unc_meas(2)*pi/180; % e_velocity
% Q = diag(unc_meas.^2);
end