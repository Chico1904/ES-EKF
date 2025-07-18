function [muNom_bar, muErr_bar, sigma_bar, accel] = predict(muNom, muErr, sigma, imu_meas, covIMU, dt)
% Here the motion model is used to predict the next state. mode indicates
% the motion model being used. 
% - mode = 1: we're using the a 3D motion model in cartesian space (9
% states)
% - mode = 2: we're using the motion model of "Drone Tracking Using an 
% Innovative UKF" (see section 2). This is defined with Lee Algebra, and
% for that we need to resort to other packages to aply convenient
% operations.

% imu = [wx,wy,wz,ax,ay,az]

% ----------------------- Nominal state
% muNom = [x,y,z,vx,vy,vz,qw,qx,qy,qx, axb, ayb, azb, wxb, wyb, wzb, gx, gy, gz]
[muNom_bar, R, accel] = updateNomState(muNom, imu_meas, dt);

% ----------------------- Error state
% muErr = [dx,dy,dz,dvx,dvy,dvz, dthetax, dthetay, dthetaz, daxb, dayb, dazb, dwxb, dwyb, dwzb, dgx, dgy, dgz]
[muErr_bar, RR] = updateErrorState(muErr, muNom, imu_meas, covIMU, R, dt);

% ----------------------- Covariance
[sigma_bar] = updateCovariance(sigma, muNom, imu_meas, covIMU, R, RR, dt);



end


function [muNom_bar, R, accel] = updateNomState(muNom, u, dt)
%u = [wx,wy,wz,ax,ay,az]
% muNom = [x,y,z,vx,vy,vz,qw,qx,qy,qx, axb, ayb, azb, wxb, wyb, wzb, gx, gy, gz]
muNom_bar = zeros(19,1);
R = quat2rotm(muNom(7:10)'); % from the body frame to the world frame, same as quat2RotationM
accel = R*(u(4:6)- muNom(11:13)) + muNom(17:19); % assuming gravity vector is in the world frame

% update positions
muNom_bar(1:3) = muNom(1:3) + dt.*muNom(4:6) + 0.5.*accel.*dt^2;

% update velocities
muNom_bar(4:6) = muNom(4:6) + dt.*accel;

% update orientation/quaternion
angle = (u(1:3) - muNom(14:16)).*dt;
normangle = norm(angle);
q_omega = [cos(normangle/2); sin(normangle/2).*angle/normangle];

muNom_bar(7:10) = quatmultiply(muNom(7:10)', q_omega');
muNom_bar(7:10) = muNom_bar(7:10)/ norm(muNom_bar(7:10)); %Normalize the quaternion to ensure it remains a unit quaternion

% update lin acc and ang vel bias (and gravity vector)
muNom_bar(11:19) = muNom(11:19);
end


function [muErr_bar, RR] = updateErrorState(muErr, muNom, u, covIMU, R, dt)
%u = [wx,wy,wz,ax,ay,az]
% muErr = [dx,dy,dz,dvx,dvy,dvz, dthetax, dthetay, dthetaz, daxb, dayb, dazb, dwxb, dwyb, dwzb, dgx, dgy, dgz]

muErr_bar = zeros(18,1);
% calculate random impulses/noises
impulses = zeros(3,4);
for i = 1:4
    impulses(:,i) = mvnrnd([0,0,0], covIMU(:,3*i-2:3*i) , 1);
end

%impulses = dt.*impulse;
% impulses(:,1:6) = dt^2.*impulses(:,1:6);
% impulses(:,7:12) = dt.*impulses(:,7:12);
% update positions
muErr_bar(1:3) = muErr(1:3) + dt.*muErr(4:6);

% update velocities
muErr_bar(4:6) = muErr(4:6) + (-R*skew(u(4:6)-muNom(11:13))*muErr(7:9) - R*muErr(10:12) + muErr(16:18)).*dt +impulses(:,1);

% update angles vector
omega = u(1:3)-muNom(14:16);
theta = norm(omega .* dt); % Magnitude of the rotation vector
if theta > 1e-6 %Construct the rotation matrix R{omega * dt} using Rodrigues' formula
    k = (omega * dt) / theta; % Unit rotation axis
    K = skew(k);
    RR = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K); % Rodrigues' formula
else
    RR = eye(3); % Approximation for very small angles
end
muErr_bar(7:9) = RR'*muErr(7:9) - muErr(13:15).*dt + impulses(:,2); %check dimensions??

% update lin acc and ang vel bias (and gravity vector)
muErr_bar(10:12) = muErr(10:12) + impulses(:,3);
muErr_bar(13:15) = muErr(13:15) + impulses(:,4);
muErr_bar(16:18) = muErr(16:18);
end


function [sigma_bar] = updateCovariance(sigma, muNom, u, covIMU, R, RR, dt)
% muNom = [x,y,z,vx,vy,vz,qw,qx,qy,qx, axb, ayb, azb, wxb, wyb, wzb, gx, gy, gz]
%sigma_bar = F * sigma * F' + Q;
a_sk = skew(u(4:6)-muNom(11:13));

Fx = [eye(3,3) , eye(3,3).*dt, zeros(3,12);
     zeros(3,3), eye(3,3), -R*a_sk.*dt, -R.*dt, zeros(3,3), eye(3,3).*dt;
     zeros(3,6), RR', zeros(3,3), -eye(3,3).*dt, zeros(3,3);
     zeros(3,9),  eye(3,3), zeros(3,6);
     zeros(3,12),  eye(3,3), zeros(3,3);
     zeros(3,15), eye(3,3)]; 

Fi =  [zeros(3,12);
       eye(3,3), zeros(3, 9); 
       zeros(3, 3), eye(3,3), zeros(3, 6); 
       zeros(3, 6), eye(3,3), zeros(3, 3); 
       zeros(3, 9), eye(3,3);
       zeros(3,12)]; 

R = [covIMU(:,1:3), zeros(3,9); % the process noise
     zeros(3,3), covIMU(:,4:6), zeros(3,6);
     zeros(3,6), covIMU(:,7:9), zeros(3,3);
     zeros(3,9), covIMU(:,10:12)];

sigma_bar = Fx*sigma*Fx' + Fi*R*Fi';

end

function M = skew(u)
M = zeros(3,3);

M(3,2) = u(1);
M(2,3) = -M(3,2);

M(1,3) = u(2);
M(3,1) = -M(1,3);

M(2,1) = u(3);
M(1,2) = -M(2,1);
end

% function R = quat2RotationM(q)
% % q = [qw, qx, qy, qz]
% qw = q(1);
% qv = q(2:4);
% R = (qw^2 - qv'*qv)*eye(3) + 2*(qv*qv') + 2*qw*skew(qv);
% end





