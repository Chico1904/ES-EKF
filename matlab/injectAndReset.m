function [muNom, muErr, sigma] = injectAndReset(muNom_bar, muErr, sigma)
% muNom = [x,y,z,vx,vy,vz,qw,qx,qy,qx, axb, ayb, azb, wxb, wyb, wzb, gx, gy, gz]
% muErr = [dx,dy,dz,dvx,dvy,dvz, dthetax, dthetay, dthetaz, daxb, dayb, dazb, dwxb, dwyb, dwzb, dgx, dgy, dgz]

% Error injection
muNom = zeros(19,1);
muNom([1:6,11:19]) = muNom_bar([1:6,11:19]) + muErr([1:6,10:18]);

normangle = norm(muErr(7:9));
if normangle > 1e-10
    q_theta = [cos(normangle/2); sin(normangle/2).*muErr(7:9)/normangle];
    muNom(7:10) = quatmultiply(muNom_bar(7:10)', q_theta');
else
   muNom(7:10) = muNom_bar(7:10);
end

% ESKF reset
muErr = muErr-muErr;
% G = eye(18); % one might use the true expression, see eq. 288
% sigma = G*sigma*G';

end