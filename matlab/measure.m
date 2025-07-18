function [h, H] = measure(muNom_bar, ~, j, M)
% Function that at eact time instant and for each observation and each
% landmark, computes the expected measurement and the corresponding
% jacobian.
% INPUTS:
%   muNom and muErr: see predict (they're vectors)
%   j: landmark index
%   M: see createMeasurements
% OUTPUTS:
%   h: k*1 vector, where k is the dimensionality of each measurement
%   H: k*N matrix, where k is ... and N = #state vector (see documentation)
%    Jacobian of measurement wrt to error state
%    
    
    nom_pos=muNom_bar(1:3);   %pos predicted 
    

    delta_pos=M(:,j)-nom_pos;
    h=[delta_pos; muNom_bar(7:10)];

    H_x = zeros(7, 19); 

    H_x(1:3, 1:3) = -eye(3); % -1 for x,y,z
    
    
    H_x(4, 7) = 1; 
    H_x(5, 8) = 1; 
    H_x(6, 9) = 1; 
    H_x(7, 10) = 1; 

     
%___________________________________________________________________________________________________

    I6 = eye(6); % does this match??? acho que tenho isto DEMASIADO GRANDE
    I9 = eye(9);

    q_w = muNom_bar(7); 
    q_x = muNom_bar(8); 
    q_y = muNom_bar(9); 
    q_z = muNom_bar(10); 

    Q_delta_theta = 0.5 * [
        -q_x, -q_y, -q_z;
         q_w, -q_z,  q_y;
         q_z,  q_w, -q_x;
        -q_y,  q_x,  q_w
    ];


    X_delta = blkdiag(I6, Q_delta_theta, I9);
    
    % Final Jacobian
    H = H_x * X_delta;

end