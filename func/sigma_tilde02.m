function sigma_tilde = sigma_tilde02(p_obst,d,eta,DH,T_0_B)
%
% Computes the task function:
%   end-effector obstacke avoidance
%
% function sigma_tilde = sigma_tilde02(p_obst,d,eta,DH,T_0_B)
%
% input:
%
%       p_obst   dim 3x1     obstacle position
%       d        dim 1x1     desired distance from obstacle
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       sigma_tilde    dim 1x1    end-effector error position norm
%
% G. Antonelli, Simurv 4.0, 2013

p_obst = CheckVector(p_obst);
eta    = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);
sigma_tilde = d - sqrt((p_obst-eta_ee1)'*(p_obst-eta_ee1));
