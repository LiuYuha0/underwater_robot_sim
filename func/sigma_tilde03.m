function sigma_tilde = sigma_tilde03(eta_ee1d,eta,DH,T_0_B)
%
% Computes the task function:
%   end-effector position error
%
% function sigma_tilde = sigma_tilde03(eta_ee1d,eta,DH,T_0_B)
%
% input:
%       eta_ee1d dim 3x1     end-effector desired position
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                          matrix from vehicle to zero frame
%
% output:
%       sigma_tilde    dim 3x1    end-effector position error
%
% G. Antonelli, Simurv 4.0, 2013

eta_ee1d = CheckVector(eta_ee1d);
eta      = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);
sigma_tilde = eta_ee1d-eta_ee1;
