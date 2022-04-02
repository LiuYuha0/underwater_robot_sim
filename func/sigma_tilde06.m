function sigma_tilde = sigma_tilde06(a_d,eta,DH,T_0_B)
%
% Computes the task function:
%   end-effector field-of-view error
%
% function sigma_tilde = sigma_tilde06(a_d,eta,DH,T_0_B)
%
% input:
%       a_d      dim 3x1     end-effector desired "a" versor
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       sigma_tilde    dim 1x1     end-effector field-of-view error
%
% G. Antonelli, Simurv 4.0, 2013

a_d = CheckVector(a_d);
eta = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
a  = TT(1:3,3);
sigma_tilde = 0 - sqrt((a_d-a)'*(a_d-a));
